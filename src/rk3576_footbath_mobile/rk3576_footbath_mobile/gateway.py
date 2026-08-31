"""ROS 2 mobile gateway with local-only HTTP control and fail-closed motion."""
from __future__ import annotations
import json, math, os, queue, signal, subprocess, threading, time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from .logic import ALLOWED_MODES, clamp_command, new_session_token, occupancy_png, target_is_clear, valid_pose, verify_pin
from .map_store import MapStore

ENV=Path(os.environ.get("FOOTBATH_MOBILE_ENV","/etc/footbath/mobile.env"))
def env_values(path):
    out={}
    try:
        for raw in path.read_text(encoding="utf-8").splitlines():
            line=raw.strip()
            if line and not line.startswith("#") and "=" in line:
                k,v=line.split("=",1); out[k.strip()]=v.strip()
    except OSError: pass
    return out

class Gateway(Node):
    def __init__(self):
        super().__init__("footbath_mobile_gateway")
        cfg=dict(os.environ)
        cfg.update(env_values(ENV))
        self.bind=cfg.get("FOOTBATH_MOBILE_BIND","0.0.0.0")
        self.port=int(cfg.get("FOOTBATH_MOBILE_PORT","8080"))
        self.network=cfg.get("FOOTBATH_MOBILE_NETWORK","192.168.8.")
        self.pin_salt=cfg.get("FOOTBATH_MOBILE_PIN_SALT","")
        self.pin_hash=cfg.get("FOOTBATH_MOBILE_PIN_HASH","")
        if not self.pin_salt or not self.pin_hash: raise RuntimeError("mobile PIN is not configured")
        self.ws=Path(cfg.get("FOOTBATH_WS","/home/sky/rk3576_footbath_ws")).resolve()
        self.web=Path(get_package_share_directory("rk3576_footbath_mobile"))/"web"
        self.session=None; self.session_expiry=0.0; self.last_heartbeat=0.0; self.session_lost=True
        self.failed_logins=0; self.login_block_until=0.0
        self.mode="idle"; self.child=None; self.child_pgid=None; self.map_name=""; self.save_state="idle"
        self.nav_state="idle"; self.nav_message=""; self.nav_feedback=None; self.goal_pose=None
        self.initialized=False; self.initial_state="not_set"; self.initial_request=None; self.initial_request_at=0.0; self.last_amcl_pose_at=0.0
        self.transitioning=False; self.mode_started_at=0.0; self.launch_error=""
        self.latest={"scan":None,"odom":None,"map":None}; self.grid=None; self.grid_png=None; self.pose_map=None
        # Cache expensive ROS graph/filesystem queries; HTTP state polling must not
        # trigger graph discovery on every request.
        self.last_pose_lookup=0.0
        self.map_store=MapStore((
            self.ws/"maps",
            self.ws/"src/rk3576_footbath_navigation/maps"))
        self.map_render_queue=queue.Queue(maxsize=1); self.render_stop=threading.Event()
        self.manual_pub=None; self.manual_active=False
        self.commands=queue.Queue()
        transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_buffer=Buffer(); self.tf_listener=TransformListener(self.tf_buffer,self)
        self.map_sub=self.create_subscription(OccupancyGrid,"/map",self._map,transient)
        self.create_subscription(LaserScan,"/scan_high",lambda m:self._seen("scan"),10)
        self.create_subscription(Odometry,"/odom",lambda m:self._seen("odom"),20)
        self.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",self._amcl_pose,10)
        self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,"/initialpose",10)
        self.nav=ActionClient(self,NavigateToPose,"/navigate_to_pose")
        self.cancel_client=self.create_client(CancelGoal,"/navigate_to_pose/_action/cancel_goal")
        self.exp_clients={n:self.create_client(Trigger,f"/exploration/{n}") for n in ("start","stop","save_map")}
        self.create_timer(0.05,self._process)
        self.create_timer(0.10,self._watchdog)
        threading.Thread(target=self._render_maps,name="footbath-map-render",daemon=True).start()
        self.http=ThreadingHTTPServer((self.bind,self.port),self._handler())
        self.http.daemon_threads=True
        threading.Thread(target=self.http.serve_forever,daemon=True).start()
        self.get_logger().info(f"mobile UI listening on {self.bind}:{self.port}")

    def _seen(self,key): self.latest[key]=time.monotonic()
    def _map(self,msg):
        self.latest["map"]=time.monotonic(); self.grid=msg
        item=(msg.data,msg.info.width,msg.info.height)
        try: self.map_render_queue.put_nowait(item)
        except queue.Full:
            try: self.map_render_queue.get_nowait()
            except queue.Empty: pass
            try: self.map_render_queue.put_nowait(item)
            except queue.Full: pass
    def _amcl_pose(self,msg):
        now=time.monotonic(); self.last_amcl_pose_at=now
        if self.mode=="navigation" and self.initial_state=="waiting" and now>=self.initial_request_at:
            self.initialized=True; self.initial_state="confirmed"
            self.nav_state="localized"; self.nav_message="AMCL已确认初始位姿"

    def _render_maps(self):
        while not self.render_stop.is_set():
            try: data,width,height=self.map_render_queue.get(timeout=0.5)
            except queue.Empty: continue
            try: self.grid_png=occupancy_png(data,width,height)
            except ValueError: self.grid_png=None
    def _raw_healthy(self,key):
        t=self.latest.get(key)
        if t is None: return False
        if key=="map" and self.mode=="navigation":
            # The static map is transient-local and normally arrives once. A
            # received grid plus a live launch child is a stronger and much
            # cheaper health signal than polling the DDS graph.
            return (
                self.grid is not None
                and self.child is not None
                and self.child.poll() is None
            )
        max_age=6.0 if key=="map" else 2.5
        return time.monotonic()-t<=max_age
    def health_state(self,key):
        if self.mode=="idle": return "inactive"
        if self._raw_healthy(key): return "ok"
        if self.transitioning or time.monotonic()-self.mode_started_at<15.0: return "starting"
        return "missing" if self.latest.get(key) is None else "stale"
    def healthy(self,key,age=None):
        if age is not None:
            t=self.latest.get(key); return t is not None and time.monotonic()-t<=age
        return self._raw_healthy(key)
    def authorized(self,token):
        return token and token==self.session and time.monotonic()<self.session_expiry
    def login(self,pin):
        now=time.monotonic()
        if now < self.login_block_until: return None
        if not verify_pin(pin,self.pin_salt,self.pin_hash):
            self.failed_logins+=1
            if self.failed_logins>=5:
                self.login_block_until=now+30.0; self.failed_logins=0
            return None
        self.failed_logins=0; self.login_block_until=0.0
        self.session=new_session_token(); now=time.monotonic(); self.session_expiry=now+900; self.last_heartbeat=now; self.session_lost=False
        return self.session
    def heartbeat(self):
        now=time.monotonic()
        self.last_heartbeat=now; self.session_expiry=now+900; self.session_lost=False
    def _release_manual(self):
        if self.manual_pub:
            self.manual_pub.publish(Twist()); self.destroy_publisher(self.manual_pub); self.manual_pub=None
        self.manual_active=False
    def _cancel_nav(self):
        if self.cancel_client.service_is_ready(): self.cancel_client.call_async(CancelGoal.Request())
        self.nav_state="cancel_requested"; self.nav_message="已请求取消导航"
    def _service(self,name):
        client=self.exp_clients[name]
        if client.service_is_ready(): client.call_async(Trigger.Request()); return True
        return False
    def _finish_service_call(self,name,future,event,box):
        if box.get("cancelled"):
            return
        try:
            response=future.result()
            if response is None:
                box["error"]=f"{name} service returned no response"
            elif not response.success:
                box["error"]=response.message or f"{name} service rejected request"
            else:
                box["result"]={"ok":True,"message":response.message or f"{name} completed"}
        except Exception as error:
            box["error"]=f"{name} service failed: {error}"
        finally:
            event.set()
    def _group_exists(self,pgid):
        if pgid is None: return False
        try: os.killpg(pgid,0); return True
        except ProcessLookupError: return False
        except PermissionError: return True
    def _stop_child_group(self):
        pgid=self.child_pgid
        if pgid is None: return
        for sig,wait_s in ((signal.SIGINT,3.0),(signal.SIGTERM,2.0),(signal.SIGKILL,0.5)):
            if not self._group_exists(pgid): break
            try: os.killpg(pgid,sig)
            except ProcessLookupError: break
            deadline=time.monotonic()+wait_s
            while self._group_exists(pgid) and time.monotonic()<deadline: time.sleep(0.05)
    def _watchdog(self):
        now=time.monotonic()
        if now-self.last_pose_lookup>=0.20:
            self.last_pose_lookup=now
            if self.healthy("map"):
                try:
                    tf=self.tf_buffer.lookup_transform("map","base_footprint",Time())
                    q=tf.transform.rotation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    self.pose_map={"x":tf.transform.translation.x,"y":tf.transform.translation.y,"yaw":yaw}
                except TransformException:
                    self.pose_map=None
            else:
                self.pose_map=None
        if self.initial_state=="waiting" and time.monotonic()-self.initial_request_at>8.0:
            self.initialized=False; self.initial_state="failed"
            self.nav_state="localization_failed"; self.nav_message="AMCL在8秒内未确认初始位姿，请重新设置"
        if self.child and self.child.poll() is not None:
            failed_mode=self.mode; returncode=self.child.returncode
            self.get_logger().error(f"{failed_mode} launch exited: {returncode}; cleaning process group {self.child_pgid}")
            self._stop_child_group()
            self.child=None; self.child_pgid=None; self.mode="idle"; self.transitioning=False
            self.launch_error=f"{failed_mode} launch exited: {returncode}"
            self._release_manual()
        if self.transitioning and self.child and self.child.poll() is None:
            if all(self._raw_healthy(k) for k in ("scan","odom","map")):
                self.transitioning=False; self.launch_error=""
            elif time.monotonic()-self.mode_started_at>20.0:
                self.transitioning=False; self.launch_error="startup health timeout"
        loss_timeout=2.5
        if self.session and time.monotonic()-self.last_heartbeat>loss_timeout and not self.session_lost:
            was_manual=self.manual_active
            self.session_lost=True
            self._release_manual()
            detail="manual teleop released" if was_manual else "no manual teleop was active"
            self.get_logger().warning(
                f"mobile heartbeat lost; {detail}; autonomous task remains under local supervision")
        if self.manual_active and time.monotonic()-self.last_heartbeat>0.30: self._release_manual()
    def _stop_child(self):
        self._release_manual()
        if self.mode=="auto_mapping": self._service("stop")
        if self.mode=="navigation": self._cancel_nav()
        self._stop_child_group()
        if self.child:
            try: self.child.wait(timeout=0.2)
            except subprocess.TimeoutExpired: pass
        self.child=None; self.child_pgid=None; self.mode="idle"; self.initialized=False; self.pose_map=None
        self.initial_state="not_set"; self.initial_request=None; self.initial_request_at=0.0
        self.nav_state="idle"; self.nav_message=""; self.nav_feedback=None; self.goal_pose=None
    def _launch(self,mode,map_path=""):
        if mode not in ALLOWED_MODES: raise ValueError("invalid mode")
        launch=["ros2","launch","rk3576_footbath_bringup"]
        requested_map=""
        if mode=="mapping": launch+=["mapping.launch.py","headless:=true"]
        elif mode=="auto_mapping": launch+=["auto_mapping.launch.py","headless:=true","start_explorer:=true"]
        elif mode=="navigation":
            candidate=self.map_store.checked_yaml(map_path)
            requested_map=str(candidate); launch+=["navigation.launch.py","headless:=true",f"map:={candidate}"]
        if mode==self.mode and self.child and self.child.poll() is None and (mode!="navigation" or requested_map==self.map_name):
            self.get_logger().info(f"ignoring duplicate mode request: {mode}")
            return
        self.transitioning=True; self.launch_error=""
        self._stop_child()
        self.latest={"scan":None,"odom":None,"map":None}; self.grid=None; self.grid_png=None; self.pose_map=None
        if mode=="idle": self.transitioning=False; return
        env=os.environ.copy()
        self.child=subprocess.Popen(launch,cwd=self.ws,env=env,start_new_session=True)
        self.child_pgid=os.getpgid(self.child.pid)
        self.mode=mode; self.map_name=requested_map; self.nav_state="idle"; self.nav_message=""
        self.nav_feedback=None; self.goal_pose=None; self.initialized=False
        self.initial_state="not_set"; self.initial_request=None; self.initial_request_at=0.0
        self.mode_started_at=time.monotonic(); self.transitioning=True
        self.get_logger().info(f"starting mode={mode}, pid={self.child.pid}, pgid={self.child_pgid}")
    def maps(self):
        return self.map_store.paths()
    def _save_manual(self):
        if self.mode!="mapping": raise ValueError("manual map save requires mapping mode")
        prefix=self.ws/"src/rk3576_footbath_navigation/maps"/time.strftime("footbath_mobile_%Y%m%d_%H%M%S")
        subprocess.Popen([str(self.ws/"scripts/save_map.sh"),str(prefix)],cwd=self.ws,start_new_session=True)
        self.save_state=f"requested: {prefix}"
    def _teleop(self,d):
        if self.mode!="mapping": raise ValueError("teleop requires mapping mode")
        linear,angular=clamp_command(float(d.get("linear",0)),float(d.get("angular",0)))
        active=bool(d.get("active",False))
        if not active: self._release_manual(); return
        if self.manual_pub is None:
            if self.count_publishers("/cmd_vel_manual")>0: raise ValueError("another ROS manual publisher is active")
            self.manual_pub=self.create_publisher(Twist,"/cmd_vel_manual",10)
        msg=Twist(); msg.linear.x=linear; msg.angular.z=angular; self.manual_pub.publish(msg)
        self.manual_active=True; self.last_heartbeat=time.monotonic()
    def _initial(self,d):
        if self.mode!="navigation": raise ValueError("initial pose requires navigation mode")
        if self.count_subscribers("/initialpose")==0:
            raise ValueError("AMCL is not ready; wait a few seconds and retry")
        x,y,yaw=map(float,(d["x"],d["y"],d["yaw"]))
        if not valid_pose(x,y,yaw): raise ValueError("invalid pose")
        msg=PoseWithCovarianceStamped(); msg.header.frame_id="map"
        # Zero means "latest available transform" and avoids AMCL rejecting a
        # phone request that is a few milliseconds newer than odom->base TF.
        msg.header.stamp.sec=0; msg.header.stamp.nanosec=0
        msg.pose.pose.position.x=x; msg.pose.pose.position.y=y
        msg.pose.pose.orientation.z=math.sin(yaw/2); msg.pose.pose.orientation.w=math.cos(yaw/2)
        msg.pose.covariance[0]=0.25; msg.pose.covariance[7]=0.25; msg.pose.covariance[35]=0.0685
        self.initialized=False; self.initial_state="waiting"
        self.initial_request={"x":x,"y":y,"yaw":yaw}; self.initial_request_at=time.monotonic()
        self.nav_state="localizing"; self.nav_message="初始位姿已发送，等待AMCL确认"
        self.nav_feedback=None; self.goal_pose=None
        self.initial_pub.publish(msg)
    def _goal(self,d):
        if self.mode!="navigation" or not self.initialized:
            raise ValueError("navigation is not localized; wait for AMCL confirmation")
        if not all(self.healthy(k) for k in ("scan","odom","map")): raise ValueError("scan/map/odom is stale")
        if self.pose_map is None: raise ValueError("map to base_footprint transform is unavailable")
        x,y,yaw=map(float,(d["x"],d["y"],d["yaw"]))
        if not valid_pose(x,y,yaw) or self.grid is None: raise ValueError("invalid goal")
        info=self.grid.info
        if not target_is_clear(self.grid.data,info.width,info.height,info.resolution,info.origin.position.x,info.origin.position.y,x,y):
            raise ValueError("goal is not in a clear known area")
        if not self.nav.server_is_ready(): raise ValueError("NavigateToPose is unavailable")
        goal=NavigateToPose.Goal(); goal.pose=PoseStamped(); goal.pose.header.frame_id="map"; goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=x; goal.pose.pose.position.y=y
        goal.pose.pose.orientation.z=math.sin(yaw/2); goal.pose.pose.orientation.w=math.cos(yaw/2)
        self.goal_pose={"x":x,"y":y,"yaw":yaw}; self.nav_feedback=None
        self.nav_state="sending"; self.nav_message="目标已发送，等待Nav2接受"
        future=self.nav.send_goal_async(goal,feedback_callback=self._goal_feedback)
        future.add_done_callback(self._goal_response)
    def _goal_response(self,future):
        try: handle=future.result()
        except Exception as error:
            self.nav_state="error"; self.nav_message=f"目标发送失败: {error}"; return
        if not handle.accepted:
            self.nav_state="rejected"; self.nav_message="Nav2拒绝目标点"; return
        self.goal_handle=handle; self.nav_state="active"; self.nav_message="Nav2已接受目标，正在导航"
        result=handle.get_result_async(); result.add_done_callback(self._goal_result)
    def _goal_feedback(self,message):
        feedback=message.feedback
        eta=feedback.estimated_time_remaining
        self.nav_feedback={
            "distance_remaining_m":round(float(feedback.distance_remaining),3),
            "eta_s":round(float(eta.sec)+float(eta.nanosec)/1.0e9,1),
            "recoveries":int(feedback.number_of_recoveries),
        }
        if self.nav_state not in ("cancel_requested","canceled"):
            self.nav_state="active"; self.nav_message="正在导航"
    def _goal_result(self,future):
        try: status=future.result().status
        except Exception as error:
            self.nav_state="error"; self.nav_message=f"读取导航结果失败: {error}"; return
        states={
            GoalStatus.STATUS_SUCCEEDED:("succeeded","已到达目标点"),
            GoalStatus.STATUS_CANCELED:("canceled","导航已取消"),
            GoalStatus.STATUS_ABORTED:("aborted","导航失败，Nav2已中止"),
        }
        self.nav_state,self.nav_message=states.get(status,(f"result_{status}",f"导航结束，状态码 {status}"))
    def _process(self):
        for _ in range(20):
            deferred=False
            try: name,data,event,box,deadline=self.commands.get_nowait()
            except queue.Empty: break
            if box.get("cancelled") or time.monotonic()>deadline:
                box["error"]="command expired"; event.set(); continue
            try:
                if name=="mode": self._launch(data["mode"],data.get("map",""))
                elif name=="teleop": self._teleop(data)
                elif name=="initial": self._initial(data)
                elif name=="goal": self._goal(data)
                elif name=="cancel": self._cancel_nav()
                elif name=="emergency_stop": self._stop_child()
                elif name=="save_manual": self._save_manual()
                elif name=="map_rename":
                    if self.mode!="idle":
                        raise ValueError("请先结束当前建图或导航任务")
                    new_path=self.map_store.rename(data.get("path",""),data.get("name",""))
                    box["result"]={"ok":True,"message":f"地图已重命名为 {Path(new_path).stem}"}
                elif name=="map_delete":
                    if self.mode!="idle":
                        raise ValueError("请先结束当前建图或导航任务")
                    trash_path=self.map_store.trash(data.get("path",""))
                    box["result"]={"ok":True,"message":f"地图已移入回收站：{trash_path}"}
                elif name in ("start","stop","save_map"):
                    client=self.exp_clients[name]
                    if not client.service_is_ready():
                        raise ValueError(f"/exploration/{name} service unavailable")
                    future=client.call_async(Trigger.Request())
                    future.add_done_callback(
                        lambda completed,n=name,e=event,b=box:
                        self._finish_service_call(n,completed,e,b))
                    deferred=True
                if not deferred:
                    box.setdefault("result",{"ok":True})
            except Exception as e: box["error"]=str(e)
            finally:
                if not deferred: event.set()
    def execute(self,name,data):
        if name=="teleop": timeout=0.5
        elif name in ("mode","stop","cancel","emergency_stop"): timeout=20.0
        else: timeout=4.0
        event=threading.Event(); box={"cancelled":False}; deadline=time.monotonic()+timeout
        self.commands.put((name,data,event,box,deadline))
        if not event.wait(timeout):
            box["cancelled"]=True; raise TimeoutError("ROS command timeout")
        if "error" in box: raise ValueError(box["error"])
        return box.get("result",{"ok":True})
    def state(self):
        now=time.monotonic()
        session_timeout=3.0 if self.mode in ("auto_mapping","navigation") else 2.5
        ages={k:(None if t is None else round(now-t,3)) for k,t in self.latest.items()}
        return {"mode":self.mode,"map_name":self.map_name,"nav_state":self.nav_state,
         "nav_message":self.nav_message,"nav_feedback":self.nav_feedback,"goal":self.goal_pose,
         "initialized":self.initialized,"initial_state":self.initial_state,
         "initial_request":self.initial_request,
         "transitioning":self.transitioning,"launch_error":self.launch_error,
         "session_connected":bool(self.session and now-self.last_heartbeat<=session_timeout),
         "healthy":{k:self.healthy(k) for k in self.latest},
         "health_state":{k:self.health_state(k) for k in self.latest},"health_age_s":ages,
         "manual_active":self.manual_active,"maps":self.maps(),
         "map_entries":self.map_store.entries(self.map_name if self.mode=="navigation" else ""),"save_state":self.save_state,"pose":self.pose_map,
         "map":None if self.grid is None else {"width":self.grid.info.width,"height":self.grid.info.height,
          "resolution":self.grid.info.resolution,"origin_x":self.grid.info.origin.position.x,"origin_y":self.grid.info.origin.position.y}}
    def _handler(self):
        gateway=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,fmt,*args):
                # Successful heartbeat/state/map polling is expected and used to
                # flood journald several times per second. Keep only HTTP errors.
                try: status=int(args[1])
                except (IndexError,TypeError,ValueError): status=500
                if status>=400: gateway.get_logger().warning("http "+fmt%args)
            def allowed(self): return self.client_address[0].startswith(gateway.network) or self.client_address[0] in ("127.0.0.1","::1")
            def token(self): return self.headers.get("Authorization","").removeprefix("Bearer ").strip()
            def body(self):
                n=int(self.headers.get("Content-Length","0"))
                if n>65536: raise ValueError("request too large")
                return json.loads(self.rfile.read(n) or b"{}")
            def reply(self,status,payload,ctype="application/json"):
                raw=payload if isinstance(payload,bytes) else json.dumps(payload,ensure_ascii=False).encode()
                self.send_response(status)
                self.send_header("Content-Type",ctype)
                self.send_header("Cache-Control","no-store")
                self.send_header("X-Content-Type-Options","nosniff")
                self.send_header("X-Frame-Options","DENY")
                self.send_header("Referrer-Policy","no-referrer")
                self.send_header("Content-Security-Policy","default-src 'self'; img-src 'self' blob:; object-src 'none'; frame-ancestors 'none'; base-uri 'none'")
                self.send_header("Content-Length",str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            def do_GET(self):
                if not self.allowed(): return self.reply(HTTPStatus.FORBIDDEN,{"error":"local G806P network only"})
                parsed=urlparse(self.path)
                path=parsed.path
                if path=="/": return self.reply(200,(gateway.web/"index.html").read_bytes(),"text/html; charset=utf-8")
                if path=="/app.js": return self.reply(200,(gateway.web/"app.js").read_bytes(),"text/javascript; charset=utf-8")
                if path=="/style.css": return self.reply(200,(gateway.web/"style.css").read_bytes(),"text/css")
                if not gateway.authorized(self.token()): return self.reply(401,{"error":"login required"})
                if path=="/api/state": return self.reply(200,gateway.state())
                if path=="/api/map.png" and gateway.grid_png: return self.reply(200,gateway.grid_png,"image/png")
                if path=="/api/maps/preview":
                    try:
                        raw_path=parse_qs(parsed.query).get("path",[""])[0]
                        data,width,height=gateway.map_store.preview_data(raw_path)
                        preview=occupancy_png(data,width,height)
                        return self.reply(200,preview,"image/png")
                    except Exception as e: return self.reply(409,{"error":str(e)})
                return self.reply(404,{"error":"not found"})
            def do_POST(self):
                if not self.allowed(): return self.reply(403,{"error":"local G806P network only"})
                try: data=self.body()
                except Exception as e: return self.reply(400,{"error":str(e)})
                path=urlparse(self.path).path
                if path=="/api/login":
                    token=gateway.login(str(data.get("pin","")))
                    return self.reply(200,{"token":token}) if token else self.reply(403,{"error":"invalid PIN"})
                if not gateway.authorized(self.token()): return self.reply(401,{"error":"login required"})
                gateway.heartbeat()
                if path=="/api/heartbeat": return self.reply(200,{"ok":True})
                routes={"/api/mode":"mode","/api/teleop":"teleop","/api/initial_pose":"initial","/api/goal":"goal","/api/cancel":"cancel","/api/emergency_stop":"emergency_stop","/api/save_manual":"save_manual","/api/exploration/start":"start","/api/exploration/stop":"stop","/api/exploration/save":"save_map","/api/maps/rename":"map_rename","/api/maps/delete":"map_delete"}
                name=routes.get(path)
                if not name: return self.reply(404,{"error":"not found"})
                try: return self.reply(200,gateway.execute(name,data))
                except Exception as e: return self.reply(409,{"error":str(e)})
        return Handler
    def stop(self):
        self.render_stop.set(); self.http.shutdown(); self._stop_child()

def main(args=None):
    rclpy.init(args=args); node=Gateway()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        node.stop(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
