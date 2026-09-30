let token="",state=null,pick=null,mapImage=null,lastMapLoad=0,loadingMap=false,modeRequest=false,mapManagerSignature="",savedMapPreview=null;
let dragStart=null,previewPose=null,panDrag=null;
const mapView={zoom:1,rotation:0,panX:0,panY:0};
const $=s=>document.querySelector(s);
function requireLogin(message="请重新登录"){
  token="";state=null;pick=null;dragStart=null;previewPose=null;mapImage=null;savedMapPreview=null;loadingMap=false;
  $("#app").hidden=true;$("#login").hidden=false;$("#link").textContent=message;
  $(".canvasWrap").classList.remove("picking");
}
async function api(path,data,method="POST"){
  const r=await fetch(path,{method,headers:{"Content-Type":"application/json","Authorization":"Bearer "+token},body:method==="POST"?JSON.stringify(data||{}):undefined});
  const j=r.headers.get("content-type")?.includes("json")?await r.json():{};
  if(r.status===401){requireLogin("登录已过期");throw Error("登录已过期，请重新登录");}
  if(!r.ok)throw Error(j.error||r.statusText);return j;
}
$("#loginBtn").onclick=async()=>{try{token=(await api("/api/login",{pin:$("#pin").value})).token;$("#login").hidden=true;$("#app").hidden=false;$("#link").textContent="已连接";loop()}catch(e){alert(e.message)}};
function showTab(id){document.querySelectorAll(".tab").forEach(x=>x.hidden=true);$("#"+id).hidden=false;document.querySelectorAll("nav button").forEach(x=>x.classList.toggle("active",x.dataset.tab===id));}
document.querySelectorAll("nav button").forEach(b=>b.onclick=()=>showTab(b.dataset.tab));
async function loop(){if(!token)return;try{await api("/api/heartbeat",{});if(!token)return;state=await api("/api/state",null,"GET");$("#link").textContent="已连接";renderState()}catch(e){if(token){$("#link").textContent="连接中断";setStatus("mappingCountdown","连接中断，等待计时同步","wait");}if(state?.obstacle_view){state.obstacle_view.fresh=false;state.obstacle_view.editable=false;draw();}}if(token)setTimeout(loop,1000)}
function setStatus(id,text,kind){const el=$("#"+id);el.textContent=text;el.className=kind||"";}
function mappingCountdownText(current){
  if(current.mode!=="auto_mapping")return "未启动自动建图";
  const home=current.home||{},timer=home.mapping_timer;
  if(!current.home_status_fresh || !timer)return "等待计时数据";
  if(home.supervisor_state==="complete")return "建图已完成";
  if(home.supervisor_state==="exploration_timeout")return "00:00｜已达建图时限";
  if(!Number.isFinite(timer.limit_s)||!Number.isFinite(timer.remaining_s))return "等待计时数据";
  const format=seconds=>{const n=Math.max(0,Math.ceil(seconds));return `${String(Math.floor(n/60)).padStart(2,"0")}:${String(n%60).padStart(2,"0")}`;};
  if(!timer.started)return `待开始｜总时限 ${format(timer.limit_s)}`;
  const suffix=home.supervisor_state==="paused_operator"?"｜手动暂停仍计时":
    home.supervisor_state==="hazard_recovery"?"｜恢复中，以任务计时为准":
    home.supervisor_state==="paused_fault"?"｜故障停车": "";
  return `${format(timer.remaining_s)} / ${format(timer.limit_s)}${suffix}`;
}
function renderState(){
  setStatus("mappingCountdown",mappingCountdownText(state),"wait");
  const sideOn=state.side_ultrasonic_enabled!==false;
  $("#sideSonarToggle").disabled=state.mode!=="idle"||state.transitioning||modeRequest;
  $("#sideSonarToggle").textContent=sideOn?"关闭左右超声波保护":"开启左右超声波保护";
  $("#sideSonarState").textContent=`已选择：${sideOn?"开启":"关闭"}；`+(state.side_ultrasonic_actual==null?"等待下次任务/底盘确认":(state.side_ultrasonic_actual===sideOn?"底盘已确认":"底盘状态不一致，禁止 RK 运动"));
  const modes={idle:"空闲",mapping:"手动建图",auto_mapping:"自动建图",navigation:"自动导航"};
  let modeText=modes[state.mode]||state.mode;
  if(state.transitioning)modeText+="（启动中）";
  if(state.launch_error)modeText+="｜故障："+state.launch_error;
  setStatus("modeText",modeText,state.launch_error?"bad":(state.transitioning?"wait":"ok"));
  const mappingActive=["mapping","auto_mapping"].includes(state.mode);
  const home=state.home||{};
  if(state.home_status_fresh && ["hazard_recovery","paused_fault"].includes(home.supervisor_state)) {
    setStatus("modeText",modeText+"｜"+(home.supervisor_reason||"恢复已暂停，请人工处理"),
              home.supervisor_state==="paused_fault"?"bad":"wait");
  }
  if(state.home_status_fresh && home.supervisor_state==="complete") {
    setStatus("modeText",modeText+"｜"+(home.supervisor_reason||"可达区域探索完成"),"ok");
  }
  if(state.home_status_fresh && home.supervisor_state==="running" && state.mode==="auto_mapping") {
    const stages={reevaluating_frontier:"路线跟随无进展，停车检查起步空间并重新规划",waiting_exploration_health:"等待定位、地图或健康许可恢复",confirming_exploration_complete:"暂无可达探索点，持续确认30秒后完成可达区域建图",waiting_costmap:"等待导航代价地图更新",waiting_reachable_frontier:"筛选可达探索点，连续失败将尝试安全退出",selecting_frontier:"正在选择探索点",aligning_frontier:"起步准备：对齐路径朝向",waiting_frontier_retry:"其他探索点已处理，等待受阻区域有限重试",following_frontier:"正在前往探索点",escape_requested:"正在准备退出死胡同",navigation_recovery:"导航正在恢复（清图、等待或短退）"};
    setStatus("modeText",modeText+"｜"+(["reevaluating_frontier","waiting_exploration_health","confirming_exploration_complete","navigation_recovery","escape_requested","selecting_frontier","aligning_frontier","waiting_frontier_retry","waiting_costmap","waiting_reachable_frontier"].includes(home.explore_status)?stages[home.explore_status]:(home.motion_stage||stages[home.explore_status]||"正在探索")),"ok");
  }
  const returning=["preparing","sending","returning","waiting_health","saving_map","handoff_ready","undocking","docking","dock_waiting","dock_preparing","aligning","hazard_suspended"].includes(home.phase);
  let homeText=state.home_status_fresh?(home.message||"等待起点"):"返航状态未就绪 / 已失联";
  if(home.pose)homeText+=`｜基站 (${home.pose.x.toFixed(2)}, ${home.pose.y.toFixed(2)})`;
  if(Number.isFinite(home.distance_remaining_m)&&returning)homeText+=`｜剩余 ${home.distance_remaining_m.toFixed(2)} m`;
  if(state.nav_message && (!state.home_status_fresh || home.phase==="ready"))homeText+=`｜${state.nav_message}`;
  $("#homeStatus").textContent=homeText;
  $("#returnHome").disabled=!(mappingActive||state.mode==="navigation")||!state.home_status_fresh||!home.available||returning;
  const initialTarget=state.confirmed_initial_pose;
  $("#returnInitial").disabled=state.mode!=="navigation"||!initialTarget||!state.initialized||state.transitioning||returning;
  $("#initialReturnStatus").textContent=initialTarget?`本次手动初始化位置 (${initialTarget.x.toFixed(2)}, ${initialTarget.y.toFixed(2)})，朝向 ${(initialTarget.yaw*180/Math.PI).toFixed(1)}°；普通导航返回，不用于倒车入基站。`:"尚未手动设置并确认本次初始位姿；自动采用建图起点定位不会创建此返回点。";
  $("#cancelHome").disabled=!(mappingActive||state.mode==="navigation");
  $("#pauseMapping").disabled=!mappingActive;
  $("#finishManualHome").disabled=state.mode!=="mapping"||!state.home_status_fresh||!home.available||returning;
  const sourceSelect=$("#mappingScanSource");
  if(mappingActive||!sourceSelect.dataset.initialized){
    sourceSelect.value=mappingActive?state.mapping_scan_source:(state.mapping_scan_default||"fused");
    sourceSelect.dataset.initialized="1";
  }
  const sourceLabels={fused:"双雷达融合",high:"高位单雷达"};
  setStatus("scanSource",sourceLabels[mappingActive?state.mapping_scan_source:(state.mapping_scan_default||"fused")]||"--",mappingActive?"ok":"wait");
  const labels={ok:"正常",starting:"启动中",inactive:"未运行",missing:"无数据",stale:"数据超时"};
  for(const [id,k] of [["odom","odom"],["scan","scan"],["mapHealth","map"]]){
    const health=state.health_state?.[k]||(state.healthy?.[k]?"ok":"stale");
    setStatus(id,labels[health]||health,health==="ok"?"ok":((health==="starting"||health==="inactive")?"wait":"bad"));
  }
  const initialLabels={not_set:"未设置",waiting:"等待AMCL确认",confirmed:"已确认",failed:"确认失败"};
  const initialKind=state.initial_state==="confirmed"?"ok":(state.initial_state==="failed"?"bad":"wait");
  setStatus("localization",initialLabels[state.initial_state]||state.initial_state,initialKind);
  const navLabels={idle:"未运行",localizing:"等待定位",localized:"定位完成",sending:"发送目标",active:"导航中",cancel_requested:"取消中",canceled:"已取消",succeeded:"已到达",aborted:"导航失败",rejected:"目标被拒绝",error:"导航错误",localization_failed:"定位失败"};
  let navText=state.nav_message||navLabels[state.nav_state]||state.nav_state;
  if(state.initial_source==="saved_home")navText+="｜使用建图起点初始化：实车必须位于该点且朝向一致，否则请重新设置位姿";
  if(state.mode==="navigation"&&!state.saved_map_home&&state.initial_state==="not_set")navText+="｜此地图无有效起点记录，请手动初始化";
  if(state.nav_feedback){navText+=`｜剩余 ${state.nav_feedback.distance_remaining_m.toFixed(2)}m，预计 ${state.nav_feedback.eta_s.toFixed(1)}s，恢复 ${state.nav_feedback.recoveries} 次`;}
  const navBad=["aborted","rejected","error","localization_failed"].includes(state.nav_state);
  const navOk=["localized","succeeded"].includes(state.nav_state);
  setStatus("navStatus",navText,navBad?"bad":(navOk?"ok":"wait"));
  $("#navDetail").textContent=navText;
  const busy=modeRequest||state.transitioning;
  sourceSelect.disabled=busy||mappingActive;
  $("#startMapping").disabled=busy||state.mode==="mapping";
  $("#startAuto").disabled=busy||state.mode==="auto_mapping";
  $("#startNav").disabled=busy||state.mode==="navigation";
  $("#setInitial").disabled=busy||state.mode!=="navigation"||!state.healthy?.map;
  $("#setGoal").disabled=busy||state.mode!=="navigation"||!state.initialized;
  const select=$("#maps"),signature=state.maps.join("\n"),selected=select.value;
  if(select.dataset.signature!==signature){
    select.replaceChildren(...state.maps.map(path=>{const option=document.createElement("option");option.value=path;option.textContent=path.split("/").pop();return option}));
    select.dataset.signature=signature;if(state.maps.includes(selected))select.value=selected;
  }
  renderMapManager();
  $("#mapInfo").textContent=savedMapPreview?(`已保存地图：${savedMapPreview.name}  ${savedMapPreview.width}×${savedMapPreview.height}`):(state.map?(`实时地图 ${state.map.width}×${state.map.height}  分辨率 ${state.map.resolution}`):"等待地图...");
  if(!savedMapPreview&&state.map&&Date.now()-lastMapLoad>4000&&!loadingMap)loadMap();
  draw();
}
function formatBytes(bytes){if(bytes<1024)return bytes+" B";if(bytes<1048576)return (bytes/1024).toFixed(1)+" KB";return (bytes/1048576).toFixed(1)+" MB";}
function renderMapManager(){
  const list=$("#mapManagerList"),entries=state?.map_entries||[];
  const signature=JSON.stringify([state?.mode,entries.map(item=>[item.path,item.modified_ms,item.size_bytes,item.complete,item.active])]);
  if(signature===mapManagerSignature)return;mapManagerSignature=signature;list.replaceChildren();
  if(!entries.length){const empty=document.createElement("p");empty.textContent="还没有已保存地图";list.append(empty);return}
  for(const entry of entries){
    const record=document.createElement("article");record.className="mapRecord";
    const title=document.createElement("button");title.type="button";title.className="mapNameButton";title.textContent=entry.name+(entry.active?"（导航使用中）":"");title.onclick=()=>viewSavedMap(entry);
    const meta=document.createElement("p");meta.className="mapMeta";
    meta.textContent=`${entry.source}｜${new Date(entry.modified_ms).toLocaleString()}｜${formatBytes(entry.size_bytes)}｜${entry.has_posegraph?"含位姿图":"栅格地图"}${entry.complete?"":"｜文件不完整"}`;
    const actions=document.createElement("div");actions.className="mapActions";
    const use=document.createElement("button");use.type="button";use.textContent="用于导航";
    use.onclick=()=>{$("#maps").value=entry.path;showTab("nav");showActionFeedback("已选择地图："+entry.name+"，尚未启动导航","ok")};
    const rename=document.createElement("button");rename.type="button";rename.textContent="重命名";rename.disabled=state.mode!=="idle";
    rename.onclick=async()=>{const name=prompt("输入新地图名称（不用输入.yaml）",entry.name);if(name===null||name.trim()===entry.name)return;await manageMap("/api/maps/rename",{path:entry.path,name:name.trim()},"正在重命名…")};
    const remove=document.createElement("button");remove.type="button";remove.className="danger";remove.textContent="移入回收站";remove.disabled=state.mode!=="idle";
    remove.onclick=async()=>{if(confirm(`确认把地图“${entry.name}”移入RK回收站？`))await manageMap("/api/maps/delete",{path:entry.path},"正在移入回收站…")};
    actions.append(use,rename,remove);record.append(title,meta,actions);list.append(record);
  }
}
async function viewSavedMap(entry){
  showActionFeedback("正在加载地图："+entry.name+"…","wait");
  try{
    const response=await fetch("/api/maps/preview?path="+encodeURIComponent(entry.path),{headers:{"Authorization":"Bearer "+token}});
    const body=response.headers.get("content-type")?.includes("json")?await response.json():null;
    if(response.status===401){requireLogin("登录已过期");throw Error("登录已过期，请重新登录")}
    if(!response.ok)throw Error(body?.error||response.statusText);
    const url=URL.createObjectURL(await response.blob()),image=new Image();
    await new Promise((resolve,reject)=>{image.onload=resolve;image.onerror=()=>reject(Error("地图图像解码失败"));image.src=url});
    URL.revokeObjectURL(url);mapImage=image;savedMapPreview={path:entry.path,name:entry.name,width:image.naturalWidth,height:image.naturalHeight};
    resetMapView();showTab("map");$("#mapInfo").textContent=`已保存地图：${entry.name}  ${image.naturalWidth}×${image.naturalHeight}`;
    showActionFeedback("正在查看已保存地图："+entry.name+"；未改变导航选择","ok");
  }catch(error){showActionFeedback("地图预览失败："+error.message,"bad");alert(error.message)}
}
async function refreshMapCatalog(){
  state=await api("/api/state",null,"GET");mapManagerSignature="";renderState();
}
async function manageMap(path,data,pending){
  showActionFeedback(pending,"wait");
  try{const result=await api(path,data);showActionFeedback(result.message||"操作完成","ok");await refreshMapCatalog()}
  catch(error){showActionFeedback("地图操作失败："+error.message,"bad");alert(error.message)}
}
$("#refreshMaps").onclick=()=>refreshMapCatalog().catch(error=>{showActionFeedback("刷新失败："+error.message,"bad");alert(error.message)});
async function loadMap(){
  savedMapPreview=null;loadingMap=true;const r=await fetch("/api/map.png?t="+Date.now(),{headers:{"Authorization":"Bearer "+token}});
  if(r.status===401){loadingMap=false;requireLogin("登录已过期");return}
  if(!r.ok){loadingMap=false;return}
  const u=URL.createObjectURL(await r.blob());mapImage=new Image();
  mapImage.onload=()=>{URL.revokeObjectURL(u);lastMapLoad=Date.now();loadingMap=false;draw()};
  mapImage.onerror=()=>{URL.revokeObjectURL(u);loadingMap=false};mapImage.src=u;
}
$("#refreshMap").onclick=()=>{savedMapPreview=null;mapImage=null;lastMapLoad=0;loadMap()};
function viewMetrics(c){
  const m=savedMapPreview?{width:savedMapPreview.width,height:savedMapPreview.height,resolution:1,origin_x:0,origin_y:0}:state?.map;if(!m)return null;
  const fit=Math.min(c.width/m.width,c.height/m.height),scale=fit*mapView.zoom;
  return{m,scale,cx:c.width/2+mapView.panX*devicePixelRatio,cy:c.height/2+mapView.panY*devicePixelRatio,cos:Math.cos(mapView.rotation),sin:Math.sin(mapView.rotation)};
}
function mapToCanvas(p,c){
  const v=viewMetrics(c);if(!v)return{x:0,y:0};
  const a=v.m.origin_yaw||0,dx=p.x-v.m.origin_x,dy=p.y-v.m.origin_y;
  const ix=(dx*Math.cos(a)+dy*Math.sin(a))/v.m.resolution,iy=v.m.height-(-dx*Math.sin(a)+dy*Math.cos(a))/v.m.resolution;
  const x=(ix-v.m.width/2)*v.scale,y=(iy-v.m.height/2)*v.scale;
  return{x:v.cx+x*v.cos-y*v.sin,y:v.cy+x*v.sin+y*v.cos};
}
function drawPose(ctx,c,pose,color,label){
  if(savedMapPreview||!pose||!state?.map)return;const p=mapToCanvas(pose,c),scale=devicePixelRatio;
  ctx.save();ctx.translate(p.x,p.y);ctx.rotate(mapView.rotation+(state?.map?.origin_yaw||0)-pose.yaw);ctx.fillStyle=color;ctx.strokeStyle="#fff";ctx.lineWidth=1.5*scale;
  ctx.beginPath();ctx.moveTo(14*scale,0);ctx.lineTo(-9*scale,8*scale);ctx.lineTo(-5*scale,0);ctx.lineTo(-9*scale,-8*scale);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
  if(label){ctx.save();ctx.fillStyle=color;ctx.font=`${12*scale}px sans-serif`;ctx.fillText(label,p.x+10*scale,p.y-10*scale);ctx.restore();}
}
function draw(){
  const c=$("#canvas"),ctx=c.getContext("2d"),w=Math.max(1,Math.round(c.clientWidth*devicePixelRatio)),h=Math.max(1,Math.round(c.clientHeight*devicePixelRatio));
  if(c.width!==w)c.width=w;if(c.height!==h)c.height=h;ctx.clearRect(0,0,c.width,c.height);
  const v=viewMetrics(c);
  if(mapImage&&v){ctx.save();ctx.translate(v.cx,v.cy);ctx.rotate(mapView.rotation);ctx.scale(v.scale,v.scale);ctx.drawImage(mapImage,-v.m.width/2,-v.m.height/2,v.m.width,v.m.height);ctx.restore();}
  drawObstacleView(ctx,c);
  drawPose(ctx,c,state?.confirmed_initial_pose,"#1565c0","本次初始化");
  if(state?.initial_state==="waiting"&&state?.initial_source==="manual")drawPose(ctx,c,state.initial_request,"#ef8c00","初始化待确认");
  drawPose(ctx,c,state?.goal,"#2e7d32","目标");drawPose(ctx,c,state?.pose,"#e53935","小车");drawPose(ctx,c,previewPose,"#ef8c00",pick==="initial"?"待设初始":"待设目标");
  drawPose(ctx,c,state?.saved_map_home||state?.home?.pose,"#673ab7","基站（建图起点）");
  const degrees=Math.round(mapView.rotation*180/Math.PI);$("#viewText").textContent=`${Math.round(mapView.zoom*100)}% / ${degrees}°`;
}
function mapPoint(ev){
  const c=$("#canvas"),r=c.getBoundingClientRect(),v=viewMetrics(c);
  const sx=(ev.clientX-r.left)*c.width/r.width-v.cx,sy=(ev.clientY-r.top)*c.height/r.height-v.cy;
  const x=(sx*v.cos+sy*v.sin)/v.scale,y=(-sx*v.sin+sy*v.cos)/v.scale;
  const ix=x+v.m.width/2,iy=y+v.m.height/2;
  const a=v.m.origin_yaw||0,xm=ix*v.m.resolution,ym=(v.m.height-iy)*v.m.resolution;
  return{x:v.m.origin_x+xm*Math.cos(a)-ym*Math.sin(a),y:v.m.origin_y+xm*Math.sin(a)+ym*Math.cos(a)};
}
function changeZoom(factor){mapView.zoom=Math.max(0.35,Math.min(6,mapView.zoom*factor));draw();}
function rotateMap(degrees){mapView.rotation+=degrees*Math.PI/180;draw();}
function resetMapView(){mapView.zoom=1;mapView.rotation=0;mapView.panX=0;mapView.panY=0;draw();}
$("#zoomIn").onclick=()=>changeZoom(1.25);$("#zoomOut").onclick=()=>changeZoom(0.8);
$("#rotateLeft").onclick=()=>rotateMap(-15);$("#rotateRight").onclick=()=>rotateMap(15);$("#resetView").onclick=resetMapView;
function beginPick(kind){
  obstacleMode=null;obstacleDraft=null;
  if(state?.mode!=="navigation"){alert("请先加载地图进入自动导航模式");return}
  if(kind==="goal"&&!state.initialized){alert("请先设置初始位姿，并等待顶部定位状态变为“已确认”");return}
  if(savedMapPreview){savedMapPreview=null;mapImage=null;lastMapLoad=0;loadMap()}
  pick=kind;dragStart=null;previewPose=null;$(".canvasWrap").classList.add("picking");
  const text=kind==="initial"?"按住地图上的小车实际位置，沿当前车头方向拖动，松开发送初始位姿":"按住目标位置，沿到达后的期望车头方向拖动，松开发送目标";
  $("#mapPickHint").textContent=text;$("#pickHint").textContent=text;showTab("map");
}
const canvas=$("#canvas");
canvas.onpointerdown=e=>{if(!viewMetrics(canvas))return;e.preventDefault();canvas.setPointerCapture(e.pointerId);if(pick){dragStart=mapPoint(e);previewPose={...dragStart,yaw:Number($("#yaw").value)*Math.PI/180};draw();return}panDrag={id:e.pointerId,x:e.clientX,y:e.clientY,panX:mapView.panX,panY:mapView.panY};$(".canvasWrap").classList.add("panning")};
canvas.onpointermove=e=>{if(panDrag&&panDrag.id===e.pointerId){e.preventDefault();mapView.panX=panDrag.panX+e.clientX-panDrag.x;mapView.panY=panDrag.panY+e.clientY-panDrag.y;draw();return}if(!dragStart||!pick)return;e.preventDefault();const p=mapPoint(e),dx=p.x-dragStart.x,dy=p.y-dragStart.y;if(Math.hypot(dx,dy)>=0.03)previewPose={...dragStart,yaw:Math.atan2(dy,dx)};draw()};
canvas.onpointerup=async e=>{
  if(panDrag&&panDrag.id===e.pointerId){panDrag=null;$(".canvasWrap").classList.remove("panning");return}
  if(!dragStart||!pick)return;e.preventDefault();const kind=pick,p=mapPoint(e),dx=p.x-dragStart.x,dy=p.y-dragStart.y;
  const yaw=Math.hypot(dx,dy)>=0.08?Math.atan2(dy,dx):Number($("#yaw").value)*Math.PI/180;
  const pose={...dragStart,yaw};previewPose=pose;$("#yaw").value=(yaw*180/Math.PI).toFixed(1);draw();
  try{await api(kind==="initial"?"/api/initial_pose":"/api/goal",pose);$("#mapPickHint").textContent=kind==="initial"?"初始位姿已发送，等待AMCL确认。确认后蓝色箭头为本次初始化返回点；紫色基站不变。":"目标已发送，等待顶部“导航”显示Nav2已接受。绿色箭头表示目标位置和朝向。";pick=null;dragStart=null;previewPose=null;$(".canvasWrap").classList.remove("picking");}
  catch(err){$("#mapPickHint").textContent="发送失败："+err.message;alert(err.message);dragStart=null;previewPose=null;}
};
canvas.onpointercancel=()=>{dragStart=null;previewPose=null;panDrag=null;$(".canvasWrap").classList.remove("panning");draw()};
canvas.addEventListener("wheel",e=>{e.preventDefault();changeZoom(e.deltaY<0?1.15:0.87)},{passive:false});
$("#sideSonarToggle").onclick=async()=>{
  const enabled=state.side_ultrasonic_enabled===false;
  if(!confirm(enabled?"下次任务开启左右超声波保护？":"确认下次任务关闭左右12cm近障保护和新增障碍记录？已有障碍记录不会因此清除。"))return;
  try{await api('/api/side_ultrasonic',{enabled,confirm_disabled:!enabled});state=await api('/api/state',null,'GET');renderState()}catch(e){alert(e.message)}
};
$("#setInitial").onclick=()=>beginPick("initial");
$("#setGoal").onclick=()=>beginPick("goal");
function showActionFeedback(text,kind=""){const el=$("#actionFeedback");el.textContent=text;el.className=kind||"hint";}
async function runAction(button,path,pending,successText){
  const label=button.textContent;button.disabled=true;button.textContent=pending;showActionFeedback(pending,"wait");
  try{const result=await api(path,{});showActionFeedback(result.message||successText,"ok");return result}
  catch(e){showActionFeedback("操作失败："+e.message,"bad");alert(e.message);throw e}
  finally{button.disabled=false;button.textContent=label}
}
async function requestMode(mode,extra={}){
  if(modeRequest)return;modeRequest=true;showActionFeedback("正在启动，请稍候…","wait");
  try{await api("/api/mode",{mode,...extra});showActionFeedback("启动请求已由 RK 本地监督器接管","ok")}
  catch(e){showActionFeedback("启动失败："+e.message,"bad");alert(e.message)}
  finally{setTimeout(()=>{modeRequest=false;if(state)renderState()},800)}
}
$("#startMapping").onclick=()=>requestMode("mapping",{mapping_scan_source:$("#mappingScanSource").value});
$("#saveManual").onclick=()=>runAction($("#saveManual"),"/api/save_manual","正在保存…","手动地图保存请求已提交").catch(()=>{});
$("#startAuto").onclick=()=>confirm("现场无人靠近且急停可用？")&&requestMode("auto_mapping",{mapping_scan_source:$("#mappingScanSource").value});
$("#pauseAuto").onclick=()=>runAction($("#pauseAuto"),"/api/exploration/stop","正在暂停…","自动探索已暂停").catch(()=>{});
$("#pauseMapping").onclick=()=>runAction($("#pauseMapping"),"/api/exploration/stop","正在暂停…","已暂停，保留当前定位").catch(()=>{});
$("#returnHome").onclick=()=>{if(confirm("确认回地图记录的基站（建图起点）？这不是本次手动初始化位置。小车先导航到基站正前方50cm，对齐后倒车入站；建图模式会先保存地图。请确认基站通道畅通。"))runAction($("#returnHome"),"/api/return_home","正在请求回基站…","回基站请求已接受").catch(()=>{})};
$("#returnInitial").onclick=()=>{if(confirm("确认普通导航回本次手动初始化的位置和朝向？不执行基站对齐倒车；若该点在半包围基站内，请取消并使用“回基站”。"))runAction($("#returnInitial"),"/api/return_initial_pose","请求回初始化位置…","初始化位置返回请求已接受").catch(()=>{})};
$("#cancelHome").onclick=()=>runAction($("#cancelHome"),"/api/exploration/stop","正在取消…","返航已取消").catch(()=>{});
$("#finishManualHome").onclick=async()=>{
  if(!confirm("确认暂停、请求保存当前手动地图并返航？请确保现场安全。"))return;
  try{
    await runAction($("#finishManualHome"),"/api/exploration/stop","正在暂停…","已暂停");
    await runAction($("#finishManualHome"),"/api/return_home","请求返航…","返航请求已接受");
  }catch(e){}
};
$("#resumeAuto").onclick=()=>runAction($("#resumeAuto"),"/api/exploration/start","正在继续…","自动探索已继续").catch(()=>{});
$("#saveAuto").onclick=()=>runAction($("#saveAuto"),"/api/exploration/save","正在保存…","地图保存请求已接受").catch(()=>{});
$("#startNav").onclick=()=>{if(!$("#maps").value){alert("没有可用地图，请先保存地图");return}if(confirm($("#departDock").checked?"确认小车位于基站内、车头朝外？首次目标会先直行50cm，再规划导航。":"确认小车已在基站外？将直接规划导航，可能原地转向。"))requestMode("navigation",{map:$("#maps").value,depart_from_dock:$("#departDock").checked})};
$("#cancel").onclick=()=>runAction($("#cancel"),"/api/cancel","正在取消…","导航取消请求已提交").catch(()=>{});
$("#emergencyStop").onclick=()=>{if(confirm("确认执行网页急停？这会立即结束当前建图/导航任务并停车。"))runAction($("#emergencyStop"),"/api/emergency_stop","急停执行中…","网页急停已执行，当前任务已结束").catch(()=>{})};
async function tele(v,a){try{await api("/api/teleop",{linear:v,angular:a,active:a!==0||v!==0})}catch(e){}}
document.querySelectorAll("[data-v]").forEach(b=>{let timer;const start=e=>{e.preventDefault();const [v,a]=b.dataset.v.split(",").map(Number);tele(v,a);timer=setInterval(()=>tele(v,a),100)};const stop=e=>{e.preventDefault();clearInterval(timer);tele(0,0)};b.onpointerdown=start;b.onpointerup=stop;b.onpointercancel=stop;b.onpointerleave=stop});
document.addEventListener("visibilitychange",()=>{if(document.hidden)tele(0,0)});window.addEventListener("pagehide",()=>tele(0,0));window.addEventListener("resize",draw);

// Supplemental overlay edits always carry the displayed session and revision.
let obstacleMode=null, obstacleDraft=null, obstacleBusy=false, obstaclePointer=null;
function cancelObstaclePick(){obstacleMode=null;obstacleDraft=null;obstaclePointer=null;draw();}
function drawObstacleView(ctx,c){
  const o=state?.obstacle_view||{},v=viewMetrics(c);
  const editable=!!(o.fresh&&o.editable&&!savedMapPreview&&!obstacleBusy);
  $("#addObstacle").disabled=!editable;$("#deleteObstacle").disabled=!editable;
  $("#applyObstacle").disabled=!editable||!obstacleDraft||obstaclePointer!==null;
  $("#pauseObstacle").disabled=!state||!["mapping","auto_mapping","navigation"].includes(state.mode)||state.transitioning||obstacleBusy;
  $("#saveObstacleMap").disabled=!editable||!["mapping","auto_mapping"].includes(state?.mode)||!!obstacleDraft||!!obstacleMode;
  $("#resumeObstacle").disabled=!editable||state?.mode!=="auto_mapping"||!!obstacleDraft||!!obstacleMode;
  $("#obstacleStatus").textContent=!o.fresh?"障碍状态未连接，不能编辑":
    (o.message?o.message+"；":"")+(o.pending||obstacleBusy?"正在等待障碍层确认…":editable?"已停稳，可拖动画线或点选编辑；确认修改后再继续任务":"请先暂停任务并等待停稳，才可编辑")+
    `；避障记录 ${(o.obstacles||[]).length}，观察点 ${(o.observations||[]).length}（观察模式）`;
  if(savedMapPreview||!v||!o.fresh)return;
  if(obstacleDraft&&(obstacleDraft.session!==o.session||obstacleDraft.revision!==o.revision)){obstacleDraft=null;obstaclePointer=null;$("#applyObstacle").disabled=true;}
  const circle=(z,color,filled,dashed)=>{const p=mapToCanvas(z,c),r=Math.max(3*devicePixelRatio,z.radius/v.m.resolution*v.scale);ctx.save();ctx.strokeStyle=color;ctx.fillStyle=filled?color+"55":color+"15";ctx.lineWidth=2*devicePixelRatio;ctx.setLineDash(dashed?[5*devicePixelRatio,4*devicePixelRatio]:[]);ctx.beginPath();ctx.arc(p.x,p.y,r,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.restore();return p;};
  for(const z of o.unreachable||[]){const p=circle(z,"#d32f2f",false,true);ctx.save();ctx.strokeStyle="#d32f2f";ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();ctx.moveTo(p.x-5,p.y-5);ctx.lineTo(p.x+5,p.y+5);ctx.moveTo(p.x-5,p.y+5);ctx.lineTo(p.x+5,p.y-5);ctx.stroke();ctx.restore();}
  for(const z of o.observations||[])circle(z,"#d89900",false,false);
  for(const z of o.obstacles||[])circle(z,"#c62828",true,false);
  if(o.route?.length){ctx.save();ctx.strokeStyle="#16a34a";ctx.lineWidth=3*devicePixelRatio;ctx.beginPath();o.route.forEach((z,i)=>{const p=mapToCanvas(z,c);i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y)});ctx.stroke();ctx.restore();}
  if(o.goal){const p=mapToCanvas(o.goal,c);ctx.save();ctx.strokeStyle="#15803d";ctx.fillStyle="#22c55e";ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();ctx.moveTo(p.x,p.y);ctx.lineTo(p.x,p.y-22*devicePixelRatio);ctx.lineTo(p.x+15*devicePixelRatio,p.y-16*devicePixelRatio);ctx.lineTo(p.x,p.y-10*devicePixelRatio);ctx.stroke();ctx.fill();ctx.font=`${12*devicePixelRatio}px sans-serif`;ctx.fillText("探索点",p.x+5*devicePixelRatio,p.y+14*devicePixelRatio);ctx.restore();}
  if(obstacleDraft?.points){
    ctx.save();ctx.strokeStyle=obstacleDraft.op==="delete_line"?"#e65100":"#1565c0";
    ctx.globalAlpha=.65;ctx.lineCap="round";ctx.lineWidth=Math.max(3*devicePixelRatio,2*obstacleDraft.radius/v.m.resolution*v.scale);
    ctx.beginPath();obstacleDraft.points.forEach((z,i)=>{const p=mapToCanvas(z,c);i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y)});ctx.stroke();ctx.restore();
  }else if(obstacleDraft)circle(obstacleDraft,"#1565c0",false,true);
}
function beginObstaclePick(op){
  if(!state?.obstacle_view?.editable||savedMapPreview)return;
  pick=null;dragStart=null;previewPose=null;panDrag=null;obstacleMode=op;obstacleDraft=null;obstaclePointer=null;
  $("#mapPickHint").textContent=$("#obstacleShape").value==="line"?"按住起点，拖动到终点后松开；蓝色为增加预览，橙色为擦除范围，再点击确认修改。":op==="add"?"点选障碍的实际位置（蓝圈预览），再点“确认修改”；默认半径4cm。":"点选红色或黄色障碍记录，再点“确认修改”。红色虚线不可达圈不属于障碍。";draw();
}
$("#addObstacle").onclick=()=>beginObstaclePick("add");
$("#deleteObstacle").onclick=()=>beginObstaclePick("delete");
$("#cancelObstacle").onclick=cancelObstaclePick;
canvas.addEventListener("pointerdown",e=>{
  if(!obstacleMode)return;e.preventDefault();e.stopImmediatePropagation();
  const o=state?.obstacle_view;if(!o?.editable||savedMapPreview||!viewMetrics(canvas))return;
  const p=mapPoint(e);let z;
  if($("#obstacleShape").value==="line"){
    const radius=Number($("#obstacleRadius").value)/100;
    if(!Number.isFinite(radius)||radius<.02||radius>.15){alert("线条半宽应为2～15cm");return;}
    obstaclePointer=e.pointerId;canvas.setPointerCapture(e.pointerId);
    obstacleDraft={op:obstacleMode+"_line",points:[p,p],radius,session:o.session,revision:o.revision};draw();return;
  }
  if(obstacleMode==="add"){
    const radius=Number($("#obstacleRadius").value)/100;
    if(!Number.isFinite(radius)||radius<.02||radius>.15){alert("半径应为2～15cm");return}z={...p,radius};
  }else{
    const v=viewMetrics(canvas),hit=10*devicePixelRatio*v.m.resolution/v.scale;
    z=[...(o.obstacles||[]),...(o.observations||[])].filter(z=>Math.hypot(z.x-p.x,z.y-p.y)<=Math.max(z.radius,hit)).sort((a,b)=>Math.hypot(a.x-p.x,a.y-p.y)-Math.hypot(b.x-p.x,b.y-p.y))[0];
    if(!z){$("#mapPickHint").textContent="没有选中障碍记录，请点选红色实心或黄色空心点。";return}
  }
  obstacleDraft={...z,session:o.session,revision:o.revision,op:obstacleMode,obstacle_id:z.id};
  $("#mapPickHint").textContent=`待${obstacleMode==="add"?"增加":"删除"}：x=${z.x.toFixed(2)}m，y=${z.y.toFixed(2)}m，半径${Math.round(z.radius*100)}cm。点击“确认修改”生效。`;draw();
},true);
$("#applyObstacle").onclick=async()=>{
  if(!obstacleDraft||obstacleBusy)return;
  const draft={...obstacleDraft};obstacleBusy=true;draw();
  try{const result=await api("/api/obstacles/edit",draft);$("#mapPickHint").textContent=result.message;obstacleMode=null;obstacleDraft=null;state=await api("/api/state",null,"GET");}
  catch(e){$("#mapPickHint").textContent=e.message;alert(e.message);}
  finally{obstacleBusy=false;draw();}
};

canvas.addEventListener("pointermove",e=>{
  if(!obstacleMode)return;e.preventDefault();e.stopImmediatePropagation();
  if(obstaclePointer!==e.pointerId||!obstacleDraft?.points)return;
  if(!state?.obstacle_view?.editable){cancelObstaclePick();return;}
  obstacleDraft.points[1]=mapPoint(e);draw();
},true);
canvas.addEventListener("pointerup",e=>{
  if(!obstacleMode)return;e.preventDefault();e.stopImmediatePropagation();
  if(obstaclePointer!==e.pointerId)return;
  if(obstacleDraft?.points){obstacleDraft.points[1]=mapPoint(e);const [a,b]=obstacleDraft.points;
    $("#mapPickHint").textContent=`线段长度 ${Math.hypot(b.x-a.x,b.y-a.y).toFixed(2)}m；点击“确认修改”生效，完成后再继续任务。`;}
  obstaclePointer=null;if(canvas.hasPointerCapture(e.pointerId))canvas.releasePointerCapture(e.pointerId);draw();
},true);
canvas.addEventListener("pointercancel",e=>{if(obstacleMode){e.stopImmediatePropagation();cancelObstaclePick();}},true);
$("#obstacleShape").onchange=cancelObstaclePick;
$("#pauseObstacle").onclick=async()=>{
  cancelObstaclePick();
  try{await runAction($("#pauseObstacle"),"/api/exploration/stop","正在暂停…","已暂停，等待停稳后可画线编辑");state=await api("/api/state",null,"GET");renderState();}catch(e){}
};
$("#resumeObstacle").onclick=()=>runAction($("#resumeObstacle"),"/api/exploration/start","正在继续…","继续当前建图任务").catch(()=>{});

$("#saveObstacleMap").onclick=()=>runAction($("#saveObstacleMap"),"/api/exploration/save","正在保存地图和障碍…","地图和补充障碍保存请求已接受").catch(()=>{});
