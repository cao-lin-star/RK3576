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
async function loop(){if(!token)return;try{await api("/api/heartbeat",{});if(!token)return;state=await api("/api/state",null,"GET");$("#link").textContent="已连接";renderState()}catch(e){if(token)$("#link").textContent="连接中断"}if(token)setTimeout(loop,1000)}
function setStatus(id,text,kind){const el=$("#"+id);el.textContent=text;el.className=kind||"";}
function renderState(){
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
  const returning=["preparing","sending","returning","waiting_health","saving_map","handoff_ready","undocking","docking","dock_waiting","dock_preparing","aligning"].includes(home.phase);
  let homeText=state.home_status_fresh?(home.message||"等待起点"):"返航状态未就绪 / 已失联";
  if(home.pose)homeText+=`｜起点 (${home.pose.x.toFixed(2)}, ${home.pose.y.toFixed(2)})`;
  if(Number.isFinite(home.distance_remaining_m)&&returning)homeText+=`｜剩余 ${home.distance_remaining_m.toFixed(2)} m`;
  if(state.nav_message && (!state.home_status_fresh || home.phase==="ready"))homeText+=`｜${state.nav_message}`;
  $("#homeStatus").textContent=homeText;
  $("#returnHome").disabled=!(mappingActive||state.mode==="navigation")||!state.home_status_fresh||!home.available||returning;
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
  const ix=(p.x-v.m.origin_x)/v.m.resolution,iy=v.m.height-(p.y-v.m.origin_y)/v.m.resolution;
  const x=(ix-v.m.width/2)*v.scale,y=(iy-v.m.height/2)*v.scale;
  return{x:v.cx+x*v.cos-y*v.sin,y:v.cy+x*v.sin+y*v.cos};
}
function drawPose(ctx,c,pose,color,label){
  if(savedMapPreview||!pose||!state?.map)return;const p=mapToCanvas(pose,c),scale=devicePixelRatio;
  ctx.save();ctx.translate(p.x,p.y);ctx.rotate(mapView.rotation-pose.yaw);ctx.fillStyle=color;ctx.strokeStyle="#fff";ctx.lineWidth=1.5*scale;
  ctx.beginPath();ctx.moveTo(14*scale,0);ctx.lineTo(-9*scale,8*scale);ctx.lineTo(-5*scale,0);ctx.lineTo(-9*scale,-8*scale);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
  if(label){ctx.save();ctx.fillStyle=color;ctx.font=`${12*scale}px sans-serif`;ctx.fillText(label,p.x+10*scale,p.y-10*scale);ctx.restore();}
}
function draw(){
  const c=$("#canvas"),ctx=c.getContext("2d"),w=Math.max(1,Math.round(c.clientWidth*devicePixelRatio)),h=Math.max(1,Math.round(c.clientHeight*devicePixelRatio));
  if(c.width!==w)c.width=w;if(c.height!==h)c.height=h;ctx.clearRect(0,0,c.width,c.height);
  const v=viewMetrics(c);
  if(mapImage&&v){ctx.save();ctx.translate(v.cx,v.cy);ctx.rotate(mapView.rotation);ctx.scale(v.scale,v.scale);ctx.drawImage(mapImage,-v.m.width/2,-v.m.height/2,v.m.width,v.m.height);ctx.restore();}
  drawPose(ctx,c,state?.initial_request,"#1565c0","初始");drawPose(ctx,c,state?.goal,"#2e7d32","目标");drawPose(ctx,c,state?.pose,"#e53935","小车");drawPose(ctx,c,previewPose,"#ef8c00",pick==="initial"?"待设初始":"待设目标");
  drawPose(ctx,c,state?.saved_map_home||state?.home?.pose,"#673ab7",state?.saved_map_home?"建图起点":"起点");
  const degrees=Math.round(mapView.rotation*180/Math.PI);$("#viewText").textContent=`${Math.round(mapView.zoom*100)}% / ${degrees}°`;
}
function mapPoint(ev){
  const c=$("#canvas"),r=c.getBoundingClientRect(),v=viewMetrics(c);
  const sx=(ev.clientX-r.left)*c.width/r.width-v.cx,sy=(ev.clientY-r.top)*c.height/r.height-v.cy;
  const x=(sx*v.cos+sy*v.sin)/v.scale,y=(-sx*v.sin+sy*v.cos)/v.scale;
  const ix=x+v.m.width/2,iy=y+v.m.height/2;
  return{x:v.m.origin_x+ix*v.m.resolution,y:v.m.origin_y+(v.m.height-iy)*v.m.resolution};
}
function changeZoom(factor){mapView.zoom=Math.max(0.35,Math.min(6,mapView.zoom*factor));draw();}
function rotateMap(degrees){mapView.rotation+=degrees*Math.PI/180;draw();}
function resetMapView(){mapView.zoom=1;mapView.rotation=0;mapView.panX=0;mapView.panY=0;draw();}
$("#zoomIn").onclick=()=>changeZoom(1.25);$("#zoomOut").onclick=()=>changeZoom(0.8);
$("#rotateLeft").onclick=()=>rotateMap(-15);$("#rotateRight").onclick=()=>rotateMap(15);$("#resetView").onclick=resetMapView;
function beginPick(kind){
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
  try{await api(kind==="initial"?"/api/initial_pose":"/api/goal",pose);$("#mapPickHint").textContent=kind==="initial"?"初始位姿已发送，等待顶部“定位”变为已确认。蓝色箭头表示你设置的位置和朝向。":"目标已发送，等待顶部“导航”显示Nav2已接受。绿色箭头表示目标位置和朝向。";pick=null;dragStart=null;previewPose=null;$(".canvasWrap").classList.remove("picking");}
  catch(err){$("#mapPickHint").textContent="发送失败："+err.message;alert(err.message);dragStart=null;previewPose=null;}
};
canvas.onpointercancel=()=>{dragStart=null;previewPose=null;panDrag=null;$(".canvasWrap").classList.remove("panning");draw()};
canvas.addEventListener("wheel",e=>{e.preventDefault();changeZoom(e.deltaY<0?1.15:0.87)},{passive:false});
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
$("#returnHome").onclick=()=>{if(confirm("确认保存地图并返航？小车会先导航到起点正前方50cm，对齐后倒车回起点。请确认基站通道畅通。"))runAction($("#returnHome"),"/api/return_home","正在请求返航…","返航请求已接受").catch(()=>{})};
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
