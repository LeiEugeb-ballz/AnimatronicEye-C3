#include <Arduino.h>
#include <DNSServer.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

// ESP32-C3 SuperMini pin plan. GPIO2/8/9 are deliberately avoided because
// they are strapping pins sampled during reset.
constexpr uint8_t SERVO_COUNT = 4;
constexpr uint8_t AXIS_COUNT = 4;
// Logical order: top lid, bottom lid, eye horizontal, eye vertical.
constexpr uint8_t SERVO_PINS[SERVO_COUNT] = {7, 5, 6, 10};
constexpr uint8_t JOYSTICK_AXIS_PINS[AXIS_COUNT] = {0, 1, 3, 4};
constexpr uint8_t JOYSTICK_SWITCH_1_PIN = 20;
constexpr uint8_t JOYSTICK_SWITCH_2_PIN = 21;

constexpr char AP_SSID[] = "AnimatronicEye";
constexpr char AP_PASSWORD[] = "eyecontrol32";
constexpr char FIRMWARE_VERSION[] = "2.1.2-ap";
constexpr char CONFIG_NAMESPACE[] = "eye-c3-v1";

constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr uint32_t SERVO_TICK_MS = 20;
const float SERVO_STEP_DEGREES[SERVO_COUNT] = {4.0f, 4.0f, 2.0f, 2.0f};
constexpr int JOYSTICK_DEADZONE = 140;
constexpr int16_t KEEP_VALUE = 127;

enum ServoIndex : uint8_t {
  TOP_LID = 0,
  BOTTOM_LID = 1,
  HORIZONTAL = 2,
  VERTICAL = 3
};

enum ControlMode : uint8_t {
  MODE_WEB = 0,
  MODE_JOYSTICK_CLASSIC = 1,
  MODE_JOYSTICK_DUAL = 2
};

enum MotionId : uint8_t {
  MOTION_NONE = 0,
  MOTION_BLINK,
  MOTION_SCAN,
  MOTION_DIAGONAL,
  MOTION_ROLL,
  MOTION_CENTER,
  MOTION_RANDOM
};

struct ServoCalibration {
  int a;
  int center;
  int b;
};

struct AxisCalibration {
  int minimum;
  int center;
  int maximum;
  bool inverted;
};

struct MotionFrame {
  int16_t x;
  int16_t y;
  int16_t topOpen;
  int16_t bottomOpen;
  uint16_t durationMs;
};

const ServoCalibration DEFAULT_SERVO_CAL[SERVO_COUNT] = {
  {90, 55, 20},   // GPIO7: top lid, closed / midpoint / open
  {90, 135, 180}, // GPIO5: bottom lid, closed / midpoint / open
  {60, 95, 130},  // GPIO6: eye left / center / right
  {60, 90, 120}   // GPIO10: eye up / center / down
};

const MotionFrame BLINK_FRAMES[] = {
  {KEEP_VALUE, KEEP_VALUE, 0, 0, 380},
  {KEEP_VALUE, KEEP_VALUE, KEEP_VALUE, KEEP_VALUE, 420}
};

const MotionFrame SCAN_FRAMES[] = {
  {-75, 0, 90, 90, 650}, {0, 0, 0, 0, 350},
  {75, 0, 90, 90, 650}, {0, 0, 0, 0, 350},
  {0, 0, KEEP_VALUE, KEEP_VALUE, 450}
};

const MotionFrame DIAGONAL_FRAMES[] = {
  {-70, -65, 95, 65, 560}, {70, -65, 95, 65, 560},
  {0, 0, 0, 0, 350},
  {-70, 65, 65, 95, 560}, {70, 65, 65, 95, 560},
  {0, 0, 0, 0, 350}, {0, 0, KEEP_VALUE, KEEP_VALUE, 430}
};

const MotionFrame ROLL_FRAMES[] = {
  {0, -70, 95, 65, 260}, {58, -50, 92, 70, 260},
  {75, 0, 88, 88, 260}, {58, 50, 70, 92, 260},
  {0, 70, 65, 95, 260}, {-58, 50, 70, 92, 260},
  {-75, 0, 88, 88, 260}, {-58, -50, 92, 70, 260},
  {0, -70, 95, 65, 260}, {0, 0, KEEP_VALUE, KEEP_VALUE, 450}
};

const MotionFrame CENTER_FRAMES[] = {
  {0, 0, KEEP_VALUE, KEEP_VALUE, 650}
};

Servo servo[SERVO_COUNT];
ServoCalibration servoCal[SERVO_COUNT];
AxisCalibration axisCal[AXIS_COUNT];
bool servoEnabled[SERVO_COUNT] = {false, false, false, false};
float servoCurrent[SERVO_COUNT] = {90, 90, 90, 90};
float servoTarget[SERVO_COUNT] = {90, 90, 90, 90};
int joystickRaw[AXIS_COUNT] = {0, 0, 0, 0};

ControlMode controlMode = MODE_WEB;
int webX = 0;
int webY = 0;
int webTopOpen = 100;
int webBottomOpen = 100;
uint32_t blinkUntilMs = 0;
uint32_t lastServoTickMs = 0;
uint32_t lastJoystickReadMs = 0;

MotionId activeMotion = MOTION_NONE;
bool randomShow = false;
uint8_t motionFrameIndex = 0;
uint32_t motionFrameStartedMs = 0;
int motionBaseX = 0;
int motionBaseY = 0;
int motionBaseTop = 100;
int motionBaseBottom = 100;

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;

void applyControlTargets();

// Indent JavaScript so PlatformIO's sketch preprocessor skips its function declarations.
const char APP_JS[] PROGMEM = R"JS(
  const $=id=>document.getElementById(id),names=['Top lid','Bottom lid','Eye horizontal','Eye vertical'],slots=[['Closed','Midpoint','Open'],['Closed','Midpoint','Open'],['Left','Centre','Right'],['Up','Centre','Down']];
  const controls={top:$('top'),bottom:$('bottom')};let state=null,pointerId=null,sendTimer=0,poseInFlight=false,pendingPose=null,refreshing=false,calUnlocked=false,calBuilt=false,lastStateAt=0,toastTimer=0,webX=0,webY=0;
  async function api(path){
   const controller=new AbortController();
   const timeout=setTimeout(()=>controller.abort(),3000);
   try{
    const response=await fetch(path,{cache:'no-store',signal:controller.signal});
    const data=await response.json();
    if(!response.ok)throw Error(data.error||`HTTP ${response.status}`);
    return data;
   }catch(error){
    if(controller.signal.aborted)throw Error('Device response timed out');
    throw error;
   }finally{clearTimeout(timeout)}
  }
  function showError(message){const e=$('error');e.textContent=`Control link error: ${message}. Recovery: http://192.168.4.1/basic`;e.classList.add('show')}
  function clearError(){$('error').classList.remove('show')}
  function toast(message){clearTimeout(toastTimer);$('toast').textContent=message;$('toast').classList.add('show');toastTimer=setTimeout(()=>$('toast').classList.remove('show'),1800)}
  function setConnected(ok){$('connDot').className='dot '+(ok?'live':'dead');$('connText').textContent=ok?'LIVE':'OFFLINE'}
  async function act(task,ok){try{await task();if(ok)toast(ok);await refresh()}catch(e){showError(e.message)}}
  function power(cmd,servo=-1){if(cmd==='stop'){discardPendingPose();pointerId=null;state=null}return act(()=>api(`/api/power?cmd=${cmd}&servo=${servo}`),cmd==='stop'?'Outputs detached':'Outputs enabled')}
  function setMode(mode){return act(()=>api(`/api/mode?value=${mode}`),`Mode: ${mode}`)}
  function motion(name){return act(()=>api(`/api/animation?name=${name}`),name==='stop'?'Show ended':`${name} started`)}
  function setPose(x,y,top,bottom){webX=x;webY=y;controls.top.value=top;controls.bottom.value=bottom;preview();sendControl(true)}
  function preview(){const x=Number(webX),y=Number(webY),top=Number(controls.top.value),bottom=Number(controls.bottom.value);$('xv').textContent=x;$('yv').textContent=y;$('tv').textContent=top+'%';$('bv').textContent=bottom+'%';$('padDot').style.left=(50+x*.42)+'%';$('padDot').style.top=(50+y*.42)+'%';$('iris').style.transform=`translate(calc(-50% + ${x*.38}px),calc(-50% + ${y*.25}px))`;$('topLid').style.height=(50-top*.35)+'%';$('bottomLid').style.height=(50-bottom*.35)+'%'}
  function discardPendingPose(){
   clearTimeout(sendTimer);
   sendTimer=0;
   pendingPose=null;
  }
  async function flushPose(){
   sendTimer=0;
   if(poseInFlight||!pendingPose)return;
   const pose=pendingPose;
   pendingPose=null;
   poseInFlight=true;
   try{
    await api('/api/control?'+pose);
    lastStateAt=Date.now();
   }catch(error){
    discardPendingPose();
    showError(error.message);
   }finally{
    poseInFlight=false;
    if(pendingPose&&!sendTimer)sendTimer=setTimeout(flushPose,60);
   }
  }
  function sendControl(immediate=false){
   if(!state?.allEnabled)return;
   pendingPose=new URLSearchParams({x:webX,y:webY,top:controls.top.value,bottom:controls.bottom.value});
   preview();
   if(immediate){
    clearTimeout(sendTimer);
    flushPose();
   }else if(!sendTimer&&!poseInFlight){
    sendTimer=setTimeout(flushPose,60);
   }
  }
  function buildCalibration(s){const servoRoot=$('servos');servoRoot.innerHTML=s.servos.map((v,i)=>`<article id="servoCard${i}" class="servo-card locked"><div class="row spread"><div><h3>${names[i]}</h3><span class="small muted">GPIO ${v.pin}</span></div><span id="servoAngle${i}" class="servo-angle">${Math.round(v.current)}°</span></div><div class="servo-tools"><input id="cal${i}" type="range" min="0" max="180" value="${Math.round(v.target)}" onchange="calMove(${i},this.value)"><div class="row"><button id="servoPower${i}" onclick="power('${v.enabled?'stop':'enable'}',${i})">${v.enabled?'Detach':'Enable one'}</button></div><div class="saved"><button onclick="calSave(${i},'a')">Set ${slots[i][0]}<br><span id="save${i}a">${v.a}°</span></button><button onclick="calSave(${i},'center')">Set ${slots[i][1]}<br><span id="save${i}center">${v.center}°</span></button><button onclick="calSave(${i},'b')">Set ${slots[i][2]}<br><span id="save${i}b">${v.b}°</span></button></div></div></article>`).join('');
   const joyRoot=$('joysticks');joyRoot.innerHTML=s.joystick.map((v,i)=>`<article class="joy-card"><div class="row spread"><h3>J${i<2?1:2}${i%2?'Y':'X'} · GPIO ${v.pin}</h3><span id="joyRaw${i}" class="value">${v.raw}</span></div><div class="joy-values"><button onclick="joyCapture(${i},'min')">MIN<br><span id="joy${i}min">${v.min}</span></button><button onclick="joyCapture(${i},'center')">CENTRE<br><span id="joy${i}center">${v.center}</span></button><button onclick="joyCapture(${i},'max')">MAX<br><span id="joy${i}max">${v.max}</span></button></div><label class="small"><input id="joyInv${i}" type="checkbox" onchange="joyInvert(${i},this.checked)" ${v.inverted?'checked':''}> invert axis</label></article>`).join('');calBuilt=true}
  function render(s){state=s;lastStateAt=Date.now();setConnected(true);clearError();const armed=s.allEnabled;$('signal').textContent=armed?'ALL OUTPUTS ARMED':s.anyEnabled?'PARTIAL OUTPUTS ARMED':'OUTPUTS DETACHED';$('signal').className='signal-readout '+(s.anyEnabled?'on':'');document.querySelectorAll('.needs-power').forEach(el=>el.disabled=!armed);document.querySelectorAll('.mode').forEach(el=>el.classList.toggle('active',el.dataset.mode===s.mode));document.querySelectorAll('.motion').forEach(el=>el.classList.toggle('active',el.dataset.motion===s.animation));$('motionState').textContent=(s.animation||'idle').toUpperCase();
   if(pointerId===null&&!pendingPose&&!poseInFlight&&document.activeElement?.type!=='range'){webX=s.control.x;webY=s.control.y;controls.top.value=s.control.top;controls.bottom.value=s.control.bottom;preview()}if(!calBuilt)buildCalibration(s);s.servos.forEach((v,i)=>{const a=$(`servoAngle${i}`);if(a)a.textContent=`${Math.round(v.current)}°`;const p=$(`servoPower${i}`);if(p){p.textContent=v.enabled?'Detach':'Enable one';p.onclick=()=>power(v.enabled?'stop':'enable',i)}if(document.activeElement!==$(`cal${i}`))$(`cal${i}`).value=Math.round(v.target);$(`save${i}a`).textContent=v.a+'°';$(`save${i}center`).textContent=v.center+'°';$(`save${i}b`).textContent=v.b+'°'});s.joystick.forEach((v,i)=>{$(`joyRaw${i}`).textContent=v.raw;$(`joy${i}min`).textContent=v.min;$(`joy${i}center`).textContent=v.center;$(`joy${i}max`).textContent=v.max;$(`joyInv${i}`).checked=v.inverted});
   $('apSsid').textContent=s.network.apSsid;$('apIp').textContent=s.network.apIp;$('staSsid').textContent='AP only';$('staIp').textContent='Router connection disabled';$('firmware').textContent=s.firmware;$('uptime').textContent=Math.floor(s.uptime/1000)+'s';$('heap').textContent=Math.round(s.heap/1024)+'KB';$('switches').textContent=(s.switch1?'UP':'DOWN')+' / '+(s.switch2?'UP':'DOWN')}
  async function refresh(){
   if(refreshing)return;
   refreshing=true;
   try{
    const snapshot=await api('/api/state');
    if(pointerId===null&&!pendingPose&&!poseInFlight)render(snapshot);
   }catch(error){
    state=null;
    setConnected(false);
    showError(error.message);
   }finally{refreshing=false}
  }
  function toggleCalibration(){calUnlocked=!calUnlocked;$('calLock').textContent=calUnlocked?'Lock calibration':'Unlock calibration';document.querySelectorAll('.servo-card').forEach(el=>el.classList.toggle('locked',!calUnlocked))}
  function calMove(i,value){act(()=>api(`/api/cal-move?servo=${i}&angle=${value}`))}
  function calSave(i,slot){const value=$(`cal${i}`).value;act(()=>api(`/api/cal-save?servo=${i}&slot=${slot}&angle=${value}`),'Calibration saved')}
  function joyCapture(i,slot){act(()=>api(`/api/joy-capture?axis=${i}&slot=${slot}`),'Joystick point captured')}
  function joyInvert(i,value){act(()=>api(`/api/joy-invert?axis=${i}&value=${value?1:0}`),'Axis updated')}
  function resetDefaults(){if(confirm('Detach outputs and restore servo and joystick defaults?'))act(()=>api('/api/defaults'),'Motion defaults restored')}
  document.querySelectorAll('.mode').forEach(b=>b.onclick=()=>setMode(b.dataset.mode));document.querySelectorAll('.motion').forEach(b=>b.onclick=()=>motion(b.dataset.motion));
  const pad=$('pad');
  function padMove(event,immediate=false){
   const rect=pad.getBoundingClientRect();
   const scale=position=>Math.round(Math.max(-100,Math.min(100,position*200-100)));
   webX=scale((event.clientX-rect.left)/rect.width);
   webY=scale((event.clientY-rect.top)/rect.height);
   sendControl(immediate);
  }
  pad.onpointerdown=event=>{
   if(pointerId!==null||event.button!==0||!state?.allEnabled)return;
   pointerId=event.pointerId;
   pad.setPointerCapture(pointerId);
   padMove(event,true);
  };
  pad.onpointermove=event=>{if(event.pointerId===pointerId)padMove(event)};
  pad.onpointerup=event=>{
   if(event.pointerId!==pointerId)return;
   pointerId=null;
   padMove(event,true);
  };
  function endPointer(event){
   if(event.pointerId!==pointerId)return;
   pointerId=null;
   discardPendingPose();
  }
  pad.onpointercancel=endPointer;
  pad.onlostpointercapture=endPointer;
  controls.top.oninput=()=>sendControl();
  controls.bottom.oninput=()=>sendControl();
  window.addEventListener('error',event=>showError(event.message));
  refresh();
  setInterval(()=>{
   if(pointerId===null&&!pendingPose&&!poseInFlight)refresh();
   if(Date.now()-lastStateAt>3600)setConnected(false);
  },850);
)JS";

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#080b0d"><title>OCULUS // Eye Control</title>
<style>
:root{color-scheme:dark;--ink:#080b0d;--deck:#10161a;--plate:#171f24;--edge:#2a363d;--paper:#edf0e8;--muted:#839198;--cyan:#4de1e8;--amber:#ffbf4b;--red:#ff4d45;--green:#61d98b;--r:18px}
*{box-sizing:border-box}html{background:var(--ink)}body{margin:0;min-height:100vh;background:radial-gradient(circle at 15% -10%,#17313a 0,transparent 34%),linear-gradient(145deg,#080b0d,#0d1317 65%,#090c0e);color:var(--paper);font:15px "Avenir Next","Trebuchet MS",sans-serif;letter-spacing:.01em}body:before{content:"";position:fixed;inset:0;pointer-events:none;opacity:.22;background-image:repeating-linear-gradient(0deg,transparent 0 3px,#fff 4px);background-size:100% 4px;mix-blend-mode:soft-light}.mono,.value,.eyebrow{font-family:ui-monospace,"Cascadia Mono",Consolas,monospace}button,input,select{font:inherit}button{min-height:44px;border:1px solid var(--edge);border-radius:11px;background:#1b252b;color:var(--paper);padding:10px 14px;font-weight:750;cursor:pointer;transition:transform .14s,background .14s,border-color .14s}button:active{transform:scale(.975)}button:hover{border-color:#52636c}button:disabled{opacity:.35;cursor:not-allowed}.danger{background:#4a1718;border-color:#8f302e;color:#ffdfdc}.danger.hot{background:var(--red);border-color:#ff8c85;color:#180302}.primary{background:#15383b;border-color:#26777b;color:#bafcff}.amber{background:#3f3015;border-color:#7e612c;color:#ffe0a1}.ghost{background:transparent}.topbar{position:sticky;top:0;z-index:20;display:grid;grid-template-columns:1fr auto auto;align-items:center;gap:10px;padding:10px max(12px,calc((100vw - 1180px)/2));background:#080b0de8;border-bottom:1px solid #263238;backdrop-filter:blur(16px)}.brand{display:flex;align-items:center;gap:10px;min-width:0}.mark{width:38px;height:25px;border:2px solid var(--cyan);border-radius:60% 10%;transform:rotate(-8deg);position:relative;box-shadow:0 0 18px #4de1e840}.mark:after{content:"";position:absolute;width:8px;height:8px;border:3px solid var(--cyan);border-radius:50%;left:11px;top:5px}.title{font:700 19px "Bodoni 72",Didot,Georgia,serif;letter-spacing:.09em}.eyebrow{color:var(--cyan);font-size:10px;letter-spacing:.18em}.status{display:flex;align-items:center;gap:7px;border:1px solid var(--edge);border-radius:999px;padding:7px 10px;font:700 11px ui-monospace,monospace;white-space:nowrap}.dot{width:8px;height:8px;border-radius:50%;background:var(--amber);box-shadow:0 0 10px currentColor}.dot.live{background:var(--green)}.dot.dead{background:var(--red)}.shell{max-width:1180px;margin:auto;padding:18px 14px 95px}.hero{display:grid;grid-template-columns:minmax(280px,1.05fr) minmax(280px,.95fr);gap:16px}.panel{position:relative;background:linear-gradient(155deg,#182126,#10171b);border:1px solid var(--edge);border-radius:var(--r);padding:17px;box-shadow:0 18px 60px #0005,inset 0 1px #ffffff0a}.panel:after{content:"";position:absolute;inset:7px;border:1px solid #ffffff08;border-radius:12px;pointer-events:none}h1,h2,h3,p{margin-top:0}h1{font:700 clamp(32px,6vw,70px)/.9 "Bodoni 72",Didot,Georgia,serif;letter-spacing:-.045em;margin-bottom:12px}h2{font:700 22px "Bodoni 72",Didot,Georgia,serif;letter-spacing:.02em}h3{font-size:13px;letter-spacing:.14em;text-transform:uppercase}.muted{color:var(--muted)}.small{font-size:12px}.row{display:flex;align-items:center;gap:9px;flex-wrap:wrap}.spread{justify-content:space-between}.hero-copy{display:flex;flex-direction:column;justify-content:space-between;min-height:240px}.hero-copy p{max-width:48ch;color:#aab5b8}.signal-readout{font:800 12px ui-monospace,monospace;letter-spacing:.12em;color:var(--red)}.signal-readout.on{color:var(--green)}.eye-stage{min-height:240px;display:grid;place-items:center;overflow:hidden;background:radial-gradient(circle,#1b3035 0,#10171b 54%,#090d0f 100%)}.eye{width:min(86%,430px);aspect-ratio:1.75;position:relative;overflow:hidden;border:2px solid #53636a;border-radius:65% 12%;transform:rotate(-3deg);background:radial-gradient(circle at 50% 50%,#e9e1ca 0 48%,#88816f 70%,#1a2022 72%);box-shadow:0 0 0 7px #0c1215,0 0 70px #4de1e820,inset 0 0 40px #332c25}.iris{position:absolute;left:50%;top:50%;width:32%;aspect-ratio:1;border-radius:50%;transform:translate(-50%,-50%);background:radial-gradient(circle,#02090b 0 20%,#174e55 22% 47%,#54d4d2 49% 53%,#12262a 55% 100%);box-shadow:0 0 28px #4de1e855;transition:transform .18s ease}.iris:after{content:"";position:absolute;left:26%;top:18%;width:16%;aspect-ratio:1;border-radius:50%;background:#fff;opacity:.82}.lid{position:absolute;z-index:3;left:-4%;width:108%;height:15%;background:linear-gradient(#1a2023,#0b0e10);transition:height .18s ease}.lid.top{top:-2%;border-bottom:2px solid #5f6b70;border-radius:0 0 50% 50%}.lid.bottom{bottom:-2%;border-top:2px solid #5f6b70;border-radius:50% 50% 0 0}.reticle{position:absolute;inset:12px;border:1px dashed #4de1e844;border-radius:12px;pointer-events:none}.reticle:before,.reticle:after{content:"";position:absolute;background:#4de1e833}.reticle:before{width:1px;height:100%;left:50%}.reticle:after{height:1px;width:100%;top:50%}.tabs{position:sticky;top:69px;z-index:10;display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:16px 0;padding:6px;background:#0d1316e8;border:1px solid var(--edge);border-radius:15px;backdrop-filter:blur(14px)}.tabbtn{background:transparent;border:0;color:var(--muted);font:800 11px ui-monospace,monospace;letter-spacing:.11em}.tabbtn.active{background:#1c292e;color:var(--cyan);box-shadow:inset 0 0 0 1px #3e565f}.page{display:none}.page.active{display:block;animation:rise .28s ease}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}.control-grid{display:grid;grid-template-columns:minmax(260px,.9fr) minmax(300px,1.1fr);gap:16px}.pad{position:relative;aspect-ratio:1;max-height:430px;border:1px solid #3d5058;border-radius:22px;touch-action:none;overflow:hidden;background:radial-gradient(circle,#203038 0 2px,transparent 3px),repeating-radial-gradient(circle,#203039 0 1px,transparent 2px 15%),linear-gradient(135deg,#0c1215,#172128)}.pad:before,.pad:after{content:"";position:absolute;background:#4de1e82c}.pad:before{width:1px;height:100%;left:50%}.pad:after{height:1px;width:100%;top:50%}.pad-dot{position:absolute;left:50%;top:50%;width:42px;height:42px;border:2px solid var(--cyan);border-radius:50%;transform:translate(-50%,-50%);box-shadow:0 0 25px #4de1e860;background:#4de1e814}.pad-label{position:absolute;padding:10px;font:700 10px ui-monospace,monospace;color:#6e848d;letter-spacing:.1em}.pad-label.n{top:0;left:50%;transform:translateX(-50%)}.pad-label.s{bottom:0;left:50%;transform:translateX(-50%)}.pad-label.w{left:0;top:50%;transform:translateY(-50%)}.pad-label.e{right:0;top:50%;transform:translateY(-50%)}.telemetry{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-top:10px}.metric{background:#0c1215;border:1px solid #26343a;border-radius:10px;padding:10px}.metric b{display:block;color:var(--cyan);font-size:20px}.field{margin:15px 0}.field-head{display:flex;justify-content:space-between;margin-bottom:8px}.value{color:var(--cyan);font-weight:800}input[type=range]{width:100%;accent-color:var(--cyan)}input[type=text],input[type=password],input[type=number],select{width:100%;min-height:44px;border:1px solid #3a4b52;border-radius:10px;background:#0b1114;color:var(--paper);padding:10px 12px}.modes{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}.mode.active{border-color:var(--cyan);color:var(--cyan);background:#153438}.motion-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:11px}.motion{min-height:112px;text-align:left;padding:15px;background:linear-gradient(145deg,#1c282e,#11191d)}.motion strong{display:block;font:700 21px "Bodoni 72",Didot,Georgia,serif;margin-bottom:8px}.motion span{color:var(--muted);font-size:12px}.motion.active{border-color:var(--cyan);box-shadow:0 0 25px #4de1e819}.callout{border-left:3px solid var(--amber);padding:12px 14px;background:#2d241455;border-radius:0 10px 10px 0;color:#efd5a0}.lock{display:flex;gap:12px;align-items:flex-start}.servo-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin-top:14px}.servo-card{border:1px solid #2b3a40;border-radius:14px;background:#0d1417;padding:14px}.servo-card.locked .servo-tools{opacity:.35;pointer-events:none}.servo-angle{font:800 27px ui-monospace,monospace;color:var(--cyan)}.saved{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:8px}.saved button{padding:8px 4px;font-size:11px}.joy-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.joy-card{border:1px solid #2b3a40;border-radius:12px;padding:12px;background:#0c1215}.joy-values{display:grid;grid-template-columns:repeat(3,1fr);gap:5px}.joy-values button{font:700 11px ui-monospace,monospace;padding:7px 3px}.network-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.net-card{padding:14px;border-radius:13px;border:1px solid #2b3a40;background:#0b1215}.net-card b{display:block;font-size:18px;margin:4px 0}.form-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.error{display:none;position:fixed;z-index:50;left:12px;right:12px;bottom:82px;max-width:760px;margin:auto;background:#4a1718;border:1px solid var(--red);border-radius:12px;padding:12px 15px;color:#ffe1de}.error.show{display:block}.toast{position:fixed;z-index:60;top:78px;left:50%;transform:translate(-50%,-20px);opacity:0;background:#153438;border:1px solid var(--cyan);padding:10px 16px;border-radius:999px;pointer-events:none;transition:.2s}.toast.show{opacity:1;transform:translate(-50%,0)}@keyframes rise{from{opacity:0;transform:translateY(6px)}}@media(max-width:760px){.topbar{grid-template-columns:1fr auto}.topbar .status{display:none}.hero,.grid,.control-grid,.servo-grid,.network-grid{grid-template-columns:1fr}.hero-copy{min-height:190px}.tabs{top:61px}.motion-grid{grid-template-columns:repeat(2,1fr)}.form-grid{grid-template-columns:1fr}.shell{padding-left:10px;padding-right:10px}.panel{padding:14px}.title{font-size:16px}.danger.hot{padding:9px 10px}.eye-stage{min-height:210px}}@media(prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important}}
</style><style>
.tab-radio{position:fixed;left:-10000px;width:1px;height:1px;opacity:0;pointer-events:none}.tabbtn{display:grid;place-items:center;min-height:44px;border-radius:11px;padding:10px 14px;cursor:pointer;user-select:none;transition:transform .14s,background .14s,color .14s,box-shadow .14s}.tabbtn:active{transform:scale(.975)}.page.active{display:none}
#tab-control:checked~.shell #page-control,#tab-perform:checked~.shell #page-perform,#tab-calibrate:checked~.shell #page-calibrate,#tab-setup:checked~.shell #page-setup{display:block;animation:rise .28s ease}
#tab-control:checked~.shell label[for=tab-control],#tab-perform:checked~.shell label[for=tab-perform],#tab-calibrate:checked~.shell label[for=tab-calibrate],#tab-setup:checked~.shell label[for=tab-setup]{background:#1c292e;color:var(--cyan);box-shadow:inset 0 0 0 1px #3e565f}
</style></head><body>
<header class="topbar"><div class="brand"><div class="mark"></div><div><div class="eyebrow">ESP32-C3 // OPTICAL RIG</div><div class="title">OCULUS CONTROL</div></div></div><div class="status"><span id="connDot" class="dot"></span><span id="connText">CONNECTING</span></div><button class="danger hot" onclick="power('stop')">STOP / DETACH</button></header>
<input class="tab-radio" type="radio" name="deck-tab" id="tab-control" checked><input class="tab-radio" type="radio" name="deck-tab" id="tab-perform"><input class="tab-radio" type="radio" name="deck-tab" id="tab-calibrate"><input class="tab-radio" type="radio" name="deck-tab" id="tab-setup">
<main class="shell"><div id="error" class="error"></div><div id="toast" class="toast">Saved</div>
<section class="hero"><div class="panel hero-copy"><div><div class="eyebrow">SINGLE-EYE ANIMATRONIC</div><h1>Look alive.</h1><p>Direct gaze, shape the eyelids, run performance motions, or hand control to the salvaged dual joysticks.</p></div><div class="row spread"><span id="signal" class="signal-readout">OUTPUTS DETACHED</span><div class="row"><button class="primary" onclick="power('enable')">Enable outputs</button><button class="ghost needs-power" onclick="motion('center')">Centre</button><button class="amber needs-power" onclick="motion('blink')">Blink</button></div></div></div>
<div class="panel eye-stage"><div class="reticle"></div><div class="eye"><div id="topLid" class="lid top"></div><div id="iris" class="iris"></div><div id="bottomLid" class="lid bottom"></div></div></div></section>
<nav class="tabs" role="tablist"><label class="tabbtn" for="tab-control" role="tab">CONTROL</label><label class="tabbtn" for="tab-perform" role="tab">PERFORM</label><label class="tabbtn" for="tab-calibrate" role="tab">CALIBRATE</label><label class="tabbtn" for="tab-setup" role="tab">SETUP</label></nav>

<section id="page-control" class="page active"><div class="control-grid"><div class="panel"><div class="row spread"><h2>Gaze vector</h2><span class="eyebrow">TOUCH / DRAG</span></div><div id="pad" class="pad"><span class="pad-label n">UP</span><span class="pad-label s">DOWN</span><span class="pad-label w">LEFT</span><span class="pad-label e">RIGHT</span><div id="padDot" class="pad-dot"></div></div><div class="telemetry"><div class="metric"><span class="muted small">HORIZONTAL</span><b id="xv">0</b></div><div class="metric"><span class="muted small">VERTICAL</span><b id="yv">0</b></div></div></div>
<div class="panel"><h2>Control surface</h2><h3>Input authority</h3><div class="modes"><button class="mode" data-mode="web">WEB UI</button><button class="mode" data-mode="classic">JOYSTICK 1</button><button class="mode" data-mode="dual">DUAL STICK</button></div><p class="small muted">Classic: J1 moves the eye and lids follow vertical gaze. Dual: J1 moves the eye; J2 controls the lids.</p><div class="field"><div class="field-head"><label for="top">Top lid openness</label><span id="tv" class="value">100%</span></div><input id="top" class="needs-power" type="range" min="0" max="100" value="100"></div><div class="field"><div class="field-head"><label for="bottom">Bottom lid openness</label><span id="bv" class="value">100%</span></div><input id="bottom" class="needs-power" type="range" min="0" max="100" value="100"></div><div class="row"><button class="primary needs-power" onclick="sendControl(true)">Apply pose</button><button class="ghost needs-power" onclick="setPose(0,0,100,100)">Open + centre</button></div></div></div></section>

<section id="page-perform" class="page"><div class="panel"><div class="row spread"><div><div class="eyebrow">ORIGINAL MOTION SET // REBUILT</div><h2>Performance console</h2></div><div><span class="muted small">ACTIVE </span><span id="motionState" class="value">IDLE</span></div></div><div class="motion-grid"><button class="motion needs-power" data-motion="blink"><strong>Blink</strong><span>Close, hold, restore</span></button><button class="motion needs-power" data-motion="scan"><strong>Scan</strong><span>Side-to-side with blinks</span></button><button class="motion needs-power" data-motion="diagonal"><strong>Diagonal</strong><span>Corner sweep and eyelid tracking</span></button><button class="motion needs-power" data-motion="roll"><strong>Roll</strong><span>Full circular eye roll</span></button><button class="motion needs-power" data-motion="center"><strong>Centre</strong><span>Return to neutral gaze</span></button><button class="motion needs-power" data-motion="random"><strong>Random show</strong><span>Continuous mixed performance</span></button></div><div class="row" style="margin-top:13px"><button class="amber" onclick="motion('stop')">End show + centre</button><span class="small muted">Animation selection never enables outputs automatically.</span></div></div></section>

<section id="page-calibrate" class="page"><div class="panel"><div class="lock"><button id="calLock" class="amber" onclick="toggleCalibration()">Unlock calibration</button><div><h2>Mechanical calibration</h2><p class="callout">Only unlock this section when linkages are free or the mechanism has already been mechanically centred. STOP removes pulses; it does not cut the external servo rail.</p></div></div><div id="servos" class="servo-grid"></div></div><div class="panel" style="margin-top:16px"><div class="row spread"><div><div class="eyebrow">OPTIONAL INPUT</div><h2>Dual-joystick calibration</h2></div><span class="small muted">J1X/J1Y/J2X/J2Y · GPIO 0/1/3/4</span></div><p class="small muted">Capture both stick centres while released. Then move one axis fully to each end and capture Min and Max. Switch inputs are GPIO20 and GPIO21.</p><div id="joysticks" class="joy-grid"></div></div></section>

<section id="page-setup" class="page"><div class="grid"><div class="panel"><div class="eyebrow">DIRECT CONTROL LINK</div><h2>Connectivity</h2><div class="network-grid"><div class="net-card"><span class="muted small">DEVICE ACCESS POINT</span><b id="apSsid">AnimatronicEye</b><span id="apIp" class="mono">192.168.4.1</span></div><div class="net-card"><span class="muted small">RADIO MODE</span><b id="staSsid">AP only</b><span id="staIp" class="mono">Router connection disabled</span></div></div><p class="callout">This build deliberately runs AP-only for maximum ESP32-C3 stability inside the chassis. Connect directly to <b>AnimatronicEye</b>; every control and animation remains available.</p></div>
<div class="panel"><div class="eyebrow">DEVICE TELEMETRY</div><h2>System</h2><div class="telemetry"><div class="metric"><span class="muted small">FIRMWARE</span><b id="firmware">—</b></div><div class="metric"><span class="muted small">UPTIME</span><b id="uptime">—</b></div><div class="metric"><span class="muted small">FREE HEAP</span><b id="heap">—</b></div><div class="metric"><span class="muted small">STICK SWITCHES</span><b id="switches">—</b></div></div><div class="row" style="margin-top:14px"><a href="/basic"><button class="ghost">Open recovery UI</button></a><button class="danger" onclick="resetDefaults()">Restore motion defaults</button></div></div></div></section>
</main>
<script>
)HTML";

constexpr char INDEX_END[] PROGMEM = "\n</script></body></html>\n";

int clampAngle(int angle) {
  return constrain(angle, 0, 180);
}

int lerpInteger(int from, int to, int numerator, int denominator) {
  if (denominator == 0) return from;
  return from + ((to - from) * numerator) / denominator;
}

int logicalToServoAngle(uint8_t index, int logicalValue) {
  logicalValue = constrain(logicalValue, -100, 100);
  const ServoCalibration &c = servoCal[index];
  if (logicalValue < 0) return clampAngle(lerpInteger(c.a, c.center, logicalValue + 100, 100));
  return clampAngle(lerpInteger(c.center, c.b, logicalValue, 100));
}

int opennessToServoAngle(uint8_t index, int openness) {
  return logicalToServoAngle(index, (constrain(openness, 0, 100) * 2) - 100);
}

void setServoTarget(uint8_t index, int angle) {
  if (index >= SERVO_COUNT) return;
  servoTarget[index] = clampAngle(angle);
}

void attachServo(uint8_t index) {
  if (index >= SERVO_COUNT || servoEnabled[index]) return;
  servo[index].setPeriodHertz(50);
  servoCurrent[index] = clampAngle(servoCal[index].center);
  servoTarget[index] = servoCurrent[index];
  servo[index].attach(SERVO_PINS[index], SERVO_MIN_US, SERVO_MAX_US);
  if (!servo[index].attached()) {
    Serial.printf("ERROR: could not attach servo %u on GPIO %u\n", index, SERVO_PINS[index]);
    return;
  }
  servo[index].write((int)servoCurrent[index]);
  servoEnabled[index] = true;
}

void detachServo(uint8_t index) {
  if (index >= SERVO_COUNT || !servoEnabled[index]) return;
  servo[index].detach();
  servoEnabled[index] = false;
}

void detachAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) detachServo(i);
}

bool anyServoEnabled() {
  for (bool enabled : servoEnabled) if (enabled) return true;
  return false;
}

bool allServosEnabled() {
  for (bool enabled : servoEnabled) if (!enabled) return false;
  return true;
}

const char *motionName(MotionId motion) {
  switch (motion) {
    case MOTION_BLINK: return "blink";
    case MOTION_SCAN: return "scan";
    case MOTION_DIAGONAL: return "diagonal";
    case MOTION_ROLL: return "roll";
    case MOTION_CENTER: return "center";
    case MOTION_RANDOM: return "random";
    default: return "idle";
  }
}

void getMotionFrames(MotionId motion, const MotionFrame *&frames, uint8_t &count) {
  switch (motion) {
    case MOTION_BLINK: frames = BLINK_FRAMES; count = sizeof(BLINK_FRAMES) / sizeof(BLINK_FRAMES[0]); break;
    case MOTION_SCAN: frames = SCAN_FRAMES; count = sizeof(SCAN_FRAMES) / sizeof(SCAN_FRAMES[0]); break;
    case MOTION_DIAGONAL: frames = DIAGONAL_FRAMES; count = sizeof(DIAGONAL_FRAMES) / sizeof(DIAGONAL_FRAMES[0]); break;
    case MOTION_ROLL: frames = ROLL_FRAMES; count = sizeof(ROLL_FRAMES) / sizeof(ROLL_FRAMES[0]); break;
    case MOTION_CENTER: frames = CENTER_FRAMES; count = sizeof(CENTER_FRAMES) / sizeof(CENTER_FRAMES[0]); break;
    default: frames = nullptr; count = 0; break;
  }
}

void setPoseTargets(int x, int y, int topOpen, int bottomOpen) {
  setServoTarget(HORIZONTAL, logicalToServoAngle(HORIZONTAL, constrain(x, -100, 100)));
  setServoTarget(VERTICAL, logicalToServoAngle(VERTICAL, constrain(y, -100, 100)));
  setServoTarget(TOP_LID, opennessToServoAngle(TOP_LID, constrain(topOpen, 0, 100)));
  setServoTarget(BOTTOM_LID, opennessToServoAngle(BOTTOM_LID, constrain(bottomOpen, 0, 100)));
}

void applyMotionFrame(const MotionFrame &frame) {
  const int x = frame.x == KEEP_VALUE ? motionBaseX : frame.x;
  const int y = frame.y == KEEP_VALUE ? motionBaseY : frame.y;
  const int top = frame.topOpen == KEEP_VALUE ? motionBaseTop : frame.topOpen;
  const int bottom = frame.bottomOpen == KEEP_VALUE ? motionBaseBottom : frame.bottomOpen;
  setPoseTargets(x, y, top, bottom);
}

void beginConcreteMotion(MotionId motion) {
  activeMotion = motion;
  motionFrameIndex = 0;
  motionFrameStartedMs = millis();
  motionBaseX = webX;
  motionBaseY = webY;
  motionBaseTop = webTopOpen;
  motionBaseBottom = webBottomOpen;
  const MotionFrame *frames;
  uint8_t count;
  getMotionFrames(motion, frames, count);
  if (frames && count) applyMotionFrame(frames[0]);
}

void cancelMotion() {
  activeMotion = MOTION_NONE;
  randomShow = false;
  motionFrameIndex = 0;
}

bool startMotion(MotionId motion) {
  if (!allServosEnabled()) return false;
  controlMode = MODE_WEB;
  blinkUntilMs = 0;
  randomShow = motion == MOTION_RANDOM;
  MotionId concrete = motion;
  if (randomShow) concrete = (MotionId)(MOTION_BLINK + (esp_random() % 5));
  beginConcreteMotion(concrete);
  return true;
}

void updateMotion() {
  if (activeMotion == MOTION_NONE) return;
  const MotionFrame *frames;
  uint8_t count;
  getMotionFrames(activeMotion, frames, count);
  if (!frames || !count) { cancelMotion(); return; }
  if (millis() - motionFrameStartedMs < frames[motionFrameIndex].durationMs) return;
  ++motionFrameIndex;
  if (motionFrameIndex >= count) {
    if (randomShow) {
      const MotionId next = (MotionId)(MOTION_BLINK + (esp_random() % 5));
      beginConcreteMotion(next);
    } else {
      const MotionId finished = activeMotion;
      activeMotion = MOTION_NONE;
      motionFrameIndex = 0;
      if (finished != MOTION_BLINK) { webX = 0; webY = 0; }
      applyControlTargets();
    }
    return;
  }
  motionFrameStartedMs = millis();
  applyMotionFrame(frames[motionFrameIndex]);
}

void loadConfiguration() {
  preferences.begin(CONFIG_NAMESPACE, false);
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "s%ua", i);
    servoCal[i].a = preferences.getInt(key, DEFAULT_SERVO_CAL[i].a);
    snprintf(key, sizeof(key), "s%uc", i);
    servoCal[i].center = preferences.getInt(key, DEFAULT_SERVO_CAL[i].center);
    snprintf(key, sizeof(key), "s%ub", i);
    servoCal[i].b = preferences.getInt(key, DEFAULT_SERVO_CAL[i].b);
  }
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "j%umin", i);
    axisCal[i].minimum = preferences.getInt(key, 0);
    snprintf(key, sizeof(key), "j%uc", i);
    axisCal[i].center = preferences.getInt(key, 2048);
    snprintf(key, sizeof(key), "j%umax", i);
    axisCal[i].maximum = preferences.getInt(key, 4095);
    snprintf(key, sizeof(key), "j%uinv", i);
    axisCal[i].inverted = preferences.getBool(key, false);
  }
}

void saveServoCalibration(uint8_t index) {
  char key[12];
  snprintf(key, sizeof(key), "s%ua", index); preferences.putInt(key, servoCal[index].a);
  snprintf(key, sizeof(key), "s%uc", index); preferences.putInt(key, servoCal[index].center);
  snprintf(key, sizeof(key), "s%ub", index); preferences.putInt(key, servoCal[index].b);
}

void saveAxisCalibration(uint8_t index) {
  char key[12];
  snprintf(key, sizeof(key), "j%umin", index); preferences.putInt(key, axisCal[index].minimum);
  snprintf(key, sizeof(key), "j%uc", index); preferences.putInt(key, axisCal[index].center);
  snprintf(key, sizeof(key), "j%umax", index); preferences.putInt(key, axisCal[index].maximum);
  snprintf(key, sizeof(key), "j%uinv", index); preferences.putBool(key, axisCal[index].inverted);
}

void resetConfiguration() {
  cancelMotion();
  blinkUntilMs = 0;
  detachAllServos();
  preferences.clear();
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) servoCal[i] = DEFAULT_SERVO_CAL[i];
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) axisCal[i] = {0, 2048, 4095, false};
  controlMode = MODE_WEB;
}

void readJoysticks() {
  if (millis() - lastJoystickReadMs < 10) return;
  lastJoystickReadMs = millis();
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) joystickRaw[i] = analogRead(JOYSTICK_AXIS_PINS[i]);
}

int calibratedAxis(uint8_t index) {
  const AxisCalibration &c = axisCal[index];
  int raw = joystickRaw[index];
  int result = 0;
  if (raw < c.center - JOYSTICK_DEADZONE) {
    result = lerpInteger(-100, 0, raw - c.minimum, max(1, c.center - JOYSTICK_DEADZONE - c.minimum));
  } else if (raw > c.center + JOYSTICK_DEADZONE) {
    result = lerpInteger(0, 100, raw - (c.center + JOYSTICK_DEADZONE), max(1, c.maximum - c.center - JOYSTICK_DEADZONE));
  }
  result = constrain(result, -100, 100);
  return c.inverted ? -result : result;
}

void applyControlTargets() {
  int x = webX;
  int y = webY;
  int topOpen = webTopOpen;
  int bottomOpen = webBottomOpen;

  if (controlMode != MODE_WEB) {
    x = calibratedAxis(0);
    y = calibratedAxis(1);
    if (controlMode == MODE_JOYSTICK_CLASSIC) {
      topOpen = map(y, -100, 100, 100, 0);
      bottomOpen = map(y, -100, 100, 0, 100);
    } else {
      topOpen = map(calibratedAxis(2), -100, 100, 0, 100);
      bottomOpen = map(calibratedAxis(3), -100, 100, 0, 100);
    }
  }

  const bool switchBlink = controlMode != MODE_WEB &&
    (digitalRead(JOYSTICK_SWITCH_1_PIN) == LOW || digitalRead(JOYSTICK_SWITCH_2_PIN) == LOW);
  if (switchBlink || (int32_t)(blinkUntilMs - millis()) > 0) {
    topOpen = 0;
    bottomOpen = 0;
  }

  setPoseTargets(x, y, topOpen, bottomOpen);
}

void updateServos() {
  if (millis() - lastServoTickMs < SERVO_TICK_MS) return;
  lastServoTickMs = millis();
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    if (!servoEnabled[i]) continue;
    const float difference = servoTarget[i] - servoCurrent[i];
    const float step = SERVO_STEP_DEGREES[i];
    if (fabsf(difference) <= step) servoCurrent[i] = servoTarget[i];
    else servoCurrent[i] += difference > 0 ? step : -step;
    servo[i].write((int)lroundf(servoCurrent[i]));
  }
}

const char *modeName() {
  if (controlMode == MODE_JOYSTICK_CLASSIC) return "classic";
  if (controlMode == MODE_JOYSTICK_DUAL) return "dual";
  return "web";
}

void sendState() {
  String json;
  json.reserve(3000);
  json = "{\"firmware\":\"";
  json += FIRMWARE_VERSION;
  json += "\",\"mode\":\"";
  json += modeName();
  json += "\",\"anyEnabled\":";
  json += anyServoEnabled() ? "true" : "false";
  json += ",\"allEnabled\":";
  json += allServosEnabled() ? "true" : "false";
  json += ",\"animation\":\"";
  json += randomShow ? "random" : motionName(activeMotion);
  json += "\",\"animationDetail\":\"";
  json += motionName(activeMotion);
  json += "\",\"control\":{\"x\":"; json += webX;
  json += ",\"y\":"; json += webY;
  json += ",\"top\":"; json += webTopOpen;
  json += ",\"bottom\":"; json += webBottomOpen;
  json += "},\"network\":{\"apSsid\":\"";
  json += AP_SSID;
  json += "\",\"apIp\":\"";
  json += WiFi.softAPIP().toString();
  json += "\",\"staEnabled\":false,\"staConnected\":false},\"uptime\":";
  json += millis();
  json += ",\"heap\":";
  json += ESP.getFreeHeap();
  json += ",\"switch1\":";
  json += digitalRead(JOYSTICK_SWITCH_1_PIN) == HIGH ? "true" : "false";
  json += ",\"switch2\":";
  json += digitalRead(JOYSTICK_SWITCH_2_PIN) == HIGH ? "true" : "false";
  json += ",\"servos\":[";
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    if (i) json += ',';
    json += "{\"pin\":"; json += SERVO_PINS[i];
    json += ",\"enabled\":"; json += servoEnabled[i] ? "true" : "false";
    json += ",\"current\":"; json += String(servoCurrent[i], 1);
    json += ",\"target\":"; json += String(servoTarget[i], 1);
    json += ",\"a\":"; json += servoCal[i].a;
    json += ",\"center\":"; json += servoCal[i].center;
    json += ",\"b\":"; json += servoCal[i].b;
    json += '}';
  }
  json += "],\"joystick\":[";
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (i) json += ',';
    json += "{\"pin\":"; json += JOYSTICK_AXIS_PINS[i];
    json += ",\"raw\":"; json += joystickRaw[i];
    json += ",\"min\":"; json += axisCal[i].minimum;
    json += ",\"center\":"; json += axisCal[i].center;
    json += ",\"max\":"; json += axisCal[i].maximum;
    json += ",\"inverted\":"; json += axisCal[i].inverted ? "true" : "false";
    json += '}';
  }
  json += "]}";
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send(200, "application/json", json);
}

bool validIndexArgument(const char *name, int maximum, int &value) {
  if (!server.hasArg(name)) return false;
  value = server.arg(name).toInt();
  return value >= 0 && value < maximum;
}

void sendOk() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "{\"ok\":true}");
}

void sendBadRequest(const char *message) {
  String json = "{\"ok\":false,\"error\":\"";
  json += message;
  json += "\"}";
  server.send(400, "application/json", json);
}

void sendConflict(const char *message) {
  String json = "{\"ok\":false,\"error\":\"";
  json += message;
  json += "\"}";
  server.send(409, "application/json", json);
}

void handleBasicUi() {
  const String command = server.arg("cmd");
  int index = server.hasArg("servo") ? server.arg("servo").toInt() : -1;

  if (command == "stopall") {
    cancelMotion();
    blinkUntilMs = 0;
    detachAllServos();
  } else if (command == "stop" && index >= 0 && index < SERVO_COUNT) {
    cancelMotion();
    detachServo(index);
  } else if (command == "enable" && index >= 0 && index < SERVO_COUNT) {
    attachServo(index);
  } else if (command == "move" && index >= 0 && index < SERVO_COUNT && server.hasArg("angle")) {
    cancelMotion();
    controlMode = MODE_WEB;
    setServoTarget(index, server.arg("angle").toInt());
  } else if (command == "save" && index >= 0 && index < SERVO_COUNT) {
    const String slot = server.arg("slot");
    const int angle = clampAngle((int)lroundf(servoTarget[index]));
    if (slot == "a") servoCal[index].a = angle;
    else if (slot == "center") servoCal[index].center = angle;
    else if (slot == "b") servoCal[index].b = angle;
    if (slot == "a" || slot == "center" || slot == "b") saveServoCalibration(index);
  } else if (command == "control") {
    cancelMotion();
    blinkUntilMs = 0;
    if (server.hasArg("x")) webX = constrain(server.arg("x").toInt(), -100, 100);
    if (server.hasArg("y")) webY = constrain(server.arg("y").toInt(), -100, 100);
    if (server.hasArg("top")) webTopOpen = constrain(server.arg("top").toInt(), 0, 100);
    if (server.hasArg("bottom")) webBottomOpen = constrain(server.arg("bottom").toInt(), 0, 100);
    controlMode = MODE_WEB;
    applyControlTargets();
  } else if (command == "blink") {
    startMotion(MOTION_BLINK);
  } else if (command == "center") {
    startMotion(MOTION_CENTER);
  } else if (command == "scan") {
    startMotion(MOTION_SCAN);
  }

  static const char *SERVO_NAMES[SERVO_COUNT] = {"Top lid", "Bottom lid", "Eye horizontal", "Eye vertical"};
  static const char *SLOT_A[SERVO_COUNT] = {"Closed", "Closed", "Left", "Up"};
  static const char *SLOT_B[SERVO_COUNT] = {"Open", "Open", "Right", "Down"};

  String html;
  html.reserve(9000);
  html += F("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<meta http-equiv='Cache-Control' content='no-store'><title>Animatronic Eye - Basic UI</title><style>");
  html += F("body{font-family:system-ui;background:#0d1117;color:#e6edf3;margin:0;padding:16px}main{max-width:850px;margin:auto}section{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:14px;margin:12px 0}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}a,button{display:inline-block;background:#238636;color:white;border:0;border-radius:7px;padding:10px 14px;text-decoration:none;font-weight:600}.stop{background:#da3633}.save{background:#1f6feb}input{font-size:18px;width:90px;padding:8px;background:#0d1117;color:white;border:1px solid #8b949e;border-radius:6px}small{color:#8b949e}.off{color:#f85149}.on{color:#3fb950}</style></head><body><main>");
  html += F("<h1>Animatronic Eye</h1><p><b>Basic no-JavaScript recovery control.</b> Every button reloads this page.</p><p><a class='stop' href='/basic?cmd=stopall'>STOP / detach all</a> <a href='/basic'>Refresh</a> <a class='save' href='/'>Full UI</a> <a class='save' href='/api/state'>Raw state</a></p>");

  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    html += F("<section><h2>"); html += SERVO_NAMES[i]; html += F(" - GPIO "); html += SERVO_PINS[i]; html += F("</h2><p>Signal: <b class='");
    html += servoEnabled[i] ? F("on'>ON") : F("off'>OFF");
    html += F("</b> | Current: "); html += (int)lroundf(servoCurrent[i]); html += F("&deg; | Target: "); html += (int)lroundf(servoTarget[i]); html += F("&deg;</p><div class='row'>");
    html += F("<a href='/basic?cmd=enable&servo="); html += i; html += F("'>Enable at saved centre</a>");
    html += F("<a class='stop' href='/basic?cmd=stop&servo="); html += i; html += F("'>Detach</a></div>");
    html += F("<form class='row' action='/basic' method='get'><input type='hidden' name='cmd' value='move'><input type='hidden' name='servo' value='"); html += i;
    html += F("'><label>Angle <input name='angle' type='number' min='0' max='180' value='"); html += (int)lroundf(servoTarget[i]); html += F("'></label><button type='submit'>Move</button></form>");
    html += F("<p>Saved: "); html += SLOT_A[i]; html += F(" "); html += servoCal[i].a; html += F("&deg; | Centre "); html += servoCal[i].center; html += F("&deg; | "); html += SLOT_B[i]; html += F(" "); html += servoCal[i].b; html += F("&deg;</p><div class='row'>");
    html += F("<a class='save' href='/basic?cmd=save&slot=a&servo="); html += i; html += F("'>Save as "); html += SLOT_A[i]; html += F("</a>");
    html += F("<a class='save' href='/basic?cmd=save&slot=center&servo="); html += i; html += F("'>Save as centre</a>");
    html += F("<a class='save' href='/basic?cmd=save&slot=b&servo="); html += i; html += F("'>Save as "); html += SLOT_B[i]; html += F("</a></div></section>");
  }

  html += F("<section><h2>Web control</h2><form action='/basic' method='get'><input type='hidden' name='cmd' value='control'><div class='row'><label>X (-100..100)<br><input name='x' type='number' min='-100' max='100' value='"); html += webX;
  html += F("'></label><label>Y (-100..100)<br><input name='y' type='number' min='-100' max='100' value='"); html += webY;
  html += F("'></label><label>Top open %<br><input name='top' type='number' min='0' max='100' value='"); html += webTopOpen;
  html += F("'></label><label>Bottom open %<br><input name='bottom' type='number' min='0' max='100' value='"); html += webBottomOpen;
  html += F("'></label><button type='submit'>Apply</button></div></form><p><a href='/basic?cmd=blink'>Blink</a> <a href='/basic?cmd=center'>Centre</a> <a href='/basic?cmd=scan'>Scan</a></p></section>");
  html += F("<section><h2>Device</h2><p>AP-only mode: <b>"); html += AP_SSID; html += F("</b><br>Address: <b>http://192.168.4.1</b><br>Router association: <b>disabled</b><br>Uptime: "); html += millis() / 1000; html += F(" s</p><small>This page intentionally uses no JavaScript and does not require a COM port.</small></section></main></body></html>");

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.send(200, "text/html; charset=utf-8", html);
}

void writeProgmem(PGM_P content, size_t length) {
  constexpr size_t CHUNK_SIZE = 1024;
  for (size_t offset = 0; offset < length; offset += CHUNK_SIZE) {
    const size_t remaining = length - offset;
    server.sendContent_P(content + offset, min(remaining, CHUNK_SIZE));
    delay(0);
  }
}

void sendProgmemAsset(const char *contentType, PGM_P content, bool includeController = false) {
  const size_t contentLength = strlen_P(content);
  const size_t controllerLength = includeController ? sizeof(APP_JS) - 1 : 0;
  const size_t endingLength = includeController ? sizeof(INDEX_END) - 1 : 0;

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.setContentLength(contentLength + controllerLength + endingLength);
  server.send(200, contentType, "");
  writeProgmem(content, contentLength);
  if (includeController) {
    writeProgmem(APP_JS, controllerLength);
    writeProgmem(INDEX_END, endingLength);
  }
}

void configureWebServer() {
  server.on("/", HTTP_GET, []() {
    sendProgmemAsset("text/html; charset=utf-8", INDEX_HTML, true);
  });
  server.on("/app.js", HTTP_GET, []() {
    sendProgmemAsset("application/javascript; charset=utf-8", APP_JS);
  });
  server.on("/basic", HTTP_GET, handleBasicUi);
  server.on("/api/state", HTTP_GET, sendState);

  server.on("/api/power", HTTP_GET, []() {
    const String cmd = server.arg("cmd");
    int index = server.hasArg("servo") ? server.arg("servo").toInt() : -1;
    if (cmd == "stop") {
      cancelMotion();
      blinkUntilMs = 0;
      if (index >= 0 && index < SERVO_COUNT) detachServo(index); else detachAllServos();
    } else if (cmd == "enable") {
      if (index >= 0 && index < SERVO_COUNT) attachServo(index);
      else {
        for (uint8_t i = 0; i < SERVO_COUNT; ++i) attachServo(i);
        applyControlTargets();
      }
    } else return sendBadRequest("invalid command");
    sendState();
  });

  server.on("/api/mode", HTTP_GET, []() {
    const String value = server.arg("value");
    cancelMotion();
    blinkUntilMs = 0;
    if (value == "web") controlMode = MODE_WEB;
    else if (value == "classic") controlMode = MODE_JOYSTICK_CLASSIC;
    else if (value == "dual") controlMode = MODE_JOYSTICK_DUAL;
    else return sendBadRequest("invalid mode");
    applyControlTargets();
    sendState();
  });

  server.on("/api/control", HTTP_GET, []() {
    cancelMotion();
    blinkUntilMs = 0;
    controlMode = MODE_WEB;
    if (server.hasArg("x")) webX = constrain(server.arg("x").toInt(), -100, 100);
    if (server.hasArg("y")) webY = constrain(server.arg("y").toInt(), -100, 100);
    if (server.hasArg("top")) webTopOpen = constrain(server.arg("top").toInt(), 0, 100);
    if (server.hasArg("bottom")) webBottomOpen = constrain(server.arg("bottom").toInt(), 0, 100);
    applyControlTargets();
    sendOk();
  });

  server.on("/api/blink", HTTP_GET, []() {
    if (!startMotion(MOTION_BLINK)) return sendConflict("enable all outputs before starting a motion");
    sendState();
  });

  server.on("/api/animation", HTTP_GET, []() {
    const String name = server.arg("name");
    if (name == "stop") {
      cancelMotion();
      blinkUntilMs = 0;
      controlMode = MODE_WEB;
      webX = 0; webY = 0; webTopOpen = 100; webBottomOpen = 100;
      applyControlTargets();
      return sendState();
    }
    MotionId motion = MOTION_NONE;
    if (name == "blink") motion = MOTION_BLINK;
    else if (name == "scan") motion = MOTION_SCAN;
    else if (name == "diagonal") motion = MOTION_DIAGONAL;
    else if (name == "roll") motion = MOTION_ROLL;
    else if (name == "center") motion = MOTION_CENTER;
    else if (name == "random") motion = MOTION_RANDOM;
    else return sendBadRequest("invalid animation");
    if (!startMotion(motion)) return sendConflict("enable all outputs before starting a motion");
    sendState();
  });

  server.on("/api/cal-move", HTTP_GET, []() {
    int index;
    if (!validIndexArgument("servo", SERVO_COUNT, index) || !server.hasArg("angle")) return sendBadRequest("invalid servo or angle");
    cancelMotion();
    blinkUntilMs = 0;
    controlMode = MODE_WEB;
    setServoTarget(index, server.arg("angle").toInt());
    sendOk();
  });

  server.on("/api/cal-save", HTTP_GET, []() {
    int index;
    if (!validIndexArgument("servo", SERVO_COUNT, index) || !server.hasArg("angle")) return sendBadRequest("invalid servo or angle");
    const int angle = clampAngle(server.arg("angle").toInt());
    const String slot = server.arg("slot");
    if (slot == "a") servoCal[index].a = angle;
    else if (slot == "center") servoCal[index].center = angle;
    else if (slot == "b") servoCal[index].b = angle;
    else return sendBadRequest("invalid slot");
    saveServoCalibration(index);
    sendState();
  });

  server.on("/api/joy-capture", HTTP_GET, []() {
    int index;
    if (!validIndexArgument("axis", AXIS_COUNT, index)) return sendBadRequest("invalid axis");
    const String slot = server.arg("slot");
    if (slot == "min") axisCal[index].minimum = joystickRaw[index];
    else if (slot == "center") axisCal[index].center = joystickRaw[index];
    else if (slot == "max") axisCal[index].maximum = joystickRaw[index];
    else return sendBadRequest("invalid slot");
    saveAxisCalibration(index);
    sendState();
  });

  server.on("/api/joy-invert", HTTP_GET, []() {
    int index;
    if (!validIndexArgument("axis", AXIS_COUNT, index)) return sendBadRequest("invalid axis");
    axisCal[index].inverted = server.arg("value") == "1";
    saveAxisCalibration(index);
    sendState();
  });

  server.on("/api/defaults", HTTP_GET, []() {
    resetConfiguration();
    sendState();
  });

  server.on("/api/wifi", HTTP_POST, []() {
    sendConflict("local Wi-Fi is disabled in the AP-only firmware");
  });

  auto captiveRedirect = []() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "Open Animatronic Eye");
  };
  server.on("/generate_204", HTTP_GET, captiveRedirect);
  server.on("/hotspot-detect.html", HTTP_GET, captiveRedirect);
  server.on("/connecttest.txt", HTTP_GET, captiveRedirect);
  server.on("/ncsi.txt", HTTP_GET, captiveRedirect);

  server.onNotFound([]() { server.send(404, "application/json", "{\"ok\":false,\"error\":\"not found\"}"); });
  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.printf("\nAnimatronic Eye ESP32-C3 %s starting\n", FIRMWARE_VERSION);
  Serial.println("Servo control signals are OFF until enabled in the Web UI.");

  loadConfiguration();
  analogReadResolution(12);
  for (uint8_t pin : JOYSTICK_AXIS_PINS) pinMode(pin, INPUT);
  pinMode(JOYSTICK_SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_SWITCH_2_PIN, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  const IPAddress apIp(192, 168, 4, 1);
  const IPAddress apGateway(192, 168, 4, 1);
  const IPAddress apSubnet(255, 255, 255, 0);
  if (!WiFi.softAPConfig(apIp, apGateway, apSubnet)) Serial.println("ERROR: AP static IP configuration failed");
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) Serial.println("ERROR: Wi-Fi access point failed to start");
  if (!dnsServer.start(53, "*", apIp)) Serial.println("WARNING: captive portal DNS did not start");
  configureWebServer();
  Serial.printf("Device AP '%s' active\n", AP_SSID);
  Serial.print("Open http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  static bool blinkWasActive = false;
  dnsServer.processNextRequest();
  server.handleClient();
  readJoysticks();
  const bool blinkIsActive = (int32_t)(blinkUntilMs - millis()) > 0;
  if (activeMotion != MOTION_NONE) updateMotion();
  else if (controlMode != MODE_WEB || blinkIsActive || blinkWasActive) applyControlTargets();
  blinkWasActive = blinkIsActive;
  updateServos();
  delay(1);
}
