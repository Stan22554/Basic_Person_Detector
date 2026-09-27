#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_system.h>
#include <math.h>

// UART2 defaults for a classic ESP32 DevKit. Change these for a different board.
static constexpr int SENSOR_RX_PIN = 16; // ESP32 RX2  <- LD2410C TX
static constexpr int SENSOR_TX_PIN = 17; // ESP32 TX2  -> LD2410C RX
static constexpr int SENSOR_OUT_PIN = 27; // LD2410C OUT -> ESP32 GPIO27 (non-strapping GPIO)
static constexpr uint32_t SENSOR_BAUD = 256000;
static constexpr size_t MAX_PAYLOAD = 128;
static constexpr size_t EVENT_COUNT = 40;

// Optional local configuration file (copy include/config.h.example to include/config.h).
#if __has_include("config.h")
#include "config.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef AP_SSID
#define AP_SSID "LD2410C-Presence"
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD "presence"
#endif

WebServer server(80);
Preferences preferences;
HardwareSerial SensorSerial(2);
bool stationConfigured = false;
String stationSsid;
String stationPassword;
uint8_t maxGate = 8;
uint16_t noTargetTimeout = 5;
bool sensorConfigStored = false;
uint8_t movingSensitivity[9] = {50, 50, 50, 50, 50, 50, 50, 50, 50};
uint8_t stationarySensitivity[9] = {50, 50, 50, 50, 50, 50, 50, 50, 50};
int previousWifiStatus = -1;
uint32_t lastWifiRetryAt = 0;
uint32_t lastWifiStatusAt = 0;

struct SensorEvent {
  uint32_t sequence = 0;
  uint32_t atMs = 0;
  uint16_t payloadLength = 0;
  bool validFooter = false;
  bool targetReport = false;
  bool invalidTargetReport = false;
  char raw[(MAX_PAYLOAD + 10) * 3 + 1] = {};
};
SensorEvent events[EVENT_COUNT];
uint32_t eventSequence = 0;
uint32_t frameCount = 0;
uint32_t serialByteCount = 0;
uint32_t lastReportAt = 0;
uint32_t movingDistance = 0;
uint32_t stationaryDistance = 0;
uint32_t detectionDistance = 0;
uint8_t movingEnergy = 0;
uint8_t stationaryEnergy = 0;
uint8_t targetState = 0;
bool hasReport = false;

uint8_t frameBuffer[MAX_PAYLOAD + 10] = {};
size_t frameLength = 0;
size_t expectedFrameLength = 0;
uint16_t expectedPayloadLength = 0;
uint8_t headerMatch = 0;
const uint8_t FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t FRAME_FOOTER[] = {0xF8, 0xF7, 0xF6, 0xF5};

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>LD2410C Presence</title>
<style>
:root{color-scheme:dark;--bg:#080d17;--panel:#111a2a;--line:#25344c;--muted:#91a1bb;--text:#edf4ff;--cyan:#45d7c4;--amber:#ffb84d;--blue:#6b83a7;--purple:#bd91ff}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(ellipse at 50% -20%,#1b3150 0,transparent 55%),var(--bg);color:var(--text);font:15px/1.45 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}.wrap{max-width:1180px;margin:auto;padding:28px 22px 44px}.top{display:flex;align-items:center;justify-content:space-between;gap:20px;margin-bottom:22px}.brand{display:flex;align-items:center;gap:13px}.mark{width:42px;height:42px;border-radius:13px;background:linear-gradient(145deg,#44e3c4,#5587ff);display:grid;place-items:center;color:#08111a;font-weight:900;font-size:20px;box-shadow:0 6px 24px #36d9c544}.eyebrow{font-size:11px;letter-spacing:.16em;color:var(--cyan);font-weight:800;text-transform:uppercase}.title{font-size:22px;font-weight:750;letter-spacing:-.03em}.pill{border:1px solid var(--line);border-radius:99px;padding:8px 12px;color:var(--muted);font-size:12px}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#68778f;margin-right:7px}.dot.on{background:#55e0bd;box-shadow:0 0 12px #55e0bd}.hero{display:grid;grid-template-columns:minmax(0,1.02fr) minmax(300px,.98fr);gap:18px}.card{background:linear-gradient(150deg,#141e30,#0e1624);border:1px solid var(--line);border-radius:20px;box-shadow:0 16px 40px #0003}.stage{min-height:348px;padding:26px;display:flex;flex-direction:column;align-items:center;justify-content:center;position:relative;overflow:hidden}.stage:before{content:"";position:absolute;width:270px;height:270px;border:1px solid #8ba3c014;border-radius:50%;box-shadow:0 0 0 34px #8ba3c009,0 0 0 68px #8ba3c006}.radar{position:absolute;width:212px;height:212px;border-radius:50%;background:repeating-radial-gradient(circle,transparent 0 33px,#a7bed015 34px 35px);border:1px solid #a7bed020}.beam{position:absolute;inset:0;border-radius:50%;background:conic-gradient(from 0deg,transparent 0 315deg,#56e2c420 360deg);animation:spin 5s linear infinite}@keyframes spin{to{transform:rotate(360deg)}}.orb{z-index:1;width:82px;height:82px;border-radius:50%;display:grid;place-items:center;transition:all .35s ease;background:#6b83a7;color:#06101a;box-shadow:0 0 18px #6b83a733;position:relative}.orb:after{content:"";position:absolute;inset:-12px;border-radius:50%;border:1px solid currentColor;opacity:.17}.orb span{font-size:23px}.orb.none{background:#6b83a7;color:#9eb0c8;box-shadow:0 0 18px #6b83a733}.orb.moving{background:var(--amber);color:var(--amber)}.orb.stationary{background:var(--cyan);color:var(--cyan)}.orb.both{background:var(--purple);color:var(--purple)}.orb.moving span,.orb.stationary span,.orb.both span{color:#07111c}.state{z-index:1;margin-top:22px;font-size:22px;font-weight:750}.hint{z-index:1;color:var(--muted);font-size:12px;margin-top:4px}.range{z-index:1;width:min(280px,90%);margin-top:24px}.rangehead{display:flex;justify-content:space-between;color:var(--muted);font-size:11px;margin-bottom:8px}.track{height:5px;background:#26364c;border-radius:9px;overflow:hidden}.fill{height:100%;width:0;background:linear-gradient(90deg,#45d7c4,#ffbd54);transition:width .3s;border-radius:9px}.metrics{padding:22px}.section-title{font-size:13px;font-weight:750;color:#dce8f8;margin:0 0 15px}.readings{display:grid;grid-template-columns:1fr 1fr;gap:10px}.reading{padding:14px;border:1px solid #24334a;background:#0b1320;border-radius:13px;min-height:83px}.reading .label{font-size:11px;color:var(--muted);margin-bottom:6px}.reading .value{font-size:21px;font-weight:730;letter-spacing:-.03em}.unit{font-size:12px;color:var(--muted);font-weight:500;margin-left:4px}.reading.move .value{color:var(--amber)}.reading.still .value{color:var(--cyan)}.reading.detect .value{color:#e6d3ff}.gridcard{margin-top:18px;padding:22px}.data-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.datum{border:1px solid #24334a;background:#0b1320;border-radius:12px;padding:12px 14px}.datum .k{font-size:10px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted)}.datum .v{font-size:14px;font-weight:650;margin-top:5px;overflow-wrap:anywhere}.streamcard{margin-top:18px;overflow:hidden}.streamhead{padding:18px 22px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid var(--line)}.streamhead h2{font-size:14px;margin:0}.streammeta{color:var(--muted);font-size:11px}.tablewrap{max-height:320px;overflow:auto}table{border-collapse:collapse;width:100%;font-size:12px;text-align:left}th{position:sticky;top:0;background:#111a2a;color:var(--muted);font-size:10px;text-transform:uppercase;letter-spacing:.1em}th,td{padding:11px 16px;border-bottom:1px solid #202d40}td{color:#d8e4f5}td.hex{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;color:#9cdbd2;word-break:break-all;min-width:300px}.empty{text-align:center;padding:26px;color:var(--muted)}.footer{font-size:11px;color:var(--muted);text-align:center;margin-top:18px}.badge{border-radius:99px;background:#1c2940;padding:4px 8px;color:#b8c8df;font-size:10px}
.configcard{margin-top:18px;padding:22px}.config-layout{display:grid;grid-template-columns:minmax(240px,.75fr) minmax(0,1.5fr);gap:28px}.config-block{min-width:0}.config-block h3{margin:0 0 12px;font-size:15px}.config-block p,.config-note{color:var(--muted);font-size:12px;margin:0 0 14px}.form-row{display:grid;gap:6px;margin:12px 0}.form-row label{font-size:12px;color:var(--muted)}.form-row input{min-width:0;width:100%;padding:10px 11px;border-radius:8px;border:1px solid var(--line);background:#09111d;color:var(--text);font:inherit}.config-actions{display:flex;gap:9px;flex-wrap:wrap;margin-top:14px}.config-actions button{border:1px solid var(--line);border-radius:8px;background:#1a2940;color:var(--text);padding:9px 13px;font:inherit;cursor:pointer}.config-actions button[type=submit]{background:var(--cyan);color:#07111c;border-color:var(--cyan);font-weight:700}.config-actions button:disabled{opacity:.55;cursor:wait}.range-config{display:grid;grid-template-columns:minmax(150px,1fr) 100px;align-items:center;gap:14px;margin-bottom:14px}.range-config input{width:100%}.gate-table{width:100%;border-collapse:collapse;font-size:12px}.gate-table th,.gate-table td{padding:7px 6px;border-bottom:1px solid var(--line);text-align:left}.gate-table th{color:var(--muted);font-weight:600}.gate-table input{width:82px;padding:7px;border-radius:6px;border:1px solid var(--line);background:#09111d;color:var(--text);font:inherit}.config-message{min-height:18px;color:var(--cyan);font-size:12px;margin-top:10px}@media(max-width:760px){.hero{grid-template-columns:1fr}.stage{min-height:300px}.data-grid{grid-template-columns:repeat(2,1fr)}.wrap{padding:20px 14px 32px}.top{align-items:flex-start}.title{font-size:19px}.pill{white-space:nowrap}.config-layout{grid-template-columns:1fr}.configcard{padding:17px}.gate-table th,.gate-table td{padding:6px 3px}.gate-table input{width:68px}}
</style></head><body><main class="wrap"><header class="top"><div class="brand"><div class="mark">◉</div><div><div class="eyebrow">Sensor console</div><div class="title">LD2410C Presence</div></div></div><div class="pill"><i id="connection" class="dot"></i><span id="connectText">Connecting</span></div></header>
<section class="hero"><article class="card stage"><div class="radar"><div class="beam"></div></div><div id="orb" class="orb none"><span>◉</span></div><div id="state" class="state">Waiting for sensor</div><div id="hint" class="hint">No valid target report received yet</div><div class="range"><div class="rangehead"><span>PROXIMITY</span><span id="proximity">—</span></div><div class="track"><div id="rangeFill" class="fill"></div></div></div></article>
<article class="card metrics"><h2 class="section-title">Live target readings</h2><div class="readings"><div class="reading move"><div class="label">Moving target</div><div class="value"><span id="movingDistance">—</span><span class="unit">cm</span></div></div><div class="reading"><div class="label">Moving energy</div><div class="value"><span id="movingEnergy">—</span><span class="unit">/ 100</span></div></div><div class="reading still"><div class="label">Stationary target</div><div class="value"><span id="stationaryDistance">—</span><span class="unit">cm</span></div></div><div class="reading"><div class="label">Stationary energy</div><div class="value"><span id="stationaryEnergy">—</span><span class="unit">/ 100</span></div></div><div class="reading detect"><div class="label">Detection distance</div><div class="value"><span id="detectionDistance">—</span><span class="unit">cm</span></div></div><div class="reading"><div class="label">Target-state code</div><div class="value"><span id="targetCode">—</span></div></div></div></article></section>
<section class="card gridcard"><h2 class="section-title">Sensor &amp; link details</h2><div class="data-grid"><div class="datum"><div class="k">LD2410C OUT</div><div class="v" id="outPin">—</div></div><div class="datum"><div class="k">Last report</div><div class="v" id="reportAge">—</div></div><div class="datum"><div class="k">Valid frames</div><div class="v" id="frameCount">0</div></div><div class="datum"><div class="k">UART bytes</div><div class="v" id="byteCount">0</div></div><div class="datum"><div class="k">UART settings</div><div class="v">UART2 · 256000 · 8N1</div></div><div class="datum"><div class="k">Wi-Fi address</div><div class="v" id="ipAddress">—</div></div><div class="datum"><div class="k">Sensor UART</div><div class="v" id="uartPins">RX 16 / TX 17</div></div><div class="datum"><div class="k">Report type</div><div class="v">Standard target report</div></div></div></section>
<section class="card configcard"><h2 class="section-title">Configuration</h2><div class="config-layout"><div class="config-block"><h3>Wi-Fi network</h3><p>Set or replace the station network. The setup access point remains available. Leave the password blank to keep the saved password for the same network.</p><form id="wifiForm"><div class="form-row"><label for="wifiSsid">Network name (SSID)</label><input id="wifiSsid" name="ssid" maxlength="32" autocomplete="off"></div><div class="form-row"><label for="wifiPassword">Password</label><input id="wifiPassword" name="password" type="password" maxlength="63" autocomplete="new-password" placeholder="Saved password stays unchanged"></div><div class="config-actions"><button type="submit">Save Wi-Fi</button><button type="button" id="clearWifi">Clear Wi-Fi</button></div></form></div><div class="config-block"><h3>LD2410C gates</h3><p>Gate sensitivity is configured separately for moving and stationary targets. Maximum range uses 0.75 m gate steps; no-target hold is the sensor's global timeout.</p><form id="sensorForm"><div class="range-config"><label for="maxGate">Maximum range: <strong id="maxRangeLabel">6.75 m</strong></label><input id="maxGate" name="maxGate" type="range" min="0" max="8" step="1" value="8"></div><div class="range-config"><label for="timeout">No-target hold (seconds)</label><input id="timeout" name="timeout" type="number" min="0" max="65535" value="5"></div><div class="tablewrap"><table class="gate-table"><thead><tr><th>Gate</th><th>Moving</th><th>Stationary</th></tr></thead><tbody id="gateRows"></tbody></table></div><div class="config-actions"><button type="submit">Save and apply sensor settings</button></div></form></div></div><div class="config-message" id="configMessage" role="status"></div></section>
<section class="card streamcard"><div class="streamhead"><h2>Incoming UART transactions <span class="badge" id="streamCount">0 captured</span></h2><span class="streammeta">Newest first · raw frame bytes</span></div><div class="tablewrap"><table><thead><tr><th>Time</th><th>Kind</th><th>Payload</th><th>Frame (hex)</th></tr></thead><tbody id="transactions"><tr><td colspan="4" class="empty">Listening for LD2410C frames…</td></tr></tbody></table></div></section><div class="footer">Presence is derived from UART target reports; proximity intensity is strongest at shorter measured distance.</div></main>
<script>
const $=id=>document.getElementById(id);let lastEvent=0,seen=0;
function fmtDistance(n){return n===null||n===undefined?'—':n===0?'0':n.toLocaleString()}
function update(s){$('connection').classList.toggle('on',true);$('connectText').textContent='Device online';$('ipAddress').textContent=s.ip;$('frameCount').textContent=s.frames.toLocaleString();$('byteCount').textContent=s.bytes.toLocaleString();$('outPin').textContent=s.out?'HIGH · presence':'LOW · clear';$('reportAge').textContent=s.hasReport?`${(s.reportAgeMs/1000).toFixed(1)} s ago`:'No report yet';
$('movingDistance').textContent=s.hasReport?fmtDistance(s.movingCm):'—';$('stationaryDistance').textContent=s.hasReport?fmtDistance(s.stationaryCm):'—';$('detectionDistance').textContent=s.hasReport?fmtDistance(s.detectionCm):'—';$('movingEnergy').textContent=s.hasReport?s.movingEnergy:'—';$('stationaryEnergy').textContent=s.hasReport?s.stationaryEnergy:'—';$('targetCode').textContent=s.hasReport?s.targetState:'—';
let st=s.targetState,active=s.hasReport&&s.reportAgeMs<5000,kind=active?(st===1?'moving':st===2?'stationary':st===3?'both':'none'):'none';let names={none:active?'No Presence':'No recent presence report',moving:'Moving Presence',stationary:'Non-Moving Presence',both:'Moving + Non-Moving'};$('orb').className='orb '+kind;$('state').textContent=names[kind];$('hint').textContent=!active?'Waiting for a fresh target report':kind==='none'?'Sensor reports no target':`Sensor target state ${st}${st===3?' (both target types)':''}`;
let ds=[];if(kind==='moving'||kind==='both')ds.push(s.movingCm);if(kind==='stationary'||kind==='both')ds.push(s.stationaryCm);ds=ds.filter(x=>x>0);let near=ds.length?Math.min(...ds):0;let strength=near?Math.max(8,Math.min(100,100-near/600*100)):0;$('rangeFill').style.width=strength+'%';$('proximity').textContent=near?`${near} cm · ${Math.round(strength)}%`:'—';$('orb').style.opacity=near?String(.45+strength/180):'1';$('orb').style.transform=near?`scale(${.82+strength/250})`:'scale(1)';}
function esc(v){return String(v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function showConfigMessage(text){$('configMessage').textContent=text}
function updateMaxRange(){const gate=Number($('maxGate').value);$('maxRangeLabel').textContent=gate===0?'Gate 0 · under 0.75 m':`Gate ${gate} · about ${(gate*.75).toFixed(2)} m`}
async function loadConfig(){try{const response=await fetch('/api/config',{cache:'no-store'});if(!response.ok)throw Error('Could not load configuration');const config=await response.json();$('wifiSsid').value=config.ssid;$('maxGate').value=config.maxGate;$('timeout').value=config.timeout;updateMaxRange();$('gateRows').innerHTML=config.moving.map((value,gate)=>`<tr><td>${gate} <span class="config-note">${((gate+1)*.75).toFixed(2)} m</span></td><td><input type="number" min="0" max="100" name="moving${gate}" value="${value}" aria-label="Gate ${gate} moving sensitivity"></td><td><input type="number" min="0" max="100" name="stationary${gate}" value="${config.stationary[gate]}" aria-label="Gate ${gate} stationary sensitivity"></td></tr>`).join('')}catch(error){showConfigMessage(error.message)}}
async function postForm(url,form){const response=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(new FormData(form))});const result=await response.json();if(!response.ok)throw Error(result.error||'Configuration was not applied');return result}
$('wifiForm').addEventListener('submit',async event=>{event.preventDefault();try{await postForm('/api/wifi',$('wifiForm'));$('wifiPassword').value='';showConfigMessage('Wi-Fi saved. Station connection is starting.')}catch(error){showConfigMessage(error.message)}});
$('clearWifi').addEventListener('click',async()=>{try{const response=await fetch('/api/wifi/clear',{method:'POST'});if(!response.ok)throw Error('Could not clear Wi-Fi');$('wifiSsid').value='';$('wifiPassword').value='';showConfigMessage('Saved Wi-Fi credentials cleared. Setup access point remains active.')}catch(error){showConfigMessage(error.message)}});
$('maxGate').addEventListener('input',updateMaxRange);
$('sensorForm').addEventListener('submit',async event=>{event.preventDefault();try{await postForm('/api/sensor',$('sensorForm'));showConfigMessage('Sensor settings saved and acknowledged by the LD2410C.')}catch(error){showConfigMessage(error.message)}});
function addEvents(items){const body=$('transactions');if(!seen)body.innerHTML='';for(const e of items){lastEvent=Math.max(lastEvent,e.seq);seen++;let tr=document.createElement('tr');let kind=e.invalid?'Invalid target report':e.target?'Target report':'UART frame';tr.innerHTML=`<td>${(e.ms/1000).toFixed(2)} s</td><td>${kind}${e.footer?'':' · bad footer'}</td><td>${e.length} B</td><td class="hex">${esc(e.hex)}</td>`;body.prepend(tr)}while(body.children.length>40)body.lastElementChild.remove();$('streamCount').textContent=`${seen} captured`}
async function poll(){try{let [sr,er]=await Promise.all([fetch('/api/status',{cache:'no-store'}),fetch('/api/stream?since='+lastEvent,{cache:'no-store'})]);if(!sr.ok||!er.ok)throw Error('HTTP');update(await sr.json());addEvents(await er.json())}catch(e){$('connection').classList.remove('on');$('connectText').textContent='Reconnecting'}setTimeout(poll,350)}loadConfig();poll();
</script></body></html>
)HTML";

void saveEvent(uint16_t payloadLength, bool validFooter, bool targetReport, bool invalidTargetReport) {
  const uint32_t sequence = ++eventSequence;
  SensorEvent &event = events[(sequence - 1) % EVENT_COUNT];
  event.sequence = sequence;
  event.atMs = millis();
  event.payloadLength = payloadLength;
  event.validFooter = validFooter;
  event.targetReport = targetReport;
  event.invalidTargetReport = invalidTargetReport;
  size_t out = 0;
  for (size_t i = 0; i < expectedFrameLength && out + 2 < sizeof(event.raw); ++i) {
    snprintf(event.raw + out, sizeof(event.raw) - out, "%02X", frameBuffer[i]);
    out += 2;
    if (i + 1 < expectedFrameLength) event.raw[out++] = ' ';
  }
  event.raw[out] = '\0';
  ++frameCount;
  (void)targetReport;
}

void processFrame() {
  const size_t footerAt = expectedFrameLength - 4;
  const bool validFooter = frameBuffer[footerAt] == FRAME_FOOTER[0] &&
                           frameBuffer[footerAt + 1] == FRAME_FOOTER[1] &&
                           frameBuffer[footerAt + 2] == FRAME_FOOTER[2] &&
                           frameBuffer[footerAt + 3] == FRAME_FOOTER[3];
  const uint8_t *payload = frameBuffer + 6;
  const bool targetCandidate = validFooter && expectedPayloadLength == 13 && payload[0] == 0x02;
  bool targetReport = false;
  bool invalidTargetReport = false;
  if (targetCandidate) {
    // LD2410C report data is [type, 0xAA, 9-byte target data, 0x55, 0x00].
    // The 0xAA marker precedes the target state, so target fields start at payload[2].
    const uint8_t reportedState = payload[2];
    const uint16_t reportedMovingDistance = static_cast<uint16_t>(payload[3] | (payload[4] << 8));
    const uint16_t reportedStationaryDistance = static_cast<uint16_t>(payload[6] | (payload[7] << 8));
    const uint16_t reportedDetectionDistance = static_cast<uint16_t>(payload[9] | (payload[10] << 8));
    const bool reportMarkersValid = payload[1] == 0xAA && payload[11] == 0x55 && payload[12] == 0x00;
    const bool movingInRange = (reportedState & 0x01) == 0 || reportedMovingDistance <= 1000;
    const bool stationaryInRange = (reportedState & 0x02) == 0 || reportedStationaryDistance <= 1000;
    const bool detectionInRange = reportedState == 0 || reportedDetectionDistance <= 1000;
    targetReport = reportMarkersValid && reportedState <= 3 && movingInRange && stationaryInRange && detectionInRange;
    invalidTargetReport = !targetReport;
  }
  if (targetReport) {
    targetState = payload[2];
    movingDistance = static_cast<uint16_t>(payload[3] | (payload[4] << 8));
    movingEnergy = payload[5];
    stationaryDistance = static_cast<uint16_t>(payload[6] | (payload[7] << 8));
    stationaryEnergy = payload[8];
    detectionDistance = static_cast<uint16_t>(payload[9] | (payload[10] << 8));
    lastReportAt = millis();
    hasReport = true;
  }
  saveEvent(expectedPayloadLength, validFooter, targetReport, invalidTargetReport);
}

void resetParser() {
  frameLength = 0;
  expectedFrameLength = 0;
  expectedPayloadLength = 0;
}

void feedSensorByte(uint8_t value) {
  ++serialByteCount;
  if (frameLength == 0) {
    if (value == FRAME_HEADER[headerMatch]) {
      if (headerMatch == 0) frameBuffer[0] = value;
      else frameBuffer[headerMatch] = value;
      ++headerMatch;
      if (headerMatch == sizeof(FRAME_HEADER)) {
        frameLength = sizeof(FRAME_HEADER);
        headerMatch = 0;
      }
    } else {
      headerMatch = value == FRAME_HEADER[0] ? 1 : 0;
      if (headerMatch) frameBuffer[0] = value;
    }
    return;
  }
  if (frameLength >= sizeof(frameBuffer)) {
    resetParser();
    headerMatch = 0;
    return;
  }
  frameBuffer[frameLength++] = value;
  if (frameLength == 6) {
    expectedPayloadLength = static_cast<uint16_t>(frameBuffer[4] | (frameBuffer[5] << 8));
    if (expectedPayloadLength == 0 || expectedPayloadLength > MAX_PAYLOAD) {
      resetParser();
      return;
    }
    expectedFrameLength = expectedPayloadLength + 10;
  }
  if (expectedFrameLength && frameLength == expectedFrameLength) {
    processFrame();
    resetParser();
  }
}

String jsonStatus() {
  const uint32_t now = millis();
  const String ipAddress = WiFi.status() == WL_CONNECTED
                               ? WiFi.localIP().toString()
                               : WiFi.softAPIP().toString();
  String json;
  json.reserve(420);
  json = "{\"ip\":\"" + ipAddress + "\",\"out\":";
  json += digitalRead(SENSOR_OUT_PIN) == HIGH ? "true" : "false";
  json += ",\"frames\":" + String(frameCount);
  json += ",\"bytes\":" + String(serialByteCount);
  json += ",\"hasReport\":" + String(hasReport ? "true" : "false");
  json += ",\"reportAgeMs\":" + String(hasReport ? now - lastReportAt : 0);
  json += ",\"targetState\":" + String(targetState);
  json += ",\"movingCm\":" + String(movingDistance);
  json += ",\"movingEnergy\":" + String(movingEnergy);
  json += ",\"stationaryCm\":" + String(stationaryDistance);
  json += ",\"stationaryEnergy\":" + String(stationaryEnergy);
  json += ",\"detectionCm\":" + String(detectionDistance) + "}";
  return json;
}

String jsonEscape(const String &value) {
  String escaped;
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (c == '"' || c == '\\') escaped += '\\';
    if (static_cast<uint8_t>(c) >= 0x20) escaped += c;
  }
  return escaped;
}

String jsonConfig() {
  String json = "{\"ssid\":\"" + jsonEscape(stationSsid) + "\",\"maxGate\":" + String(maxGate);
  json += ",\"timeout\":" + String(noTargetTimeout) + ",\"moving\":[";
  for (uint8_t gate = 0; gate < 9; ++gate) {
    if (gate) json += ',';
    json += String(movingSensitivity[gate]);
  }
  json += "],\"stationary\":[";
  for (uint8_t gate = 0; gate < 9; ++gate) {
    if (gate) json += ',';
    json += String(stationarySensitivity[gate]);
  }
  return json + "]}";
}

bool waitForCommandAck(uint8_t command) {
  uint8_t response[64] = {};
  size_t length = 0;
  size_t expectedLength = 0;
  const uint32_t startedAt = millis();
  while (millis() - startedAt < 300) {
    while (SensorSerial.available()) {
      const uint8_t value = static_cast<uint8_t>(SensorSerial.read());
      feedSensorByte(value);
      if (length == 0) {
        if (value == 0xFD) response[length++] = value;
        continue;
      }
      const uint8_t header[] = {0xFD, 0xFC, 0xFB, 0xFA};
      if (length < sizeof(header)) {
        if (value == header[length]) response[length++] = value;
        else {
          length = value == header[0] ? 1 : 0;
          if (length) response[0] = value;
        }
        continue;
      }
      response[length++] = value;
      if (length == 6) {
        const uint16_t bodyLength = static_cast<uint16_t>(response[4] | (response[5] << 8));
        if (bodyLength < 4 || bodyLength + 10 > sizeof(response)) length = 0;
        else expectedLength = bodyLength + 10;
      }
      if (expectedLength && length == expectedLength) {
        const bool matchingAck = response[6] == command;
        const bool success = response[8] == 0 && response[9] == 0;
        length = 0;
        expectedLength = 0;
        if (matchingAck) return success;
      }
    }
    yield();
  }
  return false;
}

bool sendSensorCommand(uint8_t command, const uint8_t *payload, size_t payloadLength) {
  uint8_t frame[40] = {0xFD, 0xFC, 0xFB, 0xFA};
  const uint16_t commandLength = static_cast<uint16_t>(payloadLength + 2);
  frame[4] = commandLength & 0xFF;
  frame[5] = commandLength >> 8;
  frame[6] = command;
  frame[7] = 0;
  if (payloadLength) memcpy(frame + 8, payload, payloadLength);
  frame[8 + payloadLength] = 0x04;
  frame[9 + payloadLength] = 0x03;
  frame[10 + payloadLength] = 0x02;
  frame[11 + payloadLength] = 0x01;
  SensorSerial.write(frame, payloadLength + 12);
  SensorSerial.flush();
  return waitForCommandAck(command);
}

void putWord(uint8_t *destination, uint16_t value) {
  destination[0] = value & 0xFF;
  destination[1] = value >> 8;
}

bool applySensorConfiguration() {
  const uint8_t enterPayload[] = {0x01, 0x00};
  if (!sendSensorCommand(0xFF, enterPayload, sizeof(enterPayload))) return false;

  uint8_t maxValues[18] = {};
  putWord(maxValues, 0);
  putWord(maxValues + 2, maxGate);
  putWord(maxValues + 6, 1);
  putWord(maxValues + 8, maxGate);
  putWord(maxValues + 12, 2);
  putWord(maxValues + 14, noTargetTimeout);
  bool success = sendSensorCommand(0x60, maxValues, sizeof(maxValues));

  for (uint8_t gate = 0; gate < 9 && success; ++gate) {
    uint8_t sensitivities[18] = {};
    putWord(sensitivities, 0);
    putWord(sensitivities + 2, gate);
    putWord(sensitivities + 6, 1);
    putWord(sensitivities + 8, movingSensitivity[gate]);
    putWord(sensitivities + 12, 2);
    putWord(sensitivities + 14, stationarySensitivity[gate]);
    success = sendSensorCommand(0x64, sensitivities, sizeof(sensitivities));
  }

  const bool leftConfiguration = sendSensorCommand(0xFE, nullptr, 0);
  return success && leftConfiguration;
}

void handleWifiSave() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  if (ssid.length() > 32 || password.length() > 63) {
    server.send(400, "application/json", "{\"error\":\"SSID or password is too long\"}");
    return;
  }
  if (!password.length() && ssid == stationSsid) password = stationPassword;
  stationSsid = ssid;
  stationPassword = password;
  stationConfigured = stationSsid.length() > 0;
  preferences.putBool("wifiSet", true);
  preferences.putString("ssid", stationSsid);
  preferences.putString("password", stationPassword);
  server.send(200, "application/json", "{\"saved\":true}");
  if (stationConfigured) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(stationSsid.c_str(), stationPassword.c_str());
  }
}

void handleWifiClear() {
  stationSsid = "";
  stationPassword = "";
  stationConfigured = false;
  preferences.putBool("wifiSet", true);
  preferences.remove("ssid");
  preferences.remove("password");
  server.send(200, "application/json", "{\"cleared\":true}");
  WiFi.disconnect(false, true);
  WiFi.mode(WIFI_AP);
}

bool parseConfigValue(const String &value, long minimum, long maximum, long &result) {
  if (!value.length()) return false;
  char *end = nullptr;
  result = strtol(value.c_str(), &end, 10);
  return end != value.c_str() && *end == '\0' && result >= minimum && result <= maximum;
}

void handleSensorSave() {
  long parsedMaxGate;
  long parsedTimeout;
  if (!parseConfigValue(server.arg("maxGate"), 0, 8, parsedMaxGate) ||
      !parseConfigValue(server.arg("timeout"), 0, 65535, parsedTimeout)) {
    server.send(400, "application/json", "{\"error\":\"Invalid range or timeout\"}");
    return;
  }
  uint8_t nextMoving[9];
  uint8_t nextStationary[9];
  for (uint8_t gate = 0; gate < 9; ++gate) {
    long moving;
    long stationary;
    if (!parseConfigValue(server.arg("moving" + String(gate)), 0, 100, moving) ||
        !parseConfigValue(server.arg("stationary" + String(gate)), 0, 100, stationary)) {
      server.send(400, "application/json", "{\"error\":\"Gate sensitivity must be 0-100\"}");
      return;
    }
    nextMoving[gate] = moving;
    nextStationary[gate] = stationary;
  }

  maxGate = parsedMaxGate;
  noTargetTimeout = parsedTimeout;
  memcpy(movingSensitivity, nextMoving, sizeof(movingSensitivity));
  memcpy(stationarySensitivity, nextStationary, sizeof(stationarySensitivity));
  preferences.putUChar("maxGate", maxGate);
  preferences.putUShort("timeout", noTargetTimeout);
  preferences.putBool("sensorSet", true);
  sensorConfigStored = true;
  for (uint8_t gate = 0; gate < 9; ++gate) {
    preferences.putUChar(("moving" + String(gate)).c_str(), movingSensitivity[gate]);
    preferences.putUChar(("stationary" + String(gate)).c_str(), stationarySensitivity[gate]);
  }
  const bool applied = applySensorConfiguration();
  server.send(applied ? 200 : 502, "application/json",
              applied ? "{\"saved\":true,\"applied\":true}" : "{\"saved\":true,\"applied\":false,\"error\":\"Sensor did not acknowledge configuration\"}");
}

void handleStream() {
  uint32_t since = server.hasArg("since") ? static_cast<uint32_t>(server.arg("since").toInt()) : 0;
  String json = "[";
  bool first = true;
  const uint32_t oldest = eventSequence > EVENT_COUNT ? eventSequence - EVENT_COUNT + 1 : 1;
  for (uint32_t seq = max(since + 1, oldest); seq <= eventSequence; ++seq) {
    const SensorEvent &event = events[(seq - 1) % EVENT_COUNT];
    if (event.sequence != seq) continue;
    if (!first) json += ',';
    first = false;
    json += "{\"seq\":" + String(event.sequence) + ",\"ms\":" + String(event.atMs);
    json += ",\"length\":" + String(event.payloadLength) + ",\"footer\":" + String(event.validFooter ? "true" : "false");
    json += ",\"target\":" + String(event.targetReport ? "true" : "false");
    json += ",\"invalid\":" + String(event.invalidTargetReport ? "true" : "false");
    json += ",\"hex\":\"" + String(event.raw) + "\"}";
  }
  json += ']';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

const char *wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "idle";
    case WL_NO_SSID_AVAIL: return "network not found (check SSID / 2.4 GHz)";
    case WL_SCAN_COMPLETED: return "scan completed";
    case WL_CONNECTED: return "connected";
    case WL_CONNECT_FAILED: return "connection failed (check password / security)";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED: return "disconnected / connecting";
    default: return "unknown";
  }
}

void reportWifiStatus() {
  const uint32_t now = millis();
  const wl_status_t status = WiFi.status();
  if (static_cast<int>(status) != previousWifiStatus) {
    previousWifiStatus = status;
    Serial.printf("Wi-Fi station: %s\n", wifiStatusName(status));
    if (status == WL_CONNECTED) {
      Serial.printf("Wi-Fi station IP: http://%s/\n", WiFi.localIP().toString().c_str());
    }
  }
  if (stationConfigured && status != WL_CONNECTED && now - lastWifiRetryAt >= 15000) {
    lastWifiRetryAt = now;
    Serial.println("Wi-Fi station still offline; retrying connection...");
    WiFi.reconnect();
  }
  lastWifiStatusAt = now;
}

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_OUT_PIN, INPUT);
  SensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  preferences.begin("presence", false);
  if (preferences.getBool("wifiSet", false)) {
    stationSsid = preferences.getString("ssid", "");
    stationPassword = preferences.getString("password", "");
  } else {
    stationSsid = WIFI_SSID;
    stationPassword = WIFI_PASSWORD;
  }
  maxGate = preferences.getUChar("maxGate", 8);
  noTargetTimeout = preferences.getUShort("timeout", 5);
  sensorConfigStored = preferences.getBool("sensorSet", false);
  for (uint8_t gate = 0; gate < 9; ++gate) {
    movingSensitivity[gate] = preferences.getUChar(("moving" + String(gate)).c_str(), 50);
    stationarySensitivity[gate] = preferences.getUChar(("stationary" + String(gate)).c_str(), 50);
  }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  stationConfigured = stationSsid.length() > 0;
  WiFi.setAutoReconnect(true);
  if (stationConfigured) {
    Serial.println("Starting Wi-Fi station connection (SSID configured; password is not logged).");
    WiFi.begin(stationSsid.c_str(), stationPassword.c_str());
  } else {
    Serial.println("No station SSID configured; running setup access point only.");
  }
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  server.on("/api/status", HTTP_GET, []() { server.send(200, "application/json", jsonStatus()); });
  server.on("/api/config", HTTP_GET, []() { server.send(200, "application/json", jsonConfig()); });
  server.on("/api/wifi", HTTP_POST, handleWifiSave);
  server.on("/api/wifi/clear", HTTP_POST, handleWifiClear);
  server.on("/api/sensor", HTTP_POST, handleSensorSave);
  server.on("/api/stream", HTTP_GET, handleStream);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.printf("Setup access point: %s\n", AP_SSID);
  Serial.printf("Setup dashboard: http://%s/\n", WiFi.softAPIP().toString().c_str());
  reportWifiStatus();
  if (sensorConfigStored) {
    Serial.println(applySensorConfiguration() ? "Saved LD2410C settings restored." : "Could not restore saved LD2410C settings.");
  }
}

void loop() {
  while (SensorSerial.available()) feedSensorByte(static_cast<uint8_t>(SensorSerial.read()));
  server.handleClient();
  if (millis() - lastWifiStatusAt >= 1000) reportWifiStatus();
  yield();
}
