#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>
#include <math.h>

// UART2 defaults for a classic ESP32 DevKit. Change these for a different board.
static constexpr int SENSOR_RX_PIN = 16; // ESP32 RX2  <- LD2410C TX
static constexpr int SENSOR_TX_PIN = 17; // ESP32 TX2  -> LD2410C RX
static constexpr int SENSOR_OUT_PIN = 2; // LD2410C OUT -> ESP32 GPIO2 ("D2" on some boards)
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
HardwareSerial SensorSerial(2);

struct SensorEvent {
  uint32_t sequence = 0;
  uint32_t atMs = 0;
  uint16_t payloadLength = 0;
  bool validFooter = false;
  bool targetReport = false;
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
@media(max-width:760px){.hero{grid-template-columns:1fr}.stage{min-height:300px}.data-grid{grid-template-columns:repeat(2,1fr)}.wrap{padding:20px 14px 32px}.top{align-items:flex-start}.title{font-size:19px}.pill{white-space:nowrap}}
</style></head><body><main class="wrap"><header class="top"><div class="brand"><div class="mark">◉</div><div><div class="eyebrow">Sensor console</div><div class="title">LD2410C Presence</div></div></div><div class="pill"><i id="connection" class="dot"></i><span id="connectText">Connecting</span></div></header>
<section class="hero"><article class="card stage"><div class="radar"><div class="beam"></div></div><div id="orb" class="orb none"><span>◉</span></div><div id="state" class="state">Waiting for sensor</div><div id="hint" class="hint">No valid target report received yet</div><div class="range"><div class="rangehead"><span>PROXIMITY</span><span id="proximity">—</span></div><div class="track"><div id="rangeFill" class="fill"></div></div></div></article>
<article class="card metrics"><h2 class="section-title">Live target readings</h2><div class="readings"><div class="reading move"><div class="label">Moving target</div><div class="value"><span id="movingDistance">—</span><span class="unit">cm</span></div></div><div class="reading"><div class="label">Moving energy</div><div class="value"><span id="movingEnergy">—</span><span class="unit">/ 100</span></div></div><div class="reading still"><div class="label">Stationary target</div><div class="value"><span id="stationaryDistance">—</span><span class="unit">cm</span></div></div><div class="reading"><div class="label">Stationary energy</div><div class="value"><span id="stationaryEnergy">—</span><span class="unit">/ 100</span></div></div><div class="reading detect"><div class="label">Detection distance</div><div class="value"><span id="detectionDistance">—</span><span class="unit">cm</span></div></div><div class="reading"><div class="label">Target-state code</div><div class="value"><span id="targetCode">—</span></div></div></div></article></section>
<section class="card gridcard"><h2 class="section-title">Sensor &amp; link details</h2><div class="data-grid"><div class="datum"><div class="k">LD2410C OUT</div><div class="v" id="outPin">—</div></div><div class="datum"><div class="k">Last report</div><div class="v" id="reportAge">—</div></div><div class="datum"><div class="k">Valid frames</div><div class="v" id="frameCount">0</div></div><div class="datum"><div class="k">UART bytes</div><div class="v" id="byteCount">0</div></div><div class="datum"><div class="k">UART settings</div><div class="v">UART2 · 256000 · 8N1</div></div><div class="datum"><div class="k">Wi-Fi address</div><div class="v" id="ipAddress">—</div></div><div class="datum"><div class="k">Sensor UART</div><div class="v" id="uartPins">RX 16 / TX 17</div></div><div class="datum"><div class="k">Report type</div><div class="v">Standard target report</div></div></div></section>
<section class="card streamcard"><div class="streamhead"><h2>Incoming UART transactions <span class="badge" id="streamCount">0 captured</span></h2><span class="streammeta">Newest first · raw frame bytes</span></div><div class="tablewrap"><table><thead><tr><th>Time</th><th>Kind</th><th>Payload</th><th>Frame (hex)</th></tr></thead><tbody id="transactions"><tr><td colspan="4" class="empty">Listening for LD2410C frames…</td></tr></tbody></table></div></section><div class="footer">Presence is derived from UART target reports; proximity intensity is strongest at shorter measured distance.</div></main>
<script>
const $=id=>document.getElementById(id);let lastEvent=0,seen=0;
function fmtDistance(n){return n===null||n===undefined?'—':n===0?'0':n.toLocaleString()}
function update(s){$('connection').classList.toggle('on',true);$('connectText').textContent='Device online';$('ipAddress').textContent=s.ip;$('frameCount').textContent=s.frames.toLocaleString();$('byteCount').textContent=s.bytes.toLocaleString();$('outPin').textContent=s.out?'HIGH · presence':'LOW · clear';$('reportAge').textContent=s.hasReport?`${(s.reportAgeMs/1000).toFixed(1)} s ago`:'No report yet';
$('movingDistance').textContent=s.hasReport?fmtDistance(s.movingCm):'—';$('stationaryDistance').textContent=s.hasReport?fmtDistance(s.stationaryCm):'—';$('detectionDistance').textContent=s.hasReport?fmtDistance(s.detectionCm):'—';$('movingEnergy').textContent=s.hasReport?s.movingEnergy:'—';$('stationaryEnergy').textContent=s.hasReport?s.stationaryEnergy:'—';$('targetCode').textContent=s.hasReport?s.targetState:'—';
let st=s.targetState,active=s.hasReport&&s.reportAgeMs<5000,kind=active?(st===1?'moving':st===2?'stationary':st===3?'both':'none'):'none';let names={none:active?'No Presence':'No recent presence report',moving:'Moving Presence',stationary:'Non-Moving Presence',both:'Moving + Non-Moving'};$('orb').className='orb '+kind;$('state').textContent=names[kind];$('hint').textContent=!active?'Waiting for a fresh target report':kind==='none'?'Sensor reports no target':`Sensor target state ${st}${st===3?' (both target types)':''}`;
let ds=[];if(kind==='moving'||kind==='both')ds.push(s.movingCm);if(kind==='stationary'||kind==='both')ds.push(s.stationaryCm);ds=ds.filter(x=>x>0);let near=ds.length?Math.min(...ds):0;let strength=near?Math.max(8,Math.min(100,100-near/600*100)):0;$('rangeFill').style.width=strength+'%';$('proximity').textContent=near?`${near} cm · ${Math.round(strength)}%`:'—';$('orb').style.opacity=near?String(.45+strength/180):'1';$('orb').style.transform=near?`scale(${.82+strength/250})`:'scale(1)';}
function esc(v){return String(v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function addEvents(items){const body=$('transactions');if(!seen)body.innerHTML='';for(const e of items){lastEvent=Math.max(lastEvent,e.seq);seen++;let tr=document.createElement('tr');tr.innerHTML=`<td>${(e.ms/1000).toFixed(2)} s</td><td>${e.target?'Target report':'UART frame'}${e.footer?'':' · bad footer'}</td><td>${e.length} B</td><td class="hex">${esc(e.hex)}</td>`;body.prepend(tr)}while(body.children.length>40)body.lastElementChild.remove();$('streamCount').textContent=`${seen} captured`}
async function poll(){try{let [sr,er]=await Promise.all([fetch('/api/status',{cache:'no-store'}),fetch('/api/stream?since='+lastEvent,{cache:'no-store'})]);if(!sr.ok||!er.ok)throw Error('HTTP');update(await sr.json());addEvents(await er.json())}catch(e){$('connection').classList.remove('on');$('connectText').textContent='Reconnecting'}setTimeout(poll,350)}poll();
</script></body></html>
)HTML";

void saveEvent(uint16_t payloadLength, bool validFooter, bool targetReport) {
  const uint32_t sequence = ++eventSequence;
  SensorEvent &event = events[(sequence - 1) % EVENT_COUNT];
  event.sequence = sequence;
  event.atMs = millis();
  event.payloadLength = payloadLength;
  event.validFooter = validFooter;
  event.targetReport = targetReport;
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
  const bool targetReport = validFooter && expectedPayloadLength >= 10 && payload[0] == 0x02;
  if (targetReport) {
    targetState = payload[1] & 0x03;
    movingDistance = static_cast<uint16_t>(payload[2] | (payload[3] << 8));
    movingEnergy = payload[4];
    stationaryDistance = static_cast<uint16_t>(payload[5] | (payload[6] << 8));
    stationaryEnergy = payload[7];
    detectionDistance = static_cast<uint16_t>(payload[8] | (payload[9] << 8));
    lastReportAt = millis();
    hasReport = true;
  }
  saveEvent(expectedPayloadLength, validFooter, targetReport);
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
    json += ",\"hex\":\"" + String(event.raw) + "\"}";
  }
  json += ']';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_OUT_PIN, INPUT);
  SensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (strlen(WIFI_SSID) > 0) WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  server.on("/api/status", HTTP_GET, []() { server.send(200, "application/json", jsonStatus()); });
  server.on("/api/stream", HTTP_GET, handleStream);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.printf("LD2410C web UI: http://%s/\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
  while (SensorSerial.available()) feedSensorByte(static_cast<uint8_t>(SensorSerial.read()));
  server.handleClient();
  yield();
}
