/*
  Bridge_A_ArtNet_Fixed.ino
  Bridge A: DMX <-> Art-Net Bridge (ESP32-WROOM)

  - ESP32 core: 2.0.17
  - esp_dmx:    4.1.0
  - Safe EN pins (no boot loops)
  - DMX → ArtNet by default
  - Universes: DMX1 = 0, DMX2 = 1
  - Web UI for config
  - DMX length clamped to 2–512, even (fixes Artnetominator error)
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArtnetWifi.h>
#include <WiFiUdp.h>
#include <esp_dmx.h>

// ---------- Pin mapping ----------
#define DMX1_PORT 0
#define DMX2_PORT 1

#define DMX1_TX 17
#define DMX1_RX 16
#define DMX1_EN 21   // SAFE EN

#define DMX2_TX 25
#define DMX2_RX 26
#define DMX2_EN 22   // SAFE EN

#define TERM1_PIN 32
#define TERM2_PIN 33

// ---------- Wi-Fi AP defaults ----------
const char* AP_NAME = "DMX-Bridge-A";
const char* AP_PASS = "dmx12345";

// ---------- Globals ----------
ArtnetWifi artnet;
AsyncWebServer server(80);
Preferences prefs;
WiFiUDP artnetUdp;

uint8_t dmx1Buf[513];
uint8_t dmx2Buf[513];

// Config (NVS-backed)
String cfg_ssid = "";
String cfg_pass = "";
String cfg_broadcast = "255.255.255.255";

// Port 1
uint16_t cfg_uni1 = 0;
bool cfg_enabled1 = true;
bool cfg_dir_in1 = false;      // false = DMX → ArtNet
unsigned int cfg_rate1 = 100;
bool cfg_term1 = false;
bool cfg_inv1 = false;

// Port 2
uint16_t cfg_uni2 = 1;
bool cfg_enabled2 = true;
bool cfg_dir_in2 = false;      // false = DMX → ArtNet
unsigned int cfg_rate2 = 100;
bool cfg_term2 = false;
bool cfg_inv2 = false;

// Status
unsigned long lastArtnetRx1 = 0;
unsigned long lastArtnetRx2 = 0;
unsigned long lastDmxSend1  = 0;
unsigned long lastDmxSend2  = 0;
unsigned long lastSendTime1 = 0;
unsigned long lastSendTime2 = 0;

// ---------- HTML ----------
const char PAGE[] PROGMEM = R"rawliteral(
<html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DMX Bridge A</title>
</head><body>
<h2>DMX Bridge A Configuration</h2>

<form method="post" action="/save">
<h3>Network</h3>
SSID: <input name="ssid" value="%SSID%"><br>
Password: <input name="pass" type="password" value="%PASS%"><br>
Broadcast IP: <input name="bcast" value="%BCAST%"><br>

<h3>DMX1</h3>
Enabled:
<select name="ena1"><option value="1" %ENA1Y%>Yes</option><option value="0" %ENA1N%>No</option></select><br>
Universe: <input name="uni1" value="%UNI1%"><br>
Direction:
<select name="dir1">
<option value="in" %DIR1IN%>ArtNet->DMX</option>
<option value="out" %DIR1OUT%>DMX->ArtNet</option>
</select><br>
Rate(ms): <input name="rate1" value="%RATE1%"><br>
Termination:
<select name="term1"><option value="1" %TERM1Y%>On</option><option value="0" %TERM1N%>Off</option></select><br>
Invert:
<select name="inv1"><option value="0" %INV1N%>No</option><option value="1" %INV1Y%>Yes</option></select><br>

<h3>DMX2</h3>
Enabled:
<select name="ena2"><option value="1" %ENA2Y%>Yes</option><option value="0" %ENA2N%>No</option></select><br>
Universe: <input name="uni2" value="%UNI2%"><br>
Direction:
<select name="dir2">
<option value="in" %DIR2IN%>ArtNet->DMX</option>
<option value="out" %DIR2OUT%>DMX->ArtNet</option>
</select><br>
Rate(ms): <input name="rate2" value="%RATE2%"><br>
Termination:
<select name="term2"><option value="1" %TERM2Y%>On</option><option value="0" %TERM2N%>Off</option></select><br>
Invert:
<select name="inv2"><option value="0" %INV2N%>No</option><option value="1" %INV2Y%>Yes</option></select><br>

<input type="submit" value="Save & Reboot">
</form>

</body></html>
)rawliteral";

String buildPage(){
  String s = FPSTR(PAGE);

  s.replace("%SSID%", cfg_ssid);
  s.replace("%PASS%", cfg_pass);
  s.replace("%BCAST%", cfg_broadcast);

  s.replace("%UNI1%", String(cfg_uni1));
  s.replace("%UNI2%", String(cfg_uni2));

  s.replace("%RATE1%", String(cfg_rate1));
  s.replace("%RATE2%", String(cfg_rate2));

  s.replace("%ENA1Y%", cfg_enabled1 ? "selected" : "");
  s.replace("%ENA1N%", cfg_enabled1 ? "" : "selected");
  s.replace("%ENA2Y%", cfg_enabled2 ? "selected" : "");
  s.replace("%ENA2N%", cfg_enabled2 ? "" : "selected");

  s.replace("%DIR1IN%",  cfg_dir_in1 ? "selected" : "");
  s.replace("%DIR1OUT%", cfg_dir_in1 ? "" : "selected");
  s.replace("%DIR2IN%",  cfg_dir_in2 ? "selected" : "");
  s.replace("%DIR2OUT%", cfg_dir_in2 ? "" : "selected");

  s.replace("%TERM1Y%", cfg_term1 ? "selected" : "");
  s.replace("%TERM1N%", cfg_term1 ? "" : "selected");
  s.replace("%TERM2Y%", cfg_term2 ? "selected" : "");
  s.replace("%TERM2N%", cfg_term2 ? "" : "selected");

  s.replace("%INV1Y%", cfg_inv1 ? "selected" : "");
  s.replace("%INV1N%", cfg_inv1 ? "" : "selected");
  s.replace("%INV2Y%", cfg_inv2 ? "selected" : "");
  s.replace("%INV2N%", cfg_inv2 ? "" : "selected");

  return s;
}

// ---------- NVS ----------
void loadCfg(){
  prefs.begin("bridgeA", true);
  cfg_ssid      = prefs.getString("ssid",  cfg_ssid);
  cfg_pass      = prefs.getString("pass",  cfg_pass);
  cfg_broadcast = prefs.getString("bcast", cfg_broadcast);

  cfg_uni1      = prefs.getUShort("uni1",  cfg_uni1);
  cfg_enabled1  = prefs.getBool  ("ena1",  cfg_enabled1);
  cfg_dir_in1   = prefs.getBool  ("dir1",  cfg_dir_in1);
  cfg_rate1     = prefs.getUInt  ("rate1", cfg_rate1);
  cfg_term1     = prefs.getBool  ("term1", cfg_term1);
  cfg_inv1      = prefs.getBool  ("inv1",  cfg_inv1);

  cfg_uni2      = prefs.getUShort("uni2",  cfg_uni2);
  cfg_enabled2  = prefs.getBool  ("ena2",  cfg_enabled2);
  cfg_dir_in2   = prefs.getBool  ("dir2",  cfg_dir_in2);
  cfg_rate2     = prefs.getUInt  ("rate2", cfg_rate2);
  cfg_term2     = prefs.getBool  ("term2", cfg_term2);
  cfg_inv2      = prefs.getBool  ("inv2",  cfg_inv2);
  prefs.end();
}

void saveCfg(){
  prefs.begin("bridgeA", false);
  prefs.putString("ssid",  cfg_ssid);
  prefs.putString("pass",  cfg_pass);
  prefs.putString("bcast", cfg_broadcast);

  prefs.putUShort("uni1",  cfg_uni1);
  prefs.putBool  ("ena1",  cfg_enabled1);
  prefs.putBool  ("dir1",  cfg_dir_in1);
  prefs.putUInt  ("rate1", cfg_rate1);
  prefs.putBool  ("term1", cfg_term1);
  prefs.putBool  ("inv1",  cfg_inv1);

  prefs.putUShort("uni2",  cfg_uni2);
  prefs.putBool  ("ena2",  cfg_enabled2);
  prefs.putBool  ("dir2",  cfg_dir_in2);
  prefs.putUInt  ("rate2", cfg_rate2);
  prefs.putBool  ("term2", cfg_term2);
  prefs.putBool  ("inv2",  cfg_inv2);
  prefs.end();
}

// ---------- DMX HW ----------
void applyDMX(){
  pinMode(TERM1_PIN, OUTPUT);
  pinMode(TERM2_PIN, OUTPUT);

  digitalWrite(TERM1_PIN, cfg_term1 ? HIGH : LOW);
  digitalWrite(TERM2_PIN, cfg_term2 ? HIGH : LOW);

  // polarity inversion = swap TX/RX in dmx_set_pin()
  if(cfg_inv1){
    dmx_set_pin(DMX1_PORT, DMX1_RX, DMX1_TX, DMX1_EN);
  } else {
    dmx_set_pin(DMX1_PORT, DMX1_TX, DMX1_RX, DMX1_EN);
  }

  if(cfg_inv2){
    dmx_set_pin(DMX2_PORT, DMX2_RX, DMX2_TX, DMX2_EN);
  } else {
    dmx_set_pin(DMX2_PORT, DMX2_TX, DMX2_RX, DMX2_EN);
  }
}

void setupDMX(){
  dmx_config_t config = DMX_CONFIG_DEFAULT;

  static dmx_personality_t p1 = {512, "DMX Port 1"};
  static dmx_personality_t p2 = {512, "DMX Port 2"};

  dmx_driver_install(DMX1_PORT, &config, &p1, 1);
  dmx_driver_install(DMX2_PORT, &config, &p2, 1);

  applyDMX();
}

// ---------- ArtNet RX → DMX ----------
void onArtDmx(uint16_t universe, uint16_t length, uint8_t seq, uint8_t* data){
  unsigned long now = millis();

  if(cfg_enabled1 && cfg_dir_in1 && universe == cfg_uni1){
    dmx_write(DMX1_PORT, data, length);
    dmx_send(DMX1_PORT);
    lastArtnetRx1 = now;
  }

  if(cfg_enabled2 && cfg_dir_in2 && universe == cfg_uni2){
    dmx_write(DMX2_PORT, data, length);
    dmx_send(DMX2_PORT);
    lastArtnetRx2 = now;
  }
}

// ---------- Safe ArtNet DMX Sender ----------
void sendArt(uint16_t uni, uint8_t* data, uint16_t len){
  // Art-Net spec: length must be 2–512, even
  if(len < 2) return;
  if(len > 512) len = 512;
  if(len & 1) len--;      // enforce even length
  if(len < 2) return;

  uint8_t pkt[18 + 512];
  int p = 0;

  memcpy(pkt + p, "Art-Net", 7); p += 7;
  pkt[p++] = 0x00;

  // OpCode = OpDmx (0x5000 LE)
  pkt[p++] = 0x00;
  pkt[p++] = 0x50;

  // ProtVer
  pkt[p++] = 0x00;
  pkt[p++] = 0x0E;

  // Sequence, Physical
  pkt[p++] = 0x00;
  pkt[p++] = 0x00;

  // Universe
  pkt[p++] = uni & 0xFF;
  pkt[p++] = (uni >> 8) & 0x7F;

  // Length
  pkt[p++] = (len >> 8) & 0xFF;
  pkt[p++] = len & 0xFF;

  memcpy(pkt + p, data, len);
  p += len;

  IPAddress bc;
  if(!cfg_broadcast.isEmpty() && bc.fromString(cfg_broadcast.c_str())){
    artnetUdp.beginPacket(bc, 6454);
  } else {
    artnetUdp.beginPacket(IPAddress(255,255,255,255), 6454);
  }

  artnetUdp.write(pkt, p);
  artnetUdp.endPacket();
}

// ---------- DMX → ArtNet periodic ----------
void loopDMX(){
  unsigned long now = millis();

  // DMX1
  if(cfg_enabled1 && !cfg_dir_in1 && now - lastSendTime1 >= cfg_rate1){
    int len = dmx_read(DMX1_PORT, dmx1Buf, sizeof(dmx1Buf));
    if(len > 0){
      // clamp length before sending
      if(len > 512) len = 512;
      sendArt(cfg_uni1, dmx1Buf, (uint16_t)len);
      lastDmxSend1  = now;
      lastSendTime1 = now;
    }
  }

  // DMX2
  if(cfg_enabled2 && !cfg_dir_in2 && now - lastSendTime2 >= cfg_rate2){
    int len = dmx_read(DMX2_PORT, dmx2Buf, sizeof(dmx2Buf));
    if(len > 0){
      if(len > 512) len = 512;
      sendArt(cfg_uni2, dmx2Buf, (uint16_t)len);
      lastDmxSend2  = now;
      lastSendTime2 = now;
    }
  }
}

// ---------- Web ----------
void setupWeb(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send(200, "text/html", buildPage());
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest* r){
    if(r->hasParam("ssid", true))  cfg_ssid      = r->getParam("ssid", true)->value();
    if(r->hasParam("pass", true))  cfg_pass      = r->getParam("pass", true)->value();
    if(r->hasParam("bcast", true)) cfg_broadcast = r->getParam("bcast", true)->value();

    if(r->hasParam("uni1", true)) cfg_uni1 = r->getParam("uni1", true)->value().toInt();
    if(r->hasParam("uni2", true)) cfg_uni2 = r->getParam("uni2", true)->value().toInt();

    if(r->hasParam("ena1", true)) cfg_enabled1 = (r->getParam("ena1", true)->value() == "1");
    if(r->hasParam("ena2", true)) cfg_enabled2 = (r->getParam("ena2", true)->value() == "1");

    if(r->hasParam("dir1", true)) cfg_dir_in1 = (r->getParam("dir1", true)->value() == "in");
    if(r->hasParam("dir2", true)) cfg_dir_in2 = (r->getParam("dir2", true)->value() == "in");

    if(r->hasParam("rate1", true)) cfg_rate1 = r->getParam("rate1", true)->value().toInt();
    if(r->hasParam("rate2", true)) cfg_rate2 = r->getParam("rate2", true)->value().toInt();

    if(r->hasParam("term1", true)) cfg_term1 = (r->getParam("term1", true)->value() == "1");
    if(r->hasParam("term2", true)) cfg_term2 = (r->getParam("term2", true)->value() == "1");

    if(r->hasParam("inv1", true)) cfg_inv1 = (r->getParam("inv1", true)->value() == "1");
    if(r->hasParam("inv2", true)) cfg_inv2 = (r->getParam("inv2", true)->value() == "1");

    saveCfg();
    applyDMX();

    r->send(200, "text/plain", "Saved. Rebooting...");
    delay(400);
    ESP.restart();
  });

  server.begin();
}

// ---------- WiFi ----------
void setupWiFi(){
  if(cfg_ssid.length()){
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg_ssid.c_str(), cfg_pass.c_str());

    unsigned long t = millis();
    while(WiFi.status() != WL_CONNECTED && millis() - t < 6000){
      delay(200);
    }

    if(WiFi.status() == WL_CONNECTED){
      return;
    }
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_NAME, AP_PASS);
}

// ---------- Setup / Loop ----------
void setup(){
  Serial.begin(115200);
  delay(100);

  loadCfg();
  setupWiFi();
  setupWeb();
  setupDMX();

  artnet.begin();
  artnet.setArtDmxCallback(onArtDmx);
  artnetUdp.begin(6454);

  Serial.println("Bridge A ready.");
}

void loop(){
  artnet.read();
  loopDMX();
}