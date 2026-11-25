/*
  fdc1004_v3.ino
  Two-sensor capacitance readout, FDC1004 + DFRobot Beetle ESP32-C6.
  Serial 460800 baud (newline), optional WiFi (TCP 8888 / HTTP 80).
  Output format is the same as the old two-channel firmware (v2), so the dashboards
  and loggers still work.


  Data:   {"c1_pf":..,"c2_pf":..,"c1_cal_pf":..,"c2_cal_pf":..,"st1":"ok","st2":"ok",
           "cd1":0,"cd2":0,"n":123,"sensor_ok":true,"millis":456}
          cN_pf = reading - zero (null if not measured), cN_cal_pf = quad*c^2 + gain*c + shift
  Other:  {"event":"config"...}, {"event":"msg"...}, {"event":"probe"...}, {"event":"scan"...}

  Commands (<s> = 1, 2 or all):
    get, help, status
    meas <1|2|all>
    mode <s> <diff|se>          (clears zero + cal)
    pins <s> <a> <b>            diff CINa-CINb
    pins <s> <a>                se CINa-GND
    pins default                S1 = CIN1-CIN2, S2 = CIN3-CIN4, diff
    zero [s]  (also cal, c)     do it with nothing on the pins (cables connected), or with
                                the sensor at rest if you only want changes
    unzero [s]
    autorange <s> <on|off>      se
    capdac <s> <0..31>          se, turns autorange off
    offset <s> <pF>             diff, +-15.99 pF
    gain|shift|quad <s> <v>     calibration, saved in flash
    calclear [s]
    probe <s>, scan
    rate <100|200|400>, avg <1..64>, bits <16|24>, stream <ms>, i2c <100|400>
    settle <0|1>, human <on|off>
    fdcreset, reg [hex], regw <hex> <hex>, factory
    wifissid, wifipass, wifiauto <on|off>, wificlear, wifi [on|off], wifiscan
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

// ---- user config ----
const bool USE_WIFI = false;  // radio causes ~0.5 pF spikes while connecting
const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const int I2C_SDA_PIN = 19;
const int I2C_SCL_PIN = 20;
const char *FW_VERSION = "3.0";

// ---- FDC1004 registers ----
const uint8_t FDC_ADDR = 0x50;
const uint8_t REG_FDC_CONF = 0x0C;
const uint8_t REG_OFFSET_CAL = 0x0D;  // 0x0D-0x10 -> CIN1-4
const uint8_t REG_GAIN_CAL = 0x11;  // 0x11-0x14 -> CIN1-4
const uint8_t REG_MANUFACTURER_ID = 0xFE;
const uint8_t REG_DEVICE_ID = 0xFF;
const uint8_t CHB_CAPDAC = 0b100;
const float CAPDAC_PF = 3.125f;
const float RAIL_PF = 15.99f;  // +-16 pF full scale
const float RANGE_HI_PF = 12.5f;  // se autorange re-centers above this

// slot k (0-3) = MEASk+1: result regs 2k/2k+1, conf 0x08+k, trigger bit 7-k, done bit 3-k
inline uint8_t slotMsb(uint8_t k) { return 2 * k; }
inline uint8_t slotLsb(uint8_t k) { return 2 * k + 1; }
inline uint8_t slotConf(uint8_t k) { return 0x08 + k; }
inline uint16_t slotTrig(uint8_t k) { return 1u << (7 - k); }
inline uint16_t slotDone(uint8_t k) { return 1u << (3 - k); }

const int NS = 2;
enum { MODE_DIFF = 0, MODE_SE = 1 };
enum { ST_OK = 0, ST_SAT, ST_SETTLE, ST_OVER, ST_NODATA };
const char *ST_NAME[] = { "ok", "sat", "settle", "over", "nodata" };

// ---- settings saved in flash (namespace "fdc3") ----
struct SensorCfg {
  uint8_t mode;  // MODE_DIFF or MODE_SE
  uint8_t pinA, pinB;  // 0-3, pinB only used in diff
  uint8_t capdac;  // se only
  uint8_t autorange;  // se: 1 = auto capdac
  float hwOffsetPf;  // diff: OFFSET_CAL for pinA
  float zeroPf;  // subtracted from reading (pF)
  uint8_t zeroSet;
  float a0, a1, a2;  // cal poly coefficients
  uint8_t calSet;
};

struct Persist {
  uint32_t magic;
  uint16_t version;
  uint16_t rateSps;
  uint8_t avgN, bits, measMask, settle;
  uint16_t streamMs, i2cKhz;
  uint8_t human;
  SensorCfg s[NS];
};

const uint32_t PERSIST_MAGIC = 0xFDC10043;
Persist P;

// prototypes for functions that use the structs above, otherwise the Arduino
// auto-prototypes land before the struct definitions and it doesn't compile
bool validPersist(const Persist &p);

void setDefaults() {
  memset(&P, 0, sizeof(P));
  P.magic = PERSIST_MAGIC;
  P.version = 3;
  P.rateSps = 400;
  P.avgN = 1;
  P.bits = 24;
  P.measMask = 3;
  P.settle = 0;
  P.streamMs = 10;
  P.i2cKhz = 400;
  P.human = 0;
  for (int i = 0; i < NS; i++) {
    SensorCfg &c = P.s[i];
    c.mode = MODE_DIFF;
    c.pinA = 2 * i;
    c.pinB = 2 * i + 1;
    c.capdac = 0;
    c.autorange = 1;
    c.hwOffsetPf = 0;
    c.zeroPf = 0;
    c.zeroSet = 0;
    c.a0 = 0; c.a1 = 1; c.a2 = 0;
    c.calSet = 0;
  }
}

// forward decls (Arduino IDE adds these, other toolchains don't)
bool sharedOk();
float estimatedRate();
String handleCommand(String line);
void emitResponse(const String &msg);
void tcpSend(const String &s);

// ---- runtime state ----
struct SensorRt {
  float accum;
  uint16_t accCount;
  uint8_t accStatus;  // worst status in the avg window
  float latestPf;  // zeroed value
  uint8_t status;
  bool fresh;
  uint8_t settleLeft;  // samples to skip after capdac change
  uint32_t nConv;
  uint16_t satRun;
  bool satWarned;
};
SensorRt R[NS];

Preferences prefs;
WebServer server(80);
const uint16_t TCP_PORT = 8888;
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

bool sensorOk = false;
bool serverStarted = false, tcpStarted = false;
uint8_t wifiState = 0;  // 0 off, 1 connecting, 2 connected
uint32_t wifiT0 = 0;
uint32_t wifiRetryAt = 0;  // wifiauto retry time, 0 = none pending
uint32_t wifiLostAt = 0;  // time the link dropped, 0 = ok
uint16_t wifiFails = 0;  // fail count, keeps messages short
const uint32_t WIFI_RETRY_MS = 30000;
String wifiSsid, wifiPass;
bool wifiAuto = USE_WIFI;
String serialBuf, tcpBuf;

uint32_t i2cErrors = 0, convTimeouts = 0, fdcResets = 0, lineCounter = 0;
uint32_t lastResultMs = 0, lastStreamMs = 0, lastHumanMs = 0, lastRetryMs = 0;

// sequencer
enum { SEQ_IDLE, SEQ_WAIT };
uint8_t seqState = SEQ_IDLE;
int seqSensor = 0;  // current sensor
int lastConverted = -1;  // previous one
bool seqDummy = false;  // dummy conversion (settle)
uint32_t seqT0 = 0, seqPollUs = 0;

// ---- helpers ----
bool measOn(int i) { return P.measMask & (1 << i); }
int nActive() { return (measOn(0) ? 1 : 0) + (measOn(1) ? 1 : 0); }
bool isSe(int i) { return P.s[i].mode == MODE_SE; }
uint32_t convUs() { return 1000000UL / P.rateSps; }
uint16_t rateBits() { return P.rateSps == 100 ? 0b01 : P.rateSps == 200 ? 0b10 : 0b11; }

String jsonSafe(String t) { t.replace("\\", "/"); t.replace("\"", "'"); return t; }
void tcpSend(const String &s);
// serial writes must never block the loop. With USB CDC and no host reading, every
// write waited for the TX timeout, the loop stalled >500 ms and the watchdog reset the chip.
// so: send the whole line or drop it (no half JSON lines after a reconnect)
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT && defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE
#define SERIAL_NONBLOCKING 1
#endif
uint32_t serialDropped = 0;
void serialLine(const String &s) {
#ifdef SERIAL_NONBLOCKING
  if (Serial.availableForWrite() < (int)s.length() + 2) { serialDropped++; return; }
#endif
  Serial.println(s);
}
void outLine(const String &s) { serialLine(s); tcpSend(s); }
void sayMsg(const String &t) { outLine("{\"event\":\"msg\",\"text\":\"" + jsonSafe(t) + "\"}"); }

String pinsText(int i) {
  const SensorCfg &c = P.s[i];
  return c.mode == MODE_SE ? String(c.pinA + 1) : String(c.pinA + 1) + "-" + String(c.pinB + 1);
}
String pinsLong(int i) {
  const SensorCfg &c = P.s[i];
  return c.mode == MODE_SE ? "CIN" + String(c.pinA + 1) + " to GND"
                           : "CIN" + String(c.pinA + 1) + " - CIN" + String(c.pinB + 1);
}
uint8_t pinMask(int i) {
  const SensorCfg &c = P.s[i];
  return (1 << c.pinA) | (c.mode == MODE_DIFF ? (1 << c.pinB) : 0);
}

void savePersist() {
  prefs.begin("fdc3", false);
  prefs.putBytes("cfg", &P, sizeof(P));
  prefs.end();
}

bool validPersist(const Persist &p) {
  if (p.magic != PERSIST_MAGIC || p.version != 3) return false;
  if (p.rateSps != 100 && p.rateSps != 200 && p.rateSps != 400) return false;
  if (p.avgN < 1 || p.avgN > 64 || (p.bits != 16 && p.bits != 24) || p.measMask < 1 || p.measMask > 3) return false;
  for (int i = 0; i < NS; i++) {
    const SensorCfg &c = p.s[i];
    if (c.mode > 1 || c.pinA > 3 || c.capdac > 31) return false;
    if (c.mode == MODE_DIFF && (c.pinB > 3 || c.pinB <= c.pinA)) return false;
    if (!isfinite(c.zeroPf) || !isfinite(c.hwOffsetPf) || !isfinite(c.a0) || !isfinite(c.a1) || !isfinite(c.a2)) return false;
  }
  return true;
}

void loadPersist() {
  setDefaults();
  Persist tmp;
  prefs.begin("fdc3", true);
  size_t n = prefs.getBytesLength("cfg");
  bool ok = (n == sizeof(Persist)) && prefs.getBytes("cfg", &tmp, sizeof(tmp)) == sizeof(tmp) && validPersist(tmp);
  wifiSsid = prefs.getString("wssid", "");
  wifiPass = prefs.getString("wpass", "");
  bool haveWifi = prefs.isKey("wauto");
  wifiAuto = prefs.getBool("wauto", USE_WIFI);
  prefs.end();
  if (ok) P = tmp;
  if (!haveWifi) {  // first v3 boot: only keep the v2 wifi creds
    prefs.begin("fdc", true);
    wifiSsid = prefs.getString("wssid", "");
    wifiPass = prefs.getString("wpass", "");
    prefs.end();
    if (wifiSsid.length() == 0 && String(WIFI_SSID) != "YOUR_WIFI_SSID") { wifiSsid = WIFI_SSID; wifiPass = WIFI_PASSWORD; }
  }
}

void saveWifi() {
  prefs.begin("fdc3", false);
  prefs.putString("wssid", wifiSsid);
  prefs.putString("wpass", wifiPass);
  prefs.putBool("wauto", wifiAuto);
  prefs.end();
}

// ---- I2C ----
bool fdcWrite(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(v >> 8));
  Wire.write((uint8_t)(v & 0xFF));
  bool ok = Wire.endTransmission() == 0;
  if (!ok) i2cErrors++;
  return ok;
}

bool fdcRead(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(FDC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { i2cErrors++; return false; }
  if (Wire.requestFrom((int)FDC_ADDR, 2) != 2) { i2cErrors++; return false; }
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  out = (hi << 8) | lo;
  return true;
}

// raw slot result in pF (relative to capdac/offset), sat = clipped
bool fdcReadSlot(uint8_t k, float &pf, bool &sat) {
  uint16_t msb, lsb = 0;
  if (!fdcRead(slotMsb(k), msb)) return false;
  if (P.bits == 24 && !fdcRead(slotLsb(k), lsb)) return false;
  int32_t raw = (int32_t)(int16_t)msb * 256 + ((lsb >> 8) & 0xFF);
  pf = raw / 524288.0f;  // 2^19 counts/pF
  sat = (msb == 0x7FFF || msb == 0x8000 || fabsf(pf) >= RAIL_PF);
  return true;
}

// OFFSET_CAL is Q5.11 signed
bool fdcSetOffsetReg(uint8_t pin, float pf) {
  int32_t r = (int32_t)roundf(pf * 2048.0f);
  if (r > 32767) r = 32767;
  if (r < -32768) r = -32768;
  return fdcWrite(REG_OFFSET_CAL + pin, (uint16_t)(r & 0xFFFF));
}

uint16_t confValue(int i) {
  const SensorCfg &c = P.s[i];
  if (c.mode == MODE_SE) return (c.pinA << 13) | (CHB_CAPDAC << 10) | ((c.capdac & 0x1F) << 5);
  return (c.pinA << 13) | (c.pinB << 10);  // capdac not used in diff
}

bool fdcSoftReset() {
  if (!fdcWrite(REG_FDC_CONF, 0x8000)) return false;
  uint32_t t0 = millis();
  uint16_t v = 0x8000;
  while (millis() - t0 < 100) {
    delay(2);
    if (fdcRead(REG_FDC_CONF, v) && !(v & 0x8000)) { fdcResets++; return true; }
  }
  return false;
}

void resetStream() {
  for (int i = 0; i < NS; i++) {
    R[i].accum = 0; R[i].accCount = 0; R[i].accStatus = ST_OK;
    R[i].fresh = false; R[i].settleLeft = 1;  // skip first result after a change
  }
  seqState = SEQ_IDLE;
  lastConverted = -1;
  lastResultMs = millis();
}

// write every register we depend on, so nothing left over in the chip shifts the reading
bool applyConfig() {
  Wire.setClock((uint32_t)P.i2cKhz * 1000UL);
  bool ok = fdcWrite(REG_FDC_CONF, rateBits() << 10);  // stop conversions
  for (uint8_t p = 0; p < 4; p++) {
    ok &= fdcWrite(REG_GAIN_CAL + p, 0x4000);  // gain = 1.0
    float off = 0;
    for (int i = 0; i < NS; i++)
      if (P.s[i].mode == MODE_DIFF && P.s[i].pinA == p) off = P.s[i].hwOffsetPf;
    ok &= fdcSetOffsetReg(p, off);
  }
  for (int i = 0; i < NS; i++) ok &= fdcWrite(slotConf(i), confValue(i));
  resetStream();
  return ok;
}

// one blocking conversion of slot k (CONF already written)
bool convertSlot(uint8_t k, float &pf, bool &sat) {
  if (!fdcWrite(REG_FDC_CONF, (rateBits() << 10) | slotTrig(k))) return false;
  uint32_t t0 = micros();
  delayMicroseconds(convUs() * 9 / 10);
  uint16_t conf;
  while (micros() - t0 < 4 * convUs() + 20000) {
    if (fdcRead(REG_FDC_CONF, conf) && (conf & slotDone(k))) return fdcReadSlot(k, pf, sat);
    delayMicroseconds(200);
  }
  convTimeouts++;
  return false;
}

// blocking average for sensor i, same conversion order as the stream
// NAN on I2C error, satCount = clipped results
float measureSensor(int i, int n, int &satCount) {
  satCount = 0;
  float sum = 0, v, w;
  bool sat, s2;
  int got = 0;
  int other = 1 - i;
  bool alternate = measOn(other) && measOn(i) && !P.settle;
  for (int k = 0; k < n + 2; k++) {
    if (alternate) convertSlot(other, w, s2);  // same order as the stream
    else if (P.settle) convertSlot(i, w, s2);  // dummy, same as settle 1
    if (!convertSlot(i, v, sat)) continue;
    if (k < 2) continue;  // let it settle
    if (sat) satCount++;
    sum += v;
    got++;
  }
  return got ? sum / got : NAN;
}

// se: absolute C from CINa to GND (raw + capdac), moves capdac so the reading is centered
// returns INFINITY when over range
float seAutorangeMeasure(int i, int n, String &note) {
  SensorCfg &c = P.s[i];
  int satCount;
  for (int iter = 0; iter < 12; iter++) {
    fdcWrite(slotConf(i), confValue(i));
    float v = measureSensor(i, n, satCount);
    if (isnan(v)) { note = "I2C error"; return NAN; }
    if (satCount == 0 && fabsf(v) < RANGE_HI_PF) return v + c.capdac * CAPDAC_PF;
    int step = (satCount > 0) ? (v > 0 ? 4 : -4) : (int)roundf(v / CAPDAC_PF);
    int nc = constrain((int)c.capdac + step, 0, 31);
    if (nc == c.capdac) {
      if (v > 0) { note = "above the CAPDAC range"; return INFINITY; }
      return v + c.capdac * CAPDAC_PF;  // negative at capdac 0, just report it
    }
    c.capdac = nc;
  }
  note = "CAPDAC search did not converge";
  return NAN;
}

// ---- zero / probe / scan ----
String zeroSensor(int i) {
  String tag = "S" + String(i + 1) + " (" + pinsLong(i) + "): ";
  SensorCfg &c = P.s[i];
  if (!measOn(i)) return tag + "skipped, not measured (meas)";
  int satCount;
  if (c.mode == MODE_SE) {
    String note;
    float abs = seAutorangeMeasure(i, 32, note);
    if (!isfinite(abs)) return tag + "ERR " + (note.length() ? note : String("no result"));
    c.zeroPf = abs;
    c.zeroSet = 1;
    return tag + "OK zero = " + String(abs, 4) + " pF to GND (CAPDAC " + String(c.capdac) + ")";
  }
  // diff: center the raw reading with the offset register (both signs tried, not sure about
  // the sign convention), whatever is left goes into the software zero
  c.hwOffsetPf = 0;
  fdcSetOffsetReg(c.pinA, 0);
  float before = measureSensor(i, 32, satCount);
  if (isnan(before)) return tag + "ERR I2C";
  if (satCount > 0) {
    c.zeroSet = 0;
    return tag + "ERR the reading is at the +-16 pF rail: an input is half connected, the sensor is too big for diff "
                 "mode (try mode " + String(i + 1) + " se), or the pins are wrong (pins default)";
  }
  float best = 0, bestAbs = fabsf(before), after = before;
  if (fabsf(before) > 0.02f) {
    const float cand[2] = { before, -before };
    for (int k = 0; k < 2; k++) {
      fdcSetOffsetReg(c.pinA, cand[k]);
      float a = measureSensor(i, 32, satCount);
      if (!isnan(a) && satCount == 0 && fabsf(a) < bestAbs) { bestAbs = fabsf(a); best = cand[k]; after = a; }
    }
  }
  c.hwOffsetPf = best;
  fdcSetOffsetReg(c.pinA, best);
  c.zeroPf = after;  // leftover, a few fF
  c.zeroSet = 1;
  return tag + "OK baseline " + String(before, 4) + " pF, chip offset " + String(best, 4) + " pF, residual " +
         String(after, 4) + " pF";
}

// diagnostics for sensor i on slot 3 (MEAS4): diff + both pins single-ended
// offsets are cleared while it runs, so we see the real pin values
String probeSensor(int i) {
  const SensorCfg &c = P.s[i];
  uint8_t a = c.pinA, b = (c.mode == MODE_DIFF) ? c.pinB : (uint8_t)((c.pinA % 2 == 0) ? c.pinA + 1 : c.pinA - 1);
  if (a > b) { uint8_t t = a; a = b; b = t; }
  for (uint8_t p = 0; p < 4; p++) fdcSetOffsetReg(p, 0);
  float vd = NAN, va = NAN, vb = NAN;
  bool sat = false, s;
  const uint8_t K = 3;
  // diff a-b
  {
    fdcWrite(slotConf(K), (a << 13) | (b << 10));
    float sum = 0, v; int got = 0;
    for (int k = 0; k < 18; k++) if (convertSlot(K, v, s) && k >= 2) { sum += v; got++; sat |= s; }
    if (got) vd = sum / got;
  }
  // single ended, search capdac
  uint8_t pins[2] = { a, b };
  float *outv[2] = { &va, &vb };
  for (int q = 0; q < 2; q++) {
    for (int cd = 0; cd <= 31; cd++) {
      fdcWrite(slotConf(K), (pins[q] << 13) | (CHB_CAPDAC << 10) | (cd << 5));
      float sum = 0, v; int got = 0; bool anySat = false;
      for (int k = 0; k < 10; k++) if (convertSlot(K, v, s) && k >= 2) { sum += v; got++; anySat |= s; }
      if (!got) break;
      float m = sum / got;
      if (!anySat && m < RANGE_HI_PF) { *outv[q] = m + cd * CAPDAC_PF; break; }
      if (cd == 31) *outv[q] = INFINITY;
    }
  }
  applyConfig();
  auto num = [](float v) { return isnan(v) ? String("null") : (isinf(v) ? String("999") : String(v, 4)); };
  outLine("{\"event\":\"probe\",\"s\":" + String(i + 1) + ",\"a\":" + String(a + 1) + ",\"b\":" + String(b + 1) +
          ",\"diff_pf\":" + num(vd) + ",\"diff_sat\":" + (sat ? "true" : "false") + ",\"se_a_pf\":" + num(va) +
          ",\"se_b_pf\":" + num(vb) + "}");
  return "probe S" + String(i + 1) + ": CIN" + String(a + 1) + "-CIN" + String(b + 1) + " = " + (sat ? String("RAIL") : num(vd)) +
         " pF, CIN" + String(a + 1) + " to GND = " + num(va) + " pF, CIN" + String(b + 1) + " to GND = " + num(vb) + " pF";
}

String scanPins() {
  for (uint8_t p = 0; p < 4; p++) fdcSetOffsetReg(p, 0);
  const uint8_t K = 3;
  float val[4];
  bool s;
  for (uint8_t p = 0; p < 4; p++) {
    val[p] = NAN;
    for (int cd = 0; cd <= 31; cd++) {
      fdcWrite(slotConf(K), (p << 13) | (CHB_CAPDAC << 10) | (cd << 5));
      float sum = 0, v; int got = 0; bool anySat = false;
      for (int k = 0; k < 8; k++) if (convertSlot(K, v, s) && k >= 2) { sum += v; got++; anySat |= s; }
      if (!got) break;
      float m = sum / got;
      if (!anySat && m < RANGE_HI_PF) { val[p] = m + cd * CAPDAC_PF; break; }
      if (cd == 31) val[p] = INFINITY;
    }
  }
  applyConfig();
  String j = "{\"event\":\"scan\"";
  String t = "scan (pin to GND):";
  for (int p = 0; p < 4; p++) {
    String v = isnan(val[p]) ? String("null") : (isinf(val[p]) ? String("999") : String(val[p], 3));
    j += ",\"cin" + String(p + 1) + "_pf\":" + v;
    t += " CIN" + String(p + 1) + " " + (isinf(val[p]) ? String(">110") : v) + " pF";
  }
  outLine(j + "}");
  return t + ". A pin with a cable + electrode reads clearly higher than a bare pin.";
}

// ---- streaming: single conversions, round robin ----
float calibrated(int i, float v) { const SensorCfg &c = P.s[i]; return c.a2 * v * v + c.a1 * v + c.a0; }

void handleResult(int i, float raw, bool sat) {
  SensorCfg &c = P.s[i];
  SensorRt &r = R[i];
  r.nConv++;
  lastResultMs = millis();
  if (r.settleLeft) { r.settleLeft--; return; }

  uint8_t st = ST_OK;
  float absPf = raw;
  if (c.mode == MODE_SE) {
    if (c.autorange && (sat || fabsf(raw) > RANGE_HI_PF)) {
      int step = sat ? (raw > 0 ? 4 : -4) : (int)roundf(raw / CAPDAC_PF);
      int nc = constrain((int)c.capdac + step, 0, 31);
      if (nc != c.capdac) {
        c.capdac = nc;
        fdcWrite(slotConf(i), confValue(i));
        r.settleLeft = 2;
        st = ST_SETTLE;
      }
    }
    absPf = raw + c.capdac * CAPDAC_PF;
    if (st != ST_SETTLE && sat) st = (raw > 0 && c.capdac == 31) ? ST_OVER : ST_SAT;
  } else if (sat) {
    st = ST_SAT;
  }

  if (st == ST_SETTLE) {  // keep last value, just flag it
    r.status = ST_SETTLE;
    r.fresh = true;
    return;
  }
  if (sat) {
    if (r.satRun < 60000) r.satRun++;
    if (r.satRun == 200 && !r.satWarned) {
      r.satWarned = true;
      sayMsg("WARNING S" + String(i + 1) + " (" + pinsLong(i) + ") is at the +-16 pF rail: one input not connected, "
             "sensor too large for this mode, or wrong pins. Run 'probe " + String(i + 1) + "'. A railed sensor also "
             "disturbs the other one: 'meas " + String(2 - i) + "' to measure only the other sensor.");
    }
  } else {
    r.satRun = 0;
    r.satWarned = false;
  }
  r.accum += absPf - c.zeroPf;
  r.accCount++;
  if (st > r.accStatus) r.accStatus = st;
  if (r.accCount >= P.avgN) {
    r.latestPf = r.accum / r.accCount;
    r.status = r.accStatus;
    r.accum = 0; r.accCount = 0; r.accStatus = ST_OK;
    r.fresh = true;
  }
}

int nextSensor(int cur) {
  for (int k = 1; k <= NS; k++) {
    int j = (cur + k + NS) % NS;
    if (measOn(j)) return j;
  }
  return -1;
}

void serviceStream() {
  uint32_t now = micros();
  if (seqState == SEQ_IDLE) {
    int s = nextSensor(lastConverted < 0 ? NS - 1 : lastConverted);
    if (s < 0) return;
    // settle 1: drop the first conversion after switching sensor
    seqDummy = P.settle && nActive() > 1 && lastConverted != s;
    seqSensor = s;
    if (!fdcWrite(REG_FDC_CONF, (rateBits() << 10) | slotTrig(s))) return;
    seqT0 = now;
    seqPollUs = convUs() * 9 / 10;
    seqState = SEQ_WAIT;
    return;
  }
  if ((uint32_t)(now - seqT0) < seqPollUs) return;
  uint16_t conf = 0;
  if (fdcRead(REG_FDC_CONF, conf) && (conf & slotDone(seqSensor))) {
    float v; bool sat;
    bool ok = fdcReadSlot(seqSensor, v, sat);
    lastConverted = seqSensor;
    seqState = SEQ_IDLE;
    if (ok && !seqDummy) handleResult(seqSensor, v, sat);
    else if (ok && seqDummy) lastResultMs = millis();
    if (seqDummy) {  // now convert it for real
      seqDummy = false;
      if (fdcWrite(REG_FDC_CONF, (rateBits() << 10) | slotTrig(seqSensor))) {
        seqT0 = micros(); seqPollUs = convUs() * 9 / 10; seqState = SEQ_WAIT;
      }
    }
    return;
  }
  seqPollUs = (uint32_t)(micros() - seqT0) + 200;  // poll again in 200 us
  if ((uint32_t)(micros() - seqT0) > 4 * convUs() + 5000) { // no DONE flag -> count + restart
    convTimeouts++;
    uint16_t dummy;
    fdcRead(slotMsb(seqSensor), dummy);  // clear stale DONE
    seqState = SEQ_IDLE;
    lastConverted = seqSensor;
  }
}

// ---- JSON ----
String configJson() {
  String w = sharedOk() ? "false" : "true";
  char b[1100];
  snprintf(b, sizeof(b),
           "{\"event\":\"config\",\"fw\":\"two_channels\",\"fw3\":true,\"ver\":\"%s\",\"rate\":%u,\"avg\":%u,\"bits\":%u,"
           "\"stream_ms\":%u,\"i2c_khz\":%u,\"meas\":%u,\"settle\":%u,\"human\":%u,"
           "\"pins1\":\"%s\",\"pins2\":\"%s\",\"mode1\":\"%s\",\"mode2\":\"%s\",\"shared\":%s,"
           "\"capdac1\":%u,\"capdac2\":%u,\"autorange1\":%s,\"autorange2\":%s,"
           "\"offset1_pf\":%.5f,\"offset2_pf\":%.5f,\"zero1_pf\":%.5f,\"zero2_pf\":%.5f,\"zeroed1\":%s,\"zeroed2\":%s,"
           "\"gain1\":%.7g,\"shift1\":%.7g,\"quad1\":%.7g,\"cal1\":%s,\"gain2\":%.7g,\"shift2\":%.7g,\"quad2\":%.7g,\"cal2\":%s,"
           "\"out_hz\":%.2f,\"i2c_err\":%lu,\"timeouts\":%lu,\"resets\":%lu,"
           "\"wifi\":%u,\"ssid\":\"%s\",\"ip\":\"%s\",\"wifi_auto\":%s,\"tcp_port\":%u}",
           FW_VERSION, P.rateSps, P.avgN, P.bits, P.streamMs, P.i2cKhz, P.measMask, P.settle, P.human,
           pinsText(0).c_str(), pinsText(1).c_str(), isSe(0) ? "se" : "diff", isSe(1) ? "se" : "diff", w.c_str(),
           P.s[0].capdac, P.s[1].capdac, P.s[0].autorange ? "true" : "false", P.s[1].autorange ? "true" : "false",
           P.s[0].hwOffsetPf, P.s[1].hwOffsetPf, P.s[0].zeroPf, P.s[1].zeroPf,
           P.s[0].zeroSet ? "true" : "false", P.s[1].zeroSet ? "true" : "false",
           P.s[0].a1, P.s[0].a0, P.s[0].a2, P.s[0].calSet ? "true" : "false",
           P.s[1].a1, P.s[1].a0, P.s[1].a2, P.s[1].calSet ? "true" : "false",
           estimatedRate(), (unsigned long)i2cErrors, (unsigned long)convTimeouts, (unsigned long)fdcResets,
           wifiState, jsonSafe(wifiSsid).c_str(), wifiState == 2 ? WiFi.localIP().toString().c_str() : "",
           wifiAuto ? "true" : "false", TCP_PORT);
  return String(b);
}

bool sharedOk() { return (pinMask(0) & pinMask(1)) == 0; }

// lines/s per sensor (approx: conversion time + I2C overhead)
float estimatedRate() {
  int n = nActive();
  if (!n) return 0;
  float convPerSample = (float)n * ((P.settle && n > 1) ? 2.0f : 1.0f);
  float overheadUs = P.i2cKhz >= 400 ? 350.0f : 1100.0f;
  return 1e6f / (convPerSample * (1e6f / P.rateSps + overheadUs)) / P.avgN;
}

String numOrNull(bool on, float v) { return on ? String(v, 5) : String("null"); }

String dataJson() {
  String s = "{\"c1_pf\":" + numOrNull(measOn(0), R[0].latestPf) +
             ",\"c2_pf\":" + numOrNull(measOn(1), R[1].latestPf) +
             ",\"c1_cal_pf\":" + numOrNull(measOn(0) && P.s[0].calSet, calibrated(0, R[0].latestPf)) +
             ",\"c2_cal_pf\":" + numOrNull(measOn(1) && P.s[1].calSet, calibrated(1, R[1].latestPf)) +
             ",\"st1\":\"" + (measOn(0) ? ST_NAME[R[0].status] : "off") + "\"" +
             ",\"st2\":\"" + (measOn(1) ? ST_NAME[R[1].status] : "off") + "\"" +
             ",\"cd1\":" + String(P.s[0].capdac) + ",\"cd2\":" + String(P.s[1].capdac) +
             ",\"n\":" + String(lineCounter) +
             ",\"sensor_ok\":" + (sensorOk ? "true" : "false") +
             ",\"millis\":" + String(millis()) + "}";
  return s;
}

// ---- WiFi (same as v2) ----
String handleCommand(String line);
void emitResponse(const String &msg);

void tcpSend(const String &s) {
  if (!tcpStarted || !tcpClient || !tcpClient.connected()) return;
  if (tcpClient.print(s + "\n") == 0) tcpClient.stop();
}

void handleData() { server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(200, "application/json", dataJson()); }
void handleRoot() { server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(200, "text/plain", "FDC1004 v3. GET /data, GET /cmd?c=help"); }
void handleCmd() {
  String msg = jsonSafe(handleCommand(server.arg("c")));
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":" + String(msg.indexOf("ERR") >= 0 ? "false" : "true") +
              ",\"msg\":\"" + msg + "\",\"config\":" + configJson() + "}");
}

// last disconnect reason + async scan, for debugging
volatile int wifiLastReason = 0;
String wifiReasonText() {
  if (!wifiLastReason) return "no reason reported";
#ifdef ESP_PLATFORM
  return String(WiFi.disconnectReasonName((wifi_err_reason_t)wifiLastReason)) + " (" + String(wifiLastReason) + ")";
#else
  return String(wifiLastReason);
#endif
}
void wifiEventsInit() {
#ifdef ESP_PLATFORM
  static bool done = false;
  if (done) return;
  done = true;
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t info) {
    int r = info.wifi_sta_disconnected.reason;
    // 8 ASSOC_LEAVE / 36 STA_LEAVING = radio signing off, keep the cause we saw before
    if (!(r == 8 || r == 36) || !wifiLastReason) wifiLastReason = r;
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
#endif
}

bool wifiScanning = false;
String wifiScanStart() {
#ifdef ESP_PLATFORM
  if (wifiScanning) return "OK scan already running";
  if (wifiState == 0) WiFi.mode(WIFI_STA);
  WiFi.scanDelete();
  if (WiFi.scanNetworks(true) == WIFI_SCAN_FAILED) return "ERR scan failed to start";
  wifiScanning = true;
  return "OK scanning (2.4 GHz only), results in a few seconds";
#else
  return "ERR scan not available";
#endif
}
void wifiScanService() {
#ifdef ESP_PLATFORM
  if (!wifiScanning) return;
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;
  wifiScanning = false;
  if (n < 0) { sayMsg("WiFi scan failed"); return; }
  static const char *AUTH[] = {"open", "WEP", "WPA", "WPA2", "WPA/WPA2", "WPA2-Enterprise", "WPA3", "WPA2/WPA3",
                               "WAPI", "OWE", "WPA3-Enterprise"};
  sayMsg("WiFi scan: " + String(n) + " networks");
  for (int i = 0; i < n; i++) {
    int a = (int)WiFi.encryptionType(i);
    sayMsg("  " + String(WiFi.SSID(i) == wifiSsid ? "* " : "  ") + "\"" + WiFi.SSID(i) + "\"  ch " +
           String(WiFi.channel(i)) + "  " + String(WiFi.RSSI(i)) + " dBm  " +
           (a >= 0 && a < (int)(sizeof(AUTH) / sizeof(AUTH[0])) ? AUTH[a] : String(a).c_str()));
  }
  WiFi.scanDelete();
  if (wifiState == 0) WiFi.mode(WIFI_OFF);
#endif
}

void wifiStart() {
  if (wifiSsid.length() == 0) return;
  wifiEventsInit();
  wifiLastReason = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  wifiState = 1;
  wifiT0 = millis();
}

void wifiStop() {
  wifiRetryAt = 0;
  wifiLostAt = 0;
  if (tcpStarted) { tcpClient.stop(); tcpServer.end(); tcpStarted = false; }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiState = 0;
}

void wifiConnected() {
  wifiState = 2;
  if (!serverStarted) {
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/cmd", handleCmd);
    server.begin();
    serverStarted = true;
  }
  if (!tcpStarted) { tcpServer.begin(); tcpServer.setNoDelay(true); tcpStarted = true; }
  sayMsg("WiFi connected, IP " + WiFi.localIP().toString() + ", TCP port " + String(TCP_PORT));
  outLine(configJson());
}

// wifiauto on: retry every 30 s and reconnect if the link drops
// (e.g. phone hotspot that wasn't visible at power-up)
void wifiService() {
  uint32_t now = millis();
  if (wifiState == 0) {
    if (wifiRetryAt && (int32_t)(now - wifiRetryAt) >= 0) {
      wifiRetryAt = 0;
      if (wifiAuto && wifiSsid.length()) wifiStart();
    }
    return;
  }
  if (wifiState == 1) {
    if (WiFi.status() == WL_CONNECTED) { wifiFails = 0; wifiConnected(); return; }
    if (now - wifiT0 <= 15000) return;
    String why = wifiReasonText();
    wifiStop();
    wifiFails++;
    if (wifiAuto) {
      wifiRetryAt = now + WIFI_RETRY_MS;
      if (wifiFails == 1 || wifiFails % 10 == 0)
        sayMsg("WiFi not connected to \"" + wifiSsid + "\": " + why + " - attempt " + String(wifiFails) +
               ", retrying every 30 s (wifi off to stop, wifiscan to list networks)");
    } else {
      sayMsg("WiFi not connected to \"" + wifiSsid + "\": " + why + " - radio switched off. Send wifiscan to list networks");
    }
    return;
  }
  // connected
  if (WiFi.status() == WL_CONNECTED) { wifiLostAt = 0; return; }
  if (!wifiLostAt) { wifiLostAt = now ? now : 1; return; }
  if (now - wifiLostAt > 10000) {
    String why = wifiReasonText();
    sayMsg("WiFi connection lost (" + why + ") - reconnecting");
    wifiStop();
    if (wifiAuto) wifiStart(); else sayMsg("wifiauto is off: send wifi on to reconnect");
  }
}

void tcpService() {
  if (wifiState != 2 || !tcpStarted) return;
  if (tcpServer.hasClient()) {
    WiFiClient c = tcpServer.accept();
    if (c) { tcpClient.stop(); tcpClient = c; tcpClient.setNoDelay(true); tcpBuf = ""; tcpSend(configJson()); }
  }
  while (tcpClient && tcpClient.connected() && tcpClient.available()) {
    char ch = (char)tcpClient.read();
    if (ch == '\n' || ch == '\r') {
      if (tcpBuf.length()) { String l = tcpBuf; tcpBuf = ""; emitResponse(handleCommand(l)); }
    } else if (tcpBuf.length() < 200) tcpBuf += ch;
  }
}

// ---- commands ----
int parseMask(const String &s) {
  if (s == "1") return 1;
  if (s == "2") return 2;
  if (s == "all" || s == "both") return 3;
  return 0;
}

int splitTokens(const String &line, String t[4]) {
  int n = 0, pos = 0, L = line.length();
  while (n < 4 && pos < L) {
    while (pos < L && line[pos] == ' ') pos++;
    if (pos >= L) break;
    int e = line.indexOf(' ', pos);
    if (e < 0) e = L;
    t[n++] = line.substring(pos, e);
    pos = e;
  }
  return n;
}

void clearZero(int i) { P.s[i].zeroPf = 0; P.s[i].zeroSet = 0; P.s[i].hwOffsetPf = 0; }
void clearCal(int i) { P.s[i].a0 = 0; P.s[i].a1 = 1; P.s[i].a2 = 0; P.s[i].calSet = 0; }

String warnings() {
  String w;
  for (int i = 0; i < NS; i++)
    if (measOn(i) && !P.s[i].zeroSet) w += " S" + String(i + 1) + " not zeroed (send zero " + String(i + 1) + " with nothing connected).";
  return w;
}

String statusText() {
  String t = "v" + String(FW_VERSION) + ":";
  for (int i = 0; i < NS; i++) {
    const SensorCfg &c = P.s[i];
    t += " S" + String(i + 1) + " " + (measOn(i) ? "on" : "off") + " " + pinsLong(i) + " [" + (c.mode == MODE_SE ? "se" : "diff") + "]";
    if (c.mode == MODE_SE) t += " capdac " + String(c.capdac) + (c.autorange ? " auto" : " fixed");
    t += c.zeroSet ? " zero " + String(c.zeroPf, 3) + " pF" : String(" NOT ZEROED");
    t += c.calSet ? " calibrated" : " no cal";
    t += ";";
  }
  t += " i2c errors " + String(i2cErrors) + ", timeouts " + String(convTimeouts) + ", resets " + String(fdcResets) +
       ", serial lines dropped " + String(serialDropped);
  return t;
}

// set pins/mode of sensor i, refuse pins the other sensor already uses
String setPins(int i, uint8_t mode, int a, int b) {
  SensorCfg old = P.s[i];
  P.s[i].mode = mode;
  P.s[i].pinA = a;
  P.s[i].pinB = (mode == MODE_DIFF) ? b : 0;
  if (!sharedOk()) {
    P.s[i] = old;
    return "ERR S" + String(i + 1) + " would share a pin with S" + String(2 - i) +
           ": a change on one sensor would appear in the other. Change S" + String(2 - i) + " first or use pins default";
  }
  if (old.mode != mode || old.pinA != a || old.pinB != P.s[i].pinB) { clearZero(i); clearCal(i); P.s[i].capdac = 0; }
  savePersist();
  if (!applyConfig()) return "ERR I2C write failed";
  return "OK S" + String(i + 1) + " = " + pinsLong(i) + " (" + (mode == MODE_SE ? "se" : "diff") +
         "). Zero and calibration cleared: send zero " + String(i + 1);
}

String handleCommand(String line) {
  line.trim();
  String raw = line;
  line.toLowerCase();
  if (line.length() == 0) return "";
  String t[4];
  int nt = splitTokens(line, t);
  String cmd = t[0];

  if (cmd == "get") return "OK";
  if (cmd == "status") return statusText() + warnings();
  if (cmd == "help")
    return "get; status; meas 1|2|all; mode <s> diff|se; pins <s> <a> [b]; pins default; zero [s]; unzero [s]; "
           "autorange <s> on|off; capdac <s> 0-31; offset <s> pF; gain|shift|quad <s> v; calclear [s]; probe <s>; scan; "
           "rate 100|200|400; avg 1-64; bits 16|24; stream ms; i2c 100|400; settle 0|1; human on|off; fdcreset; reg [hex]; "
           "regw hex hex; factory; wifissid; wifipass; wifiauto on|off; wificlear; wifi [on|off]; wifiscan";

  if (cmd == "rate") {
    int r = t[1].toInt();
    if (r != 100 && r != 200 && r != 400) return "ERR rate must be 100, 200 or 400";
    P.rateSps = r; savePersist();
    return applyConfig() ? "OK rate " + String(r) + ". The zero depends on the rate: send zero again" : "ERR I2C";
  }
  if (cmd == "avg") {
    int n = t[1].toInt();
    if (n < 1 || n > 64) return "ERR avg must be 1 to 64";
    P.avgN = n; savePersist(); resetStream();
    return "OK avg " + String(n);
  }
  if (cmd == "bits") {
    int b = t[1].toInt();
    if (b != 16 && b != 24) return "ERR bits must be 16 or 24";
    P.bits = b; savePersist(); resetStream();
    return "OK bits " + String(b);
  }
  if (cmd == "stream") {
    int ms = t[1].toInt();
    if (ms < 0 || ms > 5000) return "ERR stream must be 0 to 5000 ms";
    P.streamMs = ms; savePersist();
    return "OK stream " + String(ms) + " ms";
  }
  if (cmd == "i2c") {
    int k = t[1].toInt();
    if (k != 100 && k != 400) return "ERR i2c must be 100 or 400";
    P.i2cKhz = k; savePersist(); Wire.setClock((uint32_t)k * 1000UL);
    return "OK i2c " + String(k) + " kHz";
  }
  if (cmd == "settle") {
    if (t[1] != "0" && t[1] != "1") return "ERR settle 0|1";
    P.settle = t[1].toInt(); savePersist(); resetStream();
    return P.settle ? "OK settle 1: first conversion after switching sensor is discarded (half rate)" : "OK settle 0";
  }
  if (cmd == "human") {
    P.human = (t[1] == "on"); savePersist();
    return P.human ? "OK human-readable line every 500 ms" : "OK human-readable line off";
  }
  if (cmd == "meas") {
    int m = parseMask(t[1]);
    if (!m) return "ERR meas 1|2|all";
    P.measMask = m; savePersist();
    if (!applyConfig()) return "ERR I2C";
    return String("OK measuring ") + (m == 3 ? "both sensors" : m == 1 ? "S1 only" : "S2 only") + "." + warnings();
  }
  if (cmd == "pins" && t[1] == "default") {
    for (int i = 0; i < NS; i++) { P.s[i].mode = MODE_DIFF; P.s[i].pinA = 2 * i; P.s[i].pinB = 2 * i + 1; P.s[i].capdac = 0; clearZero(i); clearCal(i); }
    savePersist();
    return applyConfig() ? "OK pins default: S1 = CIN1 - CIN2, S2 = CIN3 - CIN4 (diff). Send zero" : "ERR I2C";
  }
  if (cmd == "pins") {
    int m = parseMask(t[1]);
    int a = t[2].toInt(), b = (nt >= 4) ? t[3].toInt() : 0;
    if ((m != 1 && m != 2) || a < 1 || a > 4 || (b != 0 && (b <= a || b > 4)))
      return "ERR pins <1|2> <a> <b> (a<b, differential) or pins <1|2> <a> (single-ended)";
    return setPins(m - 1, b ? MODE_DIFF : MODE_SE, a - 1, b ? b - 1 : 0);
  }
  if (cmd == "mode") {
    int m = parseMask(t[1]);
    if (!m || (t[2] != "diff" && t[2] != "se")) return "ERR mode <1|2|all> diff|se";
    String out;
    for (int i = 0; i < NS; i++) {
      if (!(m & (1 << i))) continue;
      uint8_t a = P.s[i].pinA, b = (P.s[i].mode == MODE_DIFF) ? P.s[i].pinB : (uint8_t)(a + 1);
      if (t[2] == "diff" && b > 3) return "ERR S" + String(i + 1) + ": CIN4 cannot be the A pin of a differential pair";
      out += (out.length() ? " | " : "") + setPins(i, t[2] == "se" ? MODE_SE : MODE_DIFF, a, b);
    }
    return out;
  }
  if (cmd == "zero" || cmd == "cal" || cmd == "c" || cmd == "autocapdac") {
    int m = (nt >= 2) ? parseMask(t[1]) : 3;
    if (!m) return "ERR zero [1|2|all]";
    String out;
    for (int i = 0; i < NS; i++) if (m & (1 << i)) out += (out.length() ? " | " : "") + zeroSensor(i);
    savePersist();
    applyConfig();
    return out;
  }
  if (cmd == "unzero") {
    int m = (nt >= 2) ? parseMask(t[1]) : 3;
    if (!m) return "ERR unzero [1|2|all]";
    for (int i = 0; i < NS; i++) if (m & (1 << i)) clearZero(i);
    savePersist(); applyConfig();
    return "OK zero cleared";
  }
  if (cmd == "autorange" || cmd == "capdac" || cmd == "offset" || cmd == "gain" || cmd == "shift" || cmd == "quad") {
    int m = parseMask(t[1]);
    if (!m || nt < 3) return "ERR usage: " + cmd + " <1|2|all> <value>";
    float v = t[2].toFloat();
    for (int i = 0; i < NS; i++) {
      if (!(m & (1 << i))) continue;
      SensorCfg &c = P.s[i];
      if (cmd == "autorange") {
        if (t[2] != "on" && t[2] != "off") return "ERR autorange <s> on|off";
        c.autorange = (t[2] == "on");
      } else if (cmd == "capdac") {
        int cd = t[2].toInt();
        if (cd < 0 || cd > 31) return "ERR capdac 0..31";
        if (c.mode != MODE_SE) return "ERR S" + String(i + 1) + ": CAPDAC has no effect in diff mode (datasheet 6.4.2); use mode " + String(i + 1) + " se";
        c.capdac = cd; c.autorange = 0;
      } else if (cmd == "offset") {
        if (c.mode != MODE_DIFF) return "ERR offset is for diff mode; in se mode the zero is stored in software";
        c.hwOffsetPf = constrain(v, -15.99f, 15.99f);
      } else {
        if (!isfinite(v) || (cmd == "gain" && (v == 0 || fabsf(v) > 100)) || fabsf(v) > 1000) return "ERR value out of range";
        if (cmd == "gain") c.a1 = v; else if (cmd == "shift") c.a0 = v; else c.a2 = v;
        c.calSet = 1;
      }
    }
    savePersist();
    if (cmd == "autorange" || cmd == "capdac" || cmd == "offset") applyConfig();
    return "OK " + cmd + " " + t[1] + " " + t[2];
  }
  if (cmd == "calclear") {
    int m = (nt >= 2) ? parseMask(t[1]) : 3;
    if (!m) return "ERR calclear [1|2|all]";
    for (int i = 0; i < NS; i++) if (m & (1 << i)) clearCal(i);
    savePersist();
    return "OK calibration cleared";
  }
  if (cmd == "probe") {
    int m = (nt >= 2) ? parseMask(t[1]) : 1;
    if (m != 1 && m != 2) return "ERR probe 1|2";
    return probeSensor(m - 1);
  }
  if (cmd == "scan") return scanPins();
  if (cmd == "fdcreset") return (fdcSoftReset() && applyConfig()) ? "OK FDC1004 reset" : "ERR reset failed";
  if (cmd == "factory") {
    prefs.begin("fdc3", false); prefs.clear(); prefs.end();
    setDefaults(); savePersist(); saveWifi();
    return applyConfig() ? "OK all settings, zeros and calibrations forgotten (defaults)" : "ERR I2C";
  }
  if (cmd == "regw" && nt >= 3) {
    uint8_t r = (uint8_t)strtol(t[1].c_str(), nullptr, 16);
    uint16_t v = (uint16_t)strtol(t[2].c_str(), nullptr, 16);
    return fdcWrite(r, v) ? "OK written (not saved; applyConfig may overwrite it)" : "ERR I2C";
  }
  if (cmd == "reg") {
    String out = "regs:";
    const uint8_t list[] = { 0x00, 0x01, 0x02, 0x03, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0xFE, 0xFF };
    for (uint8_t r : list) {
      if (nt >= 2 && strtol(t[1].c_str(), nullptr, 16) != r) continue;
      uint16_t v = 0;
      bool ok = fdcRead(r, v);
      char b[20];
      snprintf(b, sizeof(b), " %02X=%s%04X", r, ok ? "" : "ERR", v);
      out += b;
    }
    return out;
  }
  if (cmd == "wifissid" || cmd == "wifipass") {
    int sp = raw.indexOf(' ');
    String arg = sp < 0 ? String("") : raw.substring(sp + 1);
    arg.trim();
    if (cmd == "wifissid") { if (arg.length() == 0 || arg.length() > 32) return "ERR name must be 1-32 characters"; wifiSsid = arg; }
    else { if (arg.length() > 63) return "ERR password max 63 characters"; wifiPass = arg; }
    saveWifi();
    return "OK saved";
  }
  if (cmd == "wifiscan") return wifiScanStart();
  if (cmd == "wificlear") { wifiSsid = ""; wifiPass = ""; wifiAuto = false; saveWifi(); wifiStop(); return "OK WiFi forgotten"; }
  if (cmd == "wifiauto") {
    if (t[1] != "on" && t[1] != "off") return "ERR wifiauto on|off";
    wifiAuto = (t[1] == "on"); saveWifi();
    return "OK wifiauto " + t[1];
  }
  if (cmd == "wifi") {
    if (t[1] == "off") { wifiStop(); return "OK WiFi off"; }
    if (t[1] == "on") {
      if (!wifiSsid.length()) return "ERR no network: wifissid <name>, wifipass <password>";
      wifiStart();
      return "OK connecting";
    }
    return wifiState == 2 ? "OK WiFi connected, IP " + WiFi.localIP().toString() : (wifiState == 1 ? "OK connecting" : (wifiRetryAt ? "OK WiFi not connected, next attempt in " +
           String((int32_t)(wifiRetryAt - millis()) / 1000) + " s" : "OK WiFi off"));
  }
  return "ERR unknown command, send help";
}

void emitResponse(const String &msg) {
  if (msg.length()) sayMsg(msg);
  outLine(configJson());
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length()) { String l = serialBuf; serialBuf = ""; emitResponse(handleCommand(l)); }
    } else if (serialBuf.length() < 120) serialBuf += c;
  }
}

// ---- setup / loop ----
bool detectSensor() {
  uint16_t man = 0, dev = 0;
  sensorOk = fdcRead(REG_MANUFACTURER_ID, man) && man == 0x5449 && fdcRead(REG_DEVICE_ID, dev) && dev == 0x1004;
  sayMsg("FDC1004 ID 0x" + String(man, HEX) + "/0x" + String(dev, HEX) + (sensorOk ? " OK" : " NOT FOUND (expect 5449/1004)"));
  return sensorOk;
}

void setup() {
#ifdef SERIAL_NONBLOCKING
  Serial.setTxBufferSize(4096);  // before begin(), default 256 B is too small for the config line
#endif
  Serial.begin(460800);
#ifdef SERIAL_NONBLOCKING
  Serial.setTxTimeoutMs(0);  // don't block on the USB host (see serialLine)
#endif
  delay(300);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  loadPersist();
  Wire.setClock((uint32_t)P.i2cKhz * 1000UL);
  sayMsg(String("FDC1004 firmware v") + FW_VERSION + ". No automatic zero at power-up: zeros are stored.");
  if (detectSensor()) {
    bool ok = fdcSoftReset() && applyConfig();
    sayMsg(String("FDC1004 configured ") + (ok ? "OK" : "FAILED"));
  }
  sayMsg(statusText() + warnings());
  if (wifiAuto && wifiSsid.length()) {
    wifiStart();  // non-blocking, wifiService() does the rest
    sayMsg("WiFi connecting to \"" + wifiSsid + "\"");
  } else {
    wifiStop();
  }
  outLine(configJson());
}

void loop() {
  if (serverStarted) server.handleClient();
  wifiService();
  wifiScanService();
  tcpService();
  pollSerial();

  uint32_t nowMs = millis();
  if (!sensorOk) {
    if (nowMs - lastRetryMs > 2000) {
      lastRetryMs = nowMs;
      if (detectSensor()) { fdcSoftReset(); applyConfig(); }
    }
    return;
  }

  serviceStream();

  if (nActive() && millis() - lastResultMs > 500) {  // watchdog
    bool ok = fdcSoftReset() && applyConfig();
    sayMsg(String("FDC1004 stopped converting - reset ") + (ok ? "OK" : "FAILED"));
    if (!ok) sensorOk = false;
  }

  bool ready = nActive() > 0;
  for (int i = 0; i < NS; i++) if (measOn(i) && !R[i].fresh) ready = false;
  nowMs = millis();
  if (ready && (P.streamMs == 0 || nowMs - lastStreamMs >= P.streamMs)) {
    for (int i = 0; i < NS; i++) R[i].fresh = false;
    lastStreamMs = nowMs;
    lineCounter++;
    outLine(dataJson());
  }

  if (P.human && nowMs - lastHumanMs >= 500) {
    lastHumanMs = nowMs;
    serialLine("S1 " + String(R[0].latestPf, 4) + " pF [" + ST_NAME[R[0].status] + "]  S2 " + String(R[1].latestPf, 4) +
                   " pF [" + ST_NAME[R[1].status] + "]");
  }
}
