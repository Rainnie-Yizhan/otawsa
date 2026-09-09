#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_TSL2561_U.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <sys/time.h>
#include <SD.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>

/* ================= WIFI ================= */
const char* ssid     = "@IoT";
const char* password = NULL;

WebServer server(80);

/* ================= GITHUB OTA CONFIG ================= */
const char* ota_bin_url = "https://raw.githubusercontent.com/Rainnie-Yizhan/otawsa/main/firmware.bin";

/* ================= SD CARD ================= */
#define SD_MISO  32
#define SD_MOSI  13
#define SD_SCK   17
#define SD_CS    25
const char* dataFile = "/data.csv";
const char* tempFile = "/temp.csv";
unsigned long lastSDWrite = 0;
const unsigned long sdInterval = 60000;
int pendingCount = 0;
bool sdWriteSuccess = false;
String lastSDWriteTime = "Never";

SPIClass mySPI(VSPI);

/* ================= PIN CONFIG ================= */
#define SPEED_PIN 27
#define RAIN_PIN  26
#define DIR_PIN   33

/* ================= ADS1115 ================= */
#define SOIL1_CH 1
#define SOIL2_CH 2

Adafruit_ADS1115        ads;
Adafruit_BME280         bme;
Adafruit_BMP280         bmp;
Adafruit_TSL2561_Unified tsl(TSL2561_ADDR_FLOAT, 12345);

/* ================= FLAGS ================= */
bool hasADS = false, hasBME = false, hasBMP = false, hasTSL = false, hasSD = false;

/* ================= MOVING AVERAGE (WIND DIRECTION) ================= */
const int numReadings = 20;
int   readings[numReadings];
int   readIndex  = 0;
long  totalADC   = 0;
unsigned long lastADCRead = 0;

/* ================= WIND ================= */
volatile unsigned long pulseCount = 0;
volatile int64_t lastWindTime = 0;
unsigned long lastCalcTime = 0;
float  windSpeed   = 0;
float  windRPM     = 0;
String windDir     = "UNK";
int    windDirADC  = 0;

struct WindDirection { const char* name; int adcValue; };
const WindDirection windMap[] = {
  {"N",3230},{"NE",2800},{"E",3480},{"SE",3095},
  {"S",3760},{"SW",3340},{"W",3860},{"NW",2970}
};

/* ================= RAIN ================= */
volatile unsigned long rainTipCount = 0;
volatile int64_t lastRainTime = 0;
volatile unsigned long lastRainTipMillis = 0;
const float mmPerTip = 0.47;
float rainMM = 0;
float lastHourRainMM = 0;
int lastResetHour = -1;

/* ================= SENSOR VALUES ================= */
int   soil1Pct = 0, soil2Pct = 0;
float temp = 0, hum = 0, pres = 0, lux = 0;

/* ================= SLOW SENSOR TIMING ================= */
unsigned long lastSlowRead = 0;
const unsigned long slowInterval = 15000;
unsigned long lastTimeCheck = 0;

/* ================= WIFI TRACKING ================= */
bool wasWifiConnected = false;
unsigned long wifiOffTime = 0;
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 30000;

/* ================= WATCHDOG ================= */
const int wdtTimeoutSec = 60;

/* ================= INTERRUPTS ================= */
void IRAM_ATTR windISR() {
  int64_t now = esp_timer_get_time();
  if (now - lastWindTime > 3000) { 
    pulseCount++; 
    lastWindTime = now; 
  }
}

void IRAM_ATTR rainISR() {
  // [แก้ไขแล้ว] กรองสัญญาณรบกวน (Noise/Spike) จากสาย RJ11
  // ถ้าขาไม่ได้เป็น LOW จริงๆ ให้เด้งออกทันที
  if (digitalRead(RAIN_PIN) == HIGH) {
    return;
  }

  int64_t now = esp_timer_get_time();
  if (now - lastRainTime > 80000) {
    rainTipCount++;
    lastRainTime = now;
    lastRainTipMillis = (unsigned long)(now / 1000); 
  }
}

/* ================= WIND DIRECTION ================= */
String getWindDirection(int adc) {
  int minDiff = 4096; String result = "UNK";
  int numDirs = sizeof(windMap) / sizeof(windMap[0]);
  for (int i = 0; i < numDirs; i++) {
    int diff = abs(adc - windMap[i].adcValue);
    if (diff < minDiff) { minDiff = diff; result = windMap[i].name; }
  }
  return result;
}

/* ================= SOIL ================= */
int soilPercent(int adc, int wet = 7600, int dry = 14000) {
  adc = constrain(adc, wet, dry);
  return map(adc, dry, wet, 0, 100);
}

/* ================= ISO8601 TIMESTAMP ================= */
String getISOTimestamp() {
  time_t now;
  time(&now);
  if (now < 946684800) return "1970-01-01T00:00:00+07:00";
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S+07:00", &timeinfo);
  return String(buf);
}

/* ================= SENSOR UPDATE ================= */
void updateSensors() {
  windDir = getWindDirection(windDirADC);
  if (hasADS) {
    long soil1Sum = 0;
    long soil2Sum = 0;
    
    for(int i = 0; i < 10; i++) {
      soil1Sum += ads.readADC_SingleEnded(SOIL1_CH);
      soil2Sum += ads.readADC_SingleEnded(SOIL2_CH);
      delay(5);
    }
    
    int soil1ADC = soil1Sum / 10;
    int soil2ADC = soil2Sum / 10;
    
    soil1Pct = soilPercent(soil1ADC, 4000, 14000);
    soil2Pct = soilPercent(soil2ADC, 4000, 14000);
  } else {
    soil1Pct = 0;
    soil2Pct = 0;
  }
  
  // [แก้ไขแล้ว] ปิด Interrupt แป๊บเดียวเพื่อดึงค่ามาคำนวณอย่างปลอดภัย
  noInterrupts();
  unsigned long currentTips = rainTipCount;
  interrupts();
  
  rainMM = currentTips * mmPerTip;
}

void updateSlowSensors() {
  if (hasBME) {
    temp = bme.readTemperature();
    hum  = bme.readHumidity();
    pres = bme.readPressure() / 100.0F;
  } else if (hasBMP) {
    temp = bmp.readTemperature();
    pres = bmp.readPressure() / 100.0F;
  }
  if (hasTSL) {
    sensors_event_t event;
    tsl.getEvent(&event);
    lux = event.light;
  }
}

/* ================= CHECK HOURLY RAIN RESET ================= */
void checkHourlyRainReset() {
  time_t nowTime;
  time(&nowTime);
  if (nowTime > 946684800) {
    struct tm timeinfo;
    localtime_r(&nowTime, &timeinfo);
    
    if (lastResetHour == -1) {
      lastResetHour = timeinfo.tm_hour;
    } else if (timeinfo.tm_hour != lastResetHour) {
      
      // [แก้ไขแล้ว] เอาเงื่อนไขการชะลอ 5 นาทีออก เพื่อรีเซ็ตตัดยอดรายชั่วโมงให้แม่นยำ
      noInterrupts();
      lastHourRainMM = rainMM;
      rainTipCount = 0;
      rainMM = 0;
      interrupts();
      
      lastResetHour = timeinfo.tm_hour;
      Serial.printf("[RAIN] Hourly reset completed (Previous hour: %.2f mm)\n", lastHourRainMM);
    }
  }
}

/* ================= SD WRITE ================= */
void writeToSD() {
  if (!hasSD) {
    sdWriteSuccess = false;
    return;
  }
  File file = SD.open(dataFile, FILE_APPEND);
  if (!file) {
    sdWriteSuccess = false;
    Serial.println("[ERROR] SD open failed");
    return;
  }

  String ts = getISOTimestamp();
  String line = ts + "," +
                String(temp, 2)    + "," + String(hum, 2)      + "," +
                String(rainMM, 4)  + "," + String(windSpeed, 2) + "," +
                windDir            + "," +
                String(soil1Pct)   + "," + String(soil2Pct)    + "," +
                String(lux, 2)     + "," + String(pres, 2);
  file.println(line);
  file.close();
  pendingCount++;
  sdWriteSuccess = true;
  lastSDWriteTime = ts;

  Serial.print("[SD] Saved | pending: "); Serial.println(pendingCount);
}

/* ================= MICRO-BATCHING PENDING STREAM ================= */
void handlePendingStream() {
  if (!hasSD || !SD.exists(dataFile)) {
    server.send(200, "application/json", "[]");
    return;
  }

  File src = SD.open(dataFile, FILE_READ);
  if (!src) {
    server.send(500, "text/plain", "Failed to open SD");
    return;
  }

  String header = src.readStringUntil('\n');
  header.trim();

  if (!src.available()) {
    src.close();
    server.send(200, "application/json", "[]");
    return;
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("[");

  const int BATCH_SIZE = 50;
  int sentCount = 0;
  bool first = true;

  while (src.available() && sentCount < BATCH_SIZE) {
    String row = src.readStringUntil('\n');
    row.trim();
    if (row.length() == 0) continue;

    int pos = 0, field = 0;
    String cols[10];
    while (field < 10) {
      int cp = row.indexOf(',', pos);
      if (cp == -1) { cols[field] = row.substring(pos); break; }
      cols[field] = row.substring(pos, cp);
      pos = cp + 1; field++;
    }

    String item = "";
    if (!first) item += ",";
    item += "{\"timestamp\":\"" + cols[0] + "\","
          + "\"temp\":"        + cols[1] + ","
          + "\"humid\":"       + cols[2] + ","
          + "\"rain\":"        + cols[3] + ","
          + "\"wind_speed\":"  + cols[4] + ","
          + "\"wind_dir\":\""  + cols[5] + "\","
          + "\"soil1\":"       + cols[6] + ","
          + "\"soil2\":"       + cols[7] + ","
          + "\"light\":"       + cols[8] + ","
          + "\"pres\":"        + cols[9] + "}";

    server.sendContent(item);
    first = false;
    sentCount++;
    esp_task_wdt_reset();
    yield();
  }
  server.sendContent("]");

  if (sentCount > 0) {
    if (SD.exists(tempFile)) SD.remove(tempFile);
    File dst = SD.open(tempFile, FILE_WRITE);

    if (dst) {
      dst.println(header.length() > 0 ? header : "timestamp,temp,humid,rain,windspeed,winddir,soil1,soil2,light,pressure");
      int remainingRows = 0;

      while (src.available()) {
        String row = src.readStringUntil('\n');
        row.trim();
        if (row.length() > 0) {
          dst.println(row);
          remainingRows++;
        }
        esp_task_wdt_reset();
        yield();
      }

      dst.close();
      src.close();

      SD.remove(dataFile);
      SD.rename(tempFile, dataFile);

      pendingCount = remainingRows;
      Serial.printf("[SD Queue] Popped %d rows | Remaining: %d\n", sentCount, pendingCount);
    } else {
      src.close();
      Serial.println("[ERROR] Failed to open temp file for queue pop");
    }
  } else {
    src.close();
  }
}

/* ================= GITHUB OTA UPDATE HANDLER ================= */
void performGithubOTA() {
  Serial.println("[OTA] Checking for firmware update from GitHub...");
  server.send(200, "text/plain", "Starting OTA Update from GitHub... Please wait.");
  delay(1000);

  esp_task_wdt_delete(NULL);

  WiFiClientSecure clientSecure;
  clientSecure.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  t_httpUpdate_return ret = httpUpdate.update(clientSecure, ota_bin_url);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("[OTA] Update failed! Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      esp_task_wdt_add(NULL);
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA] No updates available");
      esp_task_wdt_add(NULL);
      break;
    case HTTP_UPDATE_OK:
      Serial.println("[OTA] Update successful. Rebooting...");
      break;
  }
}

/* ================= WEB PAGE ================= */
String webpage() {
return R"====(
<!DOCTYPE html><html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{margin:0;font-family:'Segoe UI',Tahoma,sans-serif;background:#0d0d0d;color:#ffffff;text-align:center;min-height:100vh;}
.container{max-width:900px;margin:20px auto;padding:15px;}
h2{margin-bottom:18px;font-weight:700;letter-spacing:1.5px;color:#ffffff;text-transform:uppercase;}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:12px;}
.card{background:#141414;border:1px solid #333333;border-radius:8px;padding:12px;}
.label{font-size:11px;color:#888888;text-transform:uppercase;letter-spacing:0.5px;font-weight:600;}
.value{font-size:18px;font-weight:bold;margin-top:6px;color:#ffffff;}
.panel{background:#141414;border:1px solid #333333;border-radius:8px;padding:16px;margin-top:16px;text-align:left;}
.panel-title{font-weight:bold;font-size:13px;margin-bottom:12px;text-transform:uppercase;letter-spacing:1px;color:#cccccc;}
.status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px;font-size:13px;}
.tag{display:inline-block;padding:2px 8px;border-radius:4px;font-weight:bold;font-size:11px;}
.ok{background:#27ae60;color:#ffffff;}
.fail{background:#e74c3c;color:#ffffff;}
.btn{background:#e67e22;color:#ffffff;border:none;padding:8px 16px;border-radius:6px;font-weight:bold;cursor:pointer;margin-top:10px;margin-right:6px;transition:0.2s;}
.btn:hover{background:#d35400;}
.btn-ota{background:#9b59b6;}
.btn-ota:hover{background:#8e44ad;}
.time{font-size:12px;color:#777777;margin-top:14px;text-align:center;}
</style>
</head>
<body>
<div class="container">
  <h2>ESP32 Weather Monitor</h2>
  <div class="grid">
    <div class="card"><div class="label">Wind Speed</div><div class="value" id="speed">--</div></div>
    <div class="card"><div class="label">Wind Dir</div><div class="value" id="dir">--</div></div>
    <div class="card"><div class="label">Wind RPM</div><div class="value" id="wrpm">--</div></div>
    <div class="card"><div class="label">Rain (Hour)</div><div class="value" id="rain">--</div></div>
    <div class="card"><div class="label">Rain Tips</div><div class="value" id="rtips">--</div></div>
    <div class="card"><div class="label">Soil 1</div><div class="value" id="soil1">--</div></div>
    <div class="card"><div class="label">Soil 2</div><div class="value" id="soil2">--</div></div>
    <div class="card"><div class="label">Temperature</div><div class="value" id="temp">--</div></div>
    <div class="card"><div class="label">Humidity</div><div class="value" id="hum">--</div></div>
    <div class="card"><div class="label">Pressure</div><div class="value" id="pres">--</div></div>
    <div class="card"><div class="label">Light (Lux)</div><div class="value" id="lux">--</div></div>
  </div>

  <div class="panel">
    <div class="panel-title">System Diagnostic & Control</div>
    <div class="status-grid">
      <div>ADS1115: <span id="st_ads" class="tag">--</span></div>
      <div>BME/BMP: <span id="st_env" class="tag">--</span></div>
      <div>TSL2561: <span id="st_tsl" class="tag">--</span></div>
      <div>SD Mount: <span id="st_sd" class="tag">--</span></div>
      <div>Last SD Write: <span id="st_sdw" class="tag">--</span></div>
      <div>Pending Rows: <span id="st_rows" style="font-weight:bold;color:#ffffff;">--</span></div>
    </div>
    <div style="margin-top:14px;border-top:1px solid #222222;padding-top:12px;">
      <button class="btn" onclick="resetRain()">Reset Rain</button>
      <button class="btn btn-ota" onclick="triggerOTA()">Check GitHub OTA</button>
    </div>
  </div>

  <div class="time" id="clock">Device Time: --</div>
</div>
<script>
fetch('/settime?ts=' + Math.floor(Date.now() / 1000));

function resetRain(){
  if(confirm("Do you want to reset the rain counter to 0?")){
    fetch('/resetrain').then(r=>r.text()).then(msg=>alert(msg));
  }
}

function triggerOTA(){
  if(confirm("Do you want to check and install firmware updates from GitHub?")){
    fetch('/checkota').then(r=>r.text()).then(msg=>alert(msg));
  }
}

setInterval(function(){
  fetch("/data?t=" + Date.now()).then(r=>r.json()).then(d=>{
    document.getElementById("speed").innerHTML   = d.wind_speed + " km/h";
    document.getElementById("dir").innerHTML     = d.wind_dir;
    document.getElementById("wrpm").innerHTML    = d.wind_rpm + " RPM";
    document.getElementById("rain").innerHTML    = d.rain + " mm";
    document.getElementById("rtips").innerHTML   = d.rain_tips;
    document.getElementById("soil1").innerHTML   = d.soil1 + " %";
    document.getElementById("soil2").innerHTML   = d.soil2 + " %";
    document.getElementById("temp").innerHTML    = d.temp + " &deg;C";
    document.getElementById("hum").innerHTML     = d.humid + " %";
    document.getElementById("pres").innerHTML    = d.pres + " hPa";
    document.getElementById("lux").innerHTML     = d.light;
    document.getElementById("clock").innerHTML   = "Device Time: " + d.timestamp;

    setTag("st_ads", d.status.ads);
    setTag("st_env", d.status.env);
    setTag("st_tsl", d.status.tsl);
    setTag("st_sd", d.status.sd);
    setTag("st_sdw", d.status.sd_write);
    document.getElementById("st_rows").innerHTML = d.status.pending + " rows";
  });
}, 500);

function setTag(id, ok){
  let el = document.getElementById(id);
  el.className = "tag " + (ok ? "ok" : "fail");
  el.innerHTML = ok ? "ONLINE" : "OFFLINE";
}
</script>
</body></html>
)====";
}

/* ================= SETUP ================= */
void setup() {
  btStop();
  setCpuFrequencyMhz(240);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 Weather Station ===");

  esp_task_wdt_init(wdtTimeoutSec, true);
  esp_task_wdt_add(NULL);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(150);
  
  ads.setGain(GAIN_ONE);
  hasADS = ads.begin();

  if (bme.begin(0x76) || bme.begin(0x77)) {
    hasBME = true;
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X16,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_500);
  } else if (bmp.begin(0x76) || bmp.begin(0x77)) {
    hasBMP = true;
  }

  hasTSL = tsl.begin();

  mySPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS, mySPI)) {
    hasSD = true;
    if (!SD.exists(dataFile)) {
      File f = SD.open(dataFile, FILE_APPEND);
      if (f) {
        f.println("timestamp,temp,humid,rain,windspeed,winddir,soil1,soil2,light,pressure");
        f.close();
      }
    }
  } else {
    hasSD = false;
  }

  pinMode(SPEED_PIN, INPUT_PULLUP);
  pinMode(RAIN_PIN,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_PIN), windISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN),  rainISR, FALLING);

  for (int i = 0; i < numReadings; i++) readings[i] = 0;

  updateSlowSensors();
  lastSlowRead = millis();

  WiFi.begin(ssid, password);
  int timeout = 20;
  while (WiFi.status() != WL_CONNECTED && timeout--) { delay(500); Serial.print("."); }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[OK] WiFi | IP: "); Serial.println(WiFi.localIP());
    WiFi.setSleep(WIFI_PS_NONE); 
  }

  server.on("/", []() { server.send(200, "text/html", webpage()); });

  server.on("/settime", []() {
    if (server.hasArg("ts")) {
      long ts = server.arg("ts").toInt();
      if (ts > 946684800) {
        struct timeval tv = { .tv_sec = ts, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        setenv("TZ", "UTC-7", 1);
        tzset();
        server.send(200, "text/plain", "OK");
        return;
      }
    }
    server.send(400, "text/plain", "Bad Request");
  });

  server.on("/resetrain", []() {
    noInterrupts();
    rainTipCount = 0;
    rainMM = 0;
    interrupts();
    server.send(200, "text/plain", "Rain counter reset to 0 mm");
  });

  server.on("/data", []() {
    String json = "{";
    json += "\"timestamp\":\""   + getISOTimestamp()      + "\",";
    json += "\"temp\":"          + String(temp, 2)        + ",";
    json += "\"humid\":"         + String(hum, 2)         + ",";
    json += "\"rain\":"          + String(rainMM, 4)      + ",";
    json += "\"rain_tips\":"     + String(rainTipCount)   + ",";
    json += "\"wind_speed\":"    + String(windSpeed, 2)   + ",";
    json += "\"wind_dir\":\""    + windDir                + "\",";
    json += "\"wind_rpm\":"      + String(windRPM, 1)     + ",";
    json += "\"soil1\":"         + String(soil1Pct)       + ",";
    json += "\"soil2\":"         + String(soil2Pct)       + ",";
    json += "\"light\":"         + String(lux, 2)         + ",";
    json += "\"pres\":"          + String(pres, 2)        + ",";
    json += "\"status\":{";
    json += "\"ads\":"           + String(hasADS ? "true" : "false") + ",";
    json += "\"env\":"           + String((hasBME || hasBMP) ? "true" : "false") + ",";
    json += "\"tsl\":"           + String(hasTSL ? "true" : "false") + ",";
    json += "\"sd\":"            + String(hasSD ? "true" : "false") + ",";
    json += "\"sd_write\":"      + String(sdWriteSuccess ? "true" : "false") + ",";
    json += "\"pending\":"       + String(pendingCount);
    json += "}}";
    server.send(200, "application/json", json);
  });

  server.on("/pending", handlePendingStream);
  server.on("/checkota", performGithubOTA);

  server.begin();
  Serial.println("[OK] Web Server Ready");
}

/* ================= LOOP ================= */
void loop() {
  unsigned long now = millis();

  if (now > 172800000UL) {
    Serial.println("[SYSTEM] 48h Auto-Restart triggered");
    delay(1000);
    ESP.restart();
  }

  if (now - lastADCRead >= 30) {
    totalADC = totalADC - readings[readIndex];
    readings[readIndex] = analogRead(DIR_PIN);
    totalADC = totalADC + readings[readIndex];
    readIndex = (readIndex + 1) % numReadings;
    windDirADC = totalADC / numReadings;
    lastADCRead = now;
  }

  if (now - lastCalcTime >= 1000) {
    noInterrupts();
    unsigned long count = pulseCount; 
    pulseCount = 0;
    interrupts();

    windRPM = (count / 2.0) * 60.0;
    windSpeed = count * 1.548;
    
    updateSensors();
    lastCalcTime = now;
  }

  if (now - lastSlowRead >= slowInterval) {
    updateSlowSensors();
    lastSlowRead = now;
  }

  if (now - lastTimeCheck >= 10000) {
    checkHourlyRainReset();
    lastTimeCheck = now;
  }

  bool isWifiOnline = (WiFi.status() == WL_CONNECTED);
  if (!isWifiOnline && wasWifiConnected) wifiOffTime = millis();
  wasWifiConnected = isWifiOnline;

  if (!isWifiOnline && (now - lastReconnectAttempt >= reconnectInterval)) {
    Serial.println("[WiFi] Offline, reconnecting...");
    WiFi.disconnect(); WiFi.reconnect();
    lastReconnectAttempt = now;
  }

  if (now - lastSDWrite >= sdInterval) {
    writeToSD();
    lastSDWrite = now;
  }

  server.handleClient();
  esp_task_wdt_reset();
}