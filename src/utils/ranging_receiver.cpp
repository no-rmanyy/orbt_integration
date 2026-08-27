#include <Arduino.h>
#include "utils/ranging_receiver.h"
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>
#include <esp_wifi.h>

static const uint8_t RANGING_CHANNEL = 6; 

// ---- Must match the anchor's struct EXACTLY (byte-identical) ----
typedef struct {
  uint8_t anchor_id;
  float   distance_m;   // -1 = anchor has no valid reading this window
} __attribute__((packed)) RangingMsg;

// ---- Field geometry (metres, origin at centre) - MUST match physical layout ----
// anchor_id 1..3 map to these positions in order.
static const float FIELD_RADIUS = 0.30;
static const float BX[3] = { 0.0,      0.2598, -0.2598 };  // anchor 1,2,3 x
static const float BY[3] = { 0.30,    -0.15,   -0.15   };  // anchor 1,2,3 y

// ---- Position Kalman tunables ----
static const float PKF_Q       = 0.05;
static const float PKF_R_BASE  = 0.02;
static const float PKF_R_SCALE = 2.0;

// ---- Freshness: a distance older than this (ms) is treated as stale ----
static const uint32_t STALE_MS = 500;

// ---- Received distances, written by ESP-NOW callback ----
static volatile float    dist[4]    = { -1, -1, -1, -1 };  // index by anchor_id
static volatile uint32_t lastMs[4]  = { 0, 0, 0, 0 };

// ---- Position Kalman state ----
static float pk_px = 0, pk_py = 0, pk_P = 1.0;
static bool  pk_init = false;

// ---- Latest fix ----
static bool  haveFix = false;
static float fixX = 0, fixY = 0, fixRes = 0;

// ==========================================================================
// ESP-NOW receive callback (newer IDF signature: esp_now_recv_info_t*)
// ==========================================================================
static void onEspNowRecv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int len) {
  Serial.print("ESPNOW rx len="); Serial.println(len);   // <-- add this first line
  if (len != sizeof(RangingMsg)) return;
  RangingMsg msg;
  memcpy(&msg, data, sizeof(msg));
  if (msg.anchor_id >= 1 && msg.anchor_id <= 3) {
    dist[msg.anchor_id]   = msg.distance_m;
    lastMs[msg.anchor_id] = millis();
  }
}

// ---- trilateration (weighted least squares) ----
static bool weightedLeastSquares(const float d[3], float &x, float &y) {
  float w[3];
  for (int i = 0; i < 3; i++) w[i] = 1.0 / (d[i]*d[i] + 0.01);

  float M00=0,M01=0,M11=0,v0=0,v1=0;
  for (int j = 1; j < 3; j++) {
    float a = 2*(BX[j]-BX[0]);
    float b = 2*(BY[j]-BY[0]);
    float c = d[0]*d[0]-d[j]*d[j] + BX[j]*BX[j]-BX[0]*BX[0] + BY[j]*BY[j]-BY[0]*BY[0];
    float rw = (w[0] < w[j]) ? w[0] : w[j];
    M00 += rw*a*a; M01 += rw*a*b; M11 += rw*b*b;
    v0  += rw*a*c; v1  += rw*b*c;
  }
  float det = M00*M11 - M01*M01;
  if (fabs(det) < 1e-9) return false;
  x = ( M11*v0 - M01*v1) / det;
  y = (-M01*v0 + M00*v1) / det;
  return true;
}

static void clampToField(float &x, float &y) {
  float r = sqrt(x*x + y*y);
  if (r > FIELD_RADIUS) { x = x*FIELD_RADIUS/r; y = y*FIELD_RADIUS/r; }
}

static float computeResidual(float x, float y, const float d[3]) {
  float ss = 0;
  for (int i = 0; i < 3; i++) {
    float pred = sqrt((x-BX[i])*(x-BX[i]) + (y-BY[i])*(y-BY[i]));
    float e = pred - d[i];
    ss += e*e;
  }
  return sqrt(ss/3);
}

static void positionKalman(float xm, float ym, float res) {
  if (!pk_init) { pk_px=xm; pk_py=ym; pk_P=1.0; pk_init=true; fixX=xm; fixY=ym; return; }
  pk_P += PKF_Q;
  float R = PKF_R_BASE + PKF_R_SCALE * res;
  float S = pk_P + R;
  float K = pk_P / S;
  pk_px += K*(xm - pk_px);
  pk_py += K*(ym - pk_py);
  pk_P   = (1.0 - K)*pk_P;
  fixX = pk_px; fixY = pk_py;
}

// ==========================================================================
void rangingReceiverBegin() {
  WiFi.mode(WIFI_STA);                                          // radio on, no network
  esp_wifi_set_channel(RANGING_CHANNEL, WIFI_SECOND_CHAN_NONE); // pin channel

  if (esp_now_init() != ESP_OK) {
    Serial.println("Ranging: ESP-NOW init FAILED");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  // confirm actual channel
  uint8_t primary; wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.print("Ranging receiver ready, channel "); Serial.println(primary);
}

void rangingReceiverUpdate() {
  uint32_t now = millis();

  float d[3];
  bool allFresh = true;
  for (int i = 0; i < 3; i++) {
    float v  = dist[i+1];
    uint32_t t = lastMs[i+1];
    if (v < 0 || (now - t) > STALE_MS) allFresh = false;
    d[i] = v;
  }

  // throttled serial print every 200ms
  static uint32_t lastPrint = 0;
  bool doPrint = (now - lastPrint >= 200);
  if (doPrint) lastPrint = now;

  if (!allFresh) {
    if (doPrint) {
      Serial.print("waiting: d1="); Serial.print(dist[1], 2);
      Serial.print(" d2="); Serial.print(dist[2], 2);
      Serial.print(" d3="); Serial.print(dist[3], 2);
      Serial.println();
    }
    haveFix = false;
    return;
  }

  float xr, yr;
  if (!weightedLeastSquares(d, xr, yr)) { haveFix = false; return; }
  clampToField(xr, yr);
  fixRes = computeResidual(xr, yr, d);
  positionKalman(xr, yr, fixRes);
  haveFix = true;

  if (doPrint) {
    Serial.print("d1="); Serial.print(d[0], 2);
    Serial.print(" d2="); Serial.print(d[1], 2);
    Serial.print(" d3="); Serial.print(d[2], 2);
    Serial.print(" | pos=("); Serial.print(fixX, 3);
    Serial.print(", "); Serial.print(fixY, 3);
    Serial.print(") res="); Serial.println(fixRes, 3);
  }
}

bool  rangingHasFix()   { return haveFix; }
float rangingX()        { return fixX; }
float rangingY()        { return fixY; }
float rangingResidual() { return fixRes; }
float rangingDistance(uint8_t id) { return (id>=1 && id<=3) ? dist[id] : -1; }

