#include <TFT_eSPI.h>
#include <math.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <algorithm> // std::sort用

// 3D生態系シミュレーション

// GPIOピン定義
#define CYD_BACKLIGHT_PIN 21

// タッチスクリーン(HSPI)ピン
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

// タッチスクリーンキャリブレーション
#define TOUCH_MIN_X 200
#define TOUCH_MAX_X 3700
#define TOUCH_MIN_Y 300
#define TOUCH_MAX_Y 3800

// 解像度とオフセット
#define TFT_WIDTH  320
#define TFT_HEIGHT 170
#define OFFSET_Y   ((240 - TFT_HEIGHT) / 2)

// ワールド座標境界
#define WORLD_MIN_X -90.0f
#define WORLD_MAX_X  90.0f
#define WORLD_MIN_Y -50.0f
#define WORLD_MAX_Y  50.0f
#define WORLD_MIN_Z  -100.0f
#define WORLD_MAX_Z  240.0f

// 60FPSタイミング定数(16666us)
#define FRAME_TIME_US 16666

// ワールドZ範囲
#define WORLD_Z_RANGE      (WORLD_MAX_Z - WORLD_MIN_Z)
#define WORLD_Z_RANGE_INV  (1.0f / WORLD_Z_RANGE)

// 視覚用Z範囲
#define VISUAL_MIN_Z       10.0f
#define VISUAL_Z_RANGE     (WORLD_MAX_Z - VISUAL_MIN_Z)
#define VISUAL_Z_RANGE_INV (1.0f / VISUAL_Z_RANGE)

const bool SWAP_RB = false; 

// 前方宣言
inline float Q_rsqrt(float number);

// 3Dベクトル
struct Vec3 {
  float x, y, z;
  
  Vec3() : x(0), y(0), z(0) {}
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  inline Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
  inline Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
  inline Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
  // 除算の乗算化
  inline Vec3 operator/(float s) const { float inv = 1.0f / s; return Vec3(x * inv, y * inv, z * inv); }

  inline Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
  inline Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
  inline Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

  inline float lengthSq() const { return x * x + y * y + z * z; }

  // 長さ計算(高速逆平方根)
  inline float length() const {
    float lenSq = lengthSq();
    if (lenSq < 1.0e-10f) return 0.0f;
    return lenSq * Q_rsqrt(lenSq);
  }

  // 正規化
  inline Vec3 normalized() const {
    float lenSq = lengthSq();
    if (lenSq > 1.0e-10f) {
      float invLen = Q_rsqrt(lenSq);
      return Vec3(x * invLen, y * invLen, z * invLen);
    }
    return Vec3(0, 0, 0);
  }

  // 計算済みの長さの二乗を用いた正規化
  inline Vec3 normalized(float lenSq) const {
    if (lenSq > 1.0e-10f) {
      float invLen = Q_rsqrt(lenSq);
      return Vec3(x * invLen, y * invLen, z * invLen);
    }
    return Vec3(0, 0, 0);
  }

  // 速度制限
  inline Vec3 limitedTo(float maxLen) const {
    float lenSq = lengthSq();
    float maxLenSq = maxLen * maxLen;
    if (lenSq > maxLenSq && lenSq > 1.0e-10f) {
      float scale = maxLen * Q_rsqrt(lenSq);
      return Vec3(x * scale, y * scale, z * scale);
    }
    return *this;
  }
};

const int MAX_PLANTS    = 50; 
const int MAX_HERBS     = 40;
const int MAX_CARNS     = 8;
const int MAX_APEX      = 3;   
const int MAX_DECOMPS   = 12;
const int MAX_SPORES    = 20;
const int MAX_PLANKTON  = 50;
const int MAX_GARBAGES  = 25;
const int MAX_PARTICLES = 120;
const int MAX_WAVES     = 15;
const int HISTORY_LEN   = 16; 
const int MAX_TOTAL_ENTITIES = MAX_PLANTS + MAX_HERBS + MAX_CARNS + MAX_APEX + MAX_DECOMPS + MAX_SPORES + MAX_PLANKTON + MAX_GARBAGES + MAX_PARTICLES + MAX_WAVES;

// 剰余演算の最適化
static_assert((HISTORY_LEN & (HISTORY_LEN - 1)) == 0, "HISTORY_LEN must be a power of two");
#define HISTORY_LEN_MASK (HISTORY_LEN - 1)

struct Entity3D {
  bool active;
  Vec3 pos;
  Vec3 vel;
  float energy;
  Vec3 hist[HISTORY_LEN];
  int histIdx;
  float flash;
  bool infected;
  int targetId;
  float speedLimit;
  int age;
  float altruism;
  float immunity;
  float radius;
};

struct Plant3D { bool active; Vec3 pos; float radius; };
struct Spore3D { bool active; Vec3 pos; Vec3 vel; };
struct Plankton3D { Vec3 pos; Vec3 vel; int layer; };
struct Garbage3D { bool active; Vec3 pos; uint8_t r, g, b; };
struct Particle3D { bool active; Vec3 pos; Vec3 vel; float life; uint8_t r, g, b; };
struct Shockwave3D { bool active; Vec3 pos; float radius; float maxRadius; float life; uint8_t r, g, b; };

// グローバルオブジェクト
SPIClass touchSpi = SPIClass(HSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);
TFT_eSprite topHud = TFT_eSprite(&tft);
TFT_eSprite botHud = TFT_eSprite(&tft);

int clockHour = 17;
int clockMin  = 54;
int clockSec  = 0;
unsigned long lastClockTick = 0; 

static uint8_t cydLut[256];
static bool cydLutInited = false;

void initCydColorLUT() {
  for (int i = 0; i < 256; i++) {
    float norm = i / 255.0f;
    float enhanced = norm * norm * (3.0f - 2.0f * norm);
    int val = (int)(enhanced * 255.0f);
    if (val > 255) val = 255;
    if (val < 0) val = 0;
    cydLut[i] = (uint8_t)val;
  }
  cydLutInited = true;
}

inline uint16_t packRGB565(uint8_t r, uint8_t g, uint8_t b) {
  // 初期化済み前提でLUT参照
  uint8_t er = cydLut[r];
  uint8_t eg = cydLut[g];
  uint8_t eb = cydLut[b];
  if (SWAP_RB) return ((eb & 0xF8) << 8) | ((eg & 0xFC) << 3) | (er >> 3);
  return ((er & 0xF8) << 8) | ((eg & 0xFC) << 3) | (eb >> 3);
}

// 高速逆平方根
inline float Q_rsqrt(float number) {
  long i;
  float x2, y;
  const float threehalfs = 1.5F;
  x2 = number * 0.5F;
  y  = number;
  memcpy(&i, &y, sizeof(i));
  i  = 0x5f3759df - ( i >> 1 );
  memcpy(&y, &i, sizeof(y));
  y  = y * ( threehalfs - ( x2 * y * y ) );
  return y;
}

// 高速逆数近似
inline float Q_rcp(float x) {
  union { float f; int32_t i; } conv;
  conv.f = x;
  conv.i = 0x7EEEEEEE - conv.i;
  conv.f = conv.f * (2.0f - x * conv.f); // ニュートン法で補正
  return conv.f;
}

// 奥行きの減衰係数
inline float getDepthSpeedFactor(float z) {
  float normDepth = (z - VISUAL_MIN_Z) * VISUAL_Z_RANGE_INV;
  normDepth = constrain(normDepth, 0.0f, 1.0f);
  return 1.30f - (normDepth * 0.90f);
}

// ローレンツ・アトラクタ(風)
inline Vec3 getAttractorForce(const Vec3& pos) {
  // 座標マッピング
  float ax = pos.x * (20.0f / WORLD_MAX_X);
  float ay = pos.y * (30.0f / WORLD_MAX_Y);
  float az = (pos.z - WORLD_MIN_Z) * (50.0f / WORLD_Z_RANGE);

  // 微分方程式
  float dx = 10.0f * (ay - ax);
  float dy = ax * (28.0f - az) - ay;
  float dz = ax * ay - 2.66f * az;

  // 正規化
  return Vec3(dx, dy, dz).normalized();
}

bool isFingerTouching = false;
Vec3 fingerWorldPos(0, 0, 70.0f);

SemaphoreHandle_t dataMutex;

Plant3D plants[MAX_PLANTS];
Entity3D herbs[MAX_HERBS];
Entity3D carns[MAX_CARNS];
Entity3D apex[MAX_APEX];
Entity3D decomps[MAX_DECOMPS];
Spore3D spores[MAX_SPORES];
Plankton3D planktons[MAX_PLANKTON];
Garbage3D garbages[MAX_GARBAGES];
Particle3D particles[MAX_PARTICLES];
Shockwave3D shockwaves[MAX_WAVES];
int garbageIdx = 0;

// アクティブエンティティ数
int activeHerbCount = 0;
int activeCarnCount = 0;
int activeApexCount = 0;
int activeDecompCount = 0;

// 描画用バッファ
Plant3D snap_plants[MAX_PLANTS];
Entity3D snap_herbs[MAX_HERBS];
Entity3D snap_carns[MAX_CARNS];
Entity3D snap_apex[MAX_APEX];
Entity3D snap_decomps[MAX_DECOMPS];
Spore3D snap_spores[MAX_SPORES];
Plankton3D snap_planktons[MAX_PLANKTON];
Garbage3D snap_garbages[MAX_GARBAGES];
Particle3D snap_particles[MAX_PARTICLES];
Shockwave3D snap_shockwaves[MAX_WAVES];
bool snap_isFingerTouching = false;
Vec3 snap_fingerWorldPos;

void captureSnapshot() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(snap_plants, plants, sizeof(plants));
  memcpy(snap_herbs, herbs, sizeof(herbs));
  memcpy(snap_carns, carns, sizeof(carns));
  memcpy(snap_apex, apex, sizeof(apex));
  memcpy(snap_decomps, decomps, sizeof(decomps));
  memcpy(snap_spores, spores, sizeof(spores));
  memcpy(snap_planktons, planktons, sizeof(planktons));
  memcpy(snap_garbages, garbages, sizeof(garbages));
  memcpy(snap_particles, particles, sizeof(particles));
  memcpy(snap_shockwaves, shockwaves, sizeof(shockwaves));
  snap_isFingerTouching = isFingerTouching;
  snap_fingerWorldPos = fingerWorldPos;
  xSemaphoreGive(dataMutex);
}

TaskHandle_t Task1;

uint8_t histPlants[320] = {0};
uint8_t histHerbs[320] = {0};
uint8_t histCarns[320] = {0};
uint8_t histApex[320] = {0};
int histCursor = 0;

// ワイヤーフレーム描画
void drawWireframeDigit(TFT_eSprite& spr, int x, int y, int digit, int w, int h, uint16_t color) {
  static const uint8_t segs[10] = {
    0b01111110, // 0
    0b00110000, // 1
    0b01101101, // 2
    0b01111001, // 3
    0b00110011, // 4
    0b01011011, // 5
    0b01011111, // 6
    0b01110000, // 7
    0b01111111, // 8
    0b01111011  // 9
  };

  if (digit < 0 || digit > 9) return;
  uint8_t mask = segs[digit];
  int hh = h / 2;

  if (mask & 0b01000000) spr.drawFastHLine(x, y, w, color);
  if (mask & 0b00100000) spr.drawFastVLine(x + w, y, hh, color);
  if (mask & 0b00010000) spr.drawFastVLine(x + w, y + hh, hh, color);
  if (mask & 0b00001000) spr.drawFastHLine(x, y + h, w, color);
  if (mask & 0b00000100) spr.drawFastVLine(x, y + hh, hh, color);
  if (mask & 0b00000010) spr.drawFastVLine(x, y, hh, color);
  if (mask & 0b00000001) spr.drawFastHLine(x, y + hh, w, color);
}

void drawWireframeColon(TFT_eSprite& spr, int x, int y, int h, uint16_t color) {
  int r = 1;
  spr.drawRect(x - r, y + h / 3 - r, 3, 3, color);
  spr.drawRect(x - r, y + (h * 2) / 3 - r, 3, 3, color);
}

// 文字描画
void drawWireframeChar(TFT_eSprite& spr, int x, int y, char c, int w, int h, uint16_t color) {
  int hh = h / 2;

  switch (c) {
    case 'C':
      spr.drawFastHLine(x, y, w, color);
      spr.drawFastVLine(x, y, h, color);
      spr.drawFastHLine(x, y + h, w, color);
      break;
    case 'Y':
      spr.drawLine(x, y, x + w / 2, y + hh, color);
      spr.drawLine(x + w, y, x + w / 2, y + hh, color);
      spr.drawFastVLine(x + w / 2, y + hh, hh, color);
      break;
    case 'B':
      spr.drawFastVLine(x, y, h, color);
      spr.drawFastHLine(x, y, w - 1, color);
      spr.drawFastHLine(x, y + hh, w - 1, color);
      spr.drawFastHLine(x, y + h, w - 1, color);
      spr.drawFastVLine(x + w - 1, y, hh, color);
      spr.drawFastVLine(x + w - 1, y + hh, hh, color);
      break;
    case 'E':
      spr.drawFastVLine(x, y, h, color);
      spr.drawFastHLine(x, y, w, color);
      spr.drawFastHLine(x, y + hh, w - 2, color);
      spr.drawFastHLine(x, y + h, w, color);
      break;
    case 'R':
      spr.drawFastVLine(x, y, h, color);
      spr.drawFastHLine(x, y, w - 1, color);
      spr.drawFastHLine(x, y + hh, w - 1, color);
      spr.drawFastVLine(x + w - 1, y, hh, color);
      spr.drawLine(x + 2, y + hh, x + w, y + h, color);
      break;
    case '-':
      spr.drawFastHLine(x, y + hh, w, color);
      break;
    case 'O':
      spr.drawRect(x, y, w, h, color);
      break;
    case 'S':
      spr.drawFastHLine(x, y, w, color);
      spr.drawFastVLine(x, y, hh, color);
      spr.drawFastHLine(x, y + hh, w, color);
      spr.drawFastVLine(x + w - 1, y + hh, hh, color);
      spr.drawFastHLine(x, y + h, w, color);
      break;
    case 'T':
      spr.drawFastHLine(x, y, w, color);
      spr.drawFastVLine(x + w / 2, y, h, color);
      break;
    case 'M':
      spr.drawFastVLine(x, y, h, color);
      spr.drawFastVLine(x + w - 1, y, h, color);
      spr.drawLine(x, y, x + w / 2, y + hh, color);
      spr.drawLine(x + w - 1, y, x + w / 2, y + hh, color);
      break;
    case '3':
      drawWireframeDigit(spr, x, y, 3, w, h, color);
      break;
    case 'D':
      spr.drawFastVLine(x, y, h, color);
      spr.drawFastHLine(x, y, w - 2, color);
      spr.drawFastHLine(x, y + h, w - 2, color);
      spr.drawLine(x + w - 2, y, x + w, y + 2, color);
      spr.drawFastVLine(x + w, y + 2, h - 4, color);
      spr.drawLine(x + w, y + h - 2, x + w - 2, y + h, color);
      break;
    default:
      break;
  }
}

void drawWireframeString(TFT_eSprite& spr, int startX, int startY, const char* str, int charW, int charH, int spacing, uint16_t color) {
  int curX = startX;
  for (int i = 0; str[i] != '\0'; i++) {
    if (str[i] != ' ') {
      drawWireframeChar(spr, curX, startY, str[i], charW, charH, color);
    }
    curX += charW + spacing;
  }
}

// エンティティ生成
void initHistory(Entity3D &e, const Vec3& p) {
  for(int i=0; i<HISTORY_LEN; i++) e.hist[i] = p;
  e.histIdx = 0; e.flash = 0; e.infected = false; e.targetId = -1; e.age = 0;
}

void updateHistory(Entity3D &e) {
  e.hist[e.histIdx] = e.pos;
  e.histIdx = (e.histIdx + 1) & HISTORY_LEN_MASK;
}

void updateEntityCommon(Entity3D& e) {
  updateHistory(e);
  e.age++;
  if (e.flash > 0.0f) e.flash *= 0.85f;
}

void initEntityBase(Entity3D& e, const Vec3& p, float pSpeed, float energy, float radius, float speedLimitMin, float speedLimitMax, float exactSpeedLimit = -1.0f) {
  e.active = true;
  if (p.x == -999) {
    e.pos = Vec3(random((int)WORLD_MIN_X + 10, (int)WORLD_MAX_X - 10),
                 random((int)WORLD_MIN_Y + 10, (int)WORLD_MAX_Y - 10),
                 random((int)WORLD_MIN_Z + 20, (int)WORLD_MAX_Z - 20));
  } else {
    e.pos = p + Vec3(random(-5, 6), random(-5, 6), random(-5, 6));
  }
  e.vel = Vec3((random(-100, 101) * 0.01f), (random(-100, 101) * 0.01f), (random(-100, 101) * 0.01f)).normalized() * pSpeed;
  e.energy = energy;
  e.radius = radius;
  e.speedLimit = (exactSpeedLimit > 0.0f) ? exactSpeedLimit : constrain(pSpeed + (random(-100, 101) * 0.001f), speedLimitMin, speedLimitMax);
  initHistory(e, e.pos);
}

void applyBoundingBox(Vec3& pos, Vec3& vel) {
  if(pos.x < WORLD_MIN_X) { pos.x = WORLD_MIN_X; vel.x *= -1; }
  else if(pos.x > WORLD_MAX_X) { pos.x = WORLD_MAX_X; vel.x *= -1; }
  if(pos.y < WORLD_MIN_Y) { pos.y = WORLD_MIN_Y; vel.y *= -1; }
  else if(pos.y > WORLD_MAX_Y) { pos.y = WORLD_MAX_Y; vel.y *= -1; }
  if(pos.z < WORLD_MIN_Z) { pos.z = WORLD_MIN_Z; vel.z *= -1; }
  else if(pos.z > WORLD_MAX_Z) { pos.z = WORLD_MAX_Z; vel.z *= -1; }
}

template <typename T>
int findClosestTarget(const Vec3& pos, T* targets, int maxTargets, float& outMinDist) {
  float minDist = 999999.0f;
  int targetId = -1;
  for (int i = 0; i < maxTargets; i++) {
    if (targets[i].active) {
      float dist = (targets[i].pos - pos).lengthSq();
      if (dist < minDist) {
        minDist = dist;
        targetId = i;
      }
    }
  }
  outMinDist = minDist;
  return targetId;
}

void spawnDeathEffects(const Vec3& pos, uint8_t r, uint8_t g, uint8_t b, int pCount, float pSpeed) {
  spawnExplosion(pos, r, g, b, pCount, pSpeed);
  spawnGarbage(pos, r, g, b);
}

void spawnShockwave(const Vec3& pos, float maxR, uint8_t r, uint8_t g, uint8_t b) {
  for(int w=0; w<MAX_WAVES; w++) {
    if(!shockwaves[w].active) {
      shockwaves[w].active = true;
      shockwaves[w].pos = pos;
      shockwaves[w].radius = 1.0f;
      shockwaves[w].maxRadius = maxR;
      shockwaves[w].life = 1.0f;
      shockwaves[w].r = r; shockwaves[w].g = g; shockwaves[w].b = b;
      break;
    }
  }
}

void spawnExplosion(const Vec3& pos, uint8_t r, uint8_t g, uint8_t b, int count, float speedBase) {
  spawnShockwave(pos, 18.0f, r, g, b);
  int spawnCount = count * 2;
  for(int p=0; p<MAX_PARTICLES && spawnCount > 0; p++) {
    if(!particles[p].active) {
      particles[p].active = true;
      particles[p].pos = pos;
      float theta = random(0, 360) * (PI / 180.0f);
      float phi = random(0, 180) * (PI / 180.0f);
      float speed = (random(8, 35) * 0.1f) * speedBase;
      particles[p].vel = Vec3(sinf(phi)*cosf(theta), sinf(phi)*sinf(theta), cosf(phi)) * speed;
      particles[p].life = 1.0f;
      particles[p].r = r; particles[p].g = g; particles[p].b = b;
      spawnCount--;
    }
  }
}

void spawnGarbage(const Vec3& pos, uint8_t r, uint8_t g, uint8_t b) {
  garbages[garbageIdx].active = true;
  garbages[garbageIdx].pos = pos + Vec3(random(-4, 5), random(-4, 5), random(-4, 5));
  garbages[garbageIdx].r = r; garbages[garbageIdx].g = g; garbages[garbageIdx].b = b;
  garbageIdx = (garbageIdx + 1) % MAX_GARBAGES;
}

bool spawnPlant() {
  for(int i=0; i<MAX_PLANTS; i++) {
    if(!plants[i].active) {
      plants[i].active = true;
      plants[i].pos = Vec3(random((int)WORLD_MIN_X + 5, (int)WORLD_MAX_X - 5),
                           random((int)WORLD_MIN_Y + 5, (int)WORLD_MAX_Y - 5),
                           random((int)WORLD_MIN_Z + 10, (int)WORLD_MAX_Z - 10));
      plants[i].radius = random(12, 23) * 0.1f;
      return true;
    }
  }
  return false;
}

bool spawnHerb(Vec3 p = Vec3(-999, -999, -999), float pSpeed = 1.4f, float pAltruism = -1.0f, float pImmunity = -1.0f) {
  for(int i=0; i<MAX_HERBS; i++) {
    if(!herbs[i].active) {
      initEntityBase(herbs[i], p, pSpeed, 80.0f, 4.5f, 0.6f, 3.0f);
      herbs[i].altruism = (pAltruism == -1.0f) ? (random(0, 100) * 0.01f) : constrain(pAltruism + random(-10, 11)*0.01f, 0.0f, 1.0f);
      herbs[i].immunity = (pImmunity == -1.0f) ? (random(0, 100) * 0.01f) : constrain(pImmunity + random(-10, 11)*0.01f, 0.0f, 1.0f);
      activeHerbCount++;
      return true;
    }
  }
  return false;
}

bool spawnCarn(Vec3 p = Vec3(-999, -999, -999), float pSpeed = 1.8f) {
  for(int i=0; i<MAX_CARNS; i++) {
    if(!carns[i].active) {
      initEntityBase(carns[i], p, pSpeed, 100.0f, 6.0f, 1.0f, 3.5f);
      activeCarnCount++;
      return true;
    }
  }
  return false;
}

bool spawnApex(Vec3 p = Vec3(-999, -999, -999), float pSpeed = 2.2f) {
  for(int i=0; i<MAX_APEX; i++) {
    if(!apex[i].active) {
      initEntityBase(apex[i], p, pSpeed, 300.0f, 8.0f, 1.5f, 4.0f);
      activeApexCount++;
      return true;
    }
  }
  return false;
}

bool spawnSpore(const Vec3& p) {
  for(int i=0; i<MAX_SPORES; i++) {
    if(!spores[i].active) {
      spores[i].active = true;
      spores[i].pos = p;
      spores[i].vel = Vec3((random(-50, 51) * 0.01f), (random(-50, 51) * 0.01f), (random(-50, 51) * 0.01f));
      return true;
    }
  }
  return false;
}

bool spawnDecomp(Vec3 p = Vec3(-999, -999, -999)) {
  for(int i=0; i<MAX_DECOMPS; i++) {
    if(!decomps[i].active) {
      initEntityBase(decomps[i], p, 1.0f, 80.0f, 4.0f, 0.0f, 0.0f, 1.2f);
      activeDecompCount++;
      return true;
    }
  }
  return false;
}

// 各エンティティの更新処理関数
void updatePlanktons() {
  for(int i=0; i<MAX_PLANKTON; i++) {
    float zFactor = getDepthSpeedFactor(planktons[i].pos.z);
    planktons[i].pos += planktons[i].vel * zFactor;
    if(planktons[i].pos.x > WORLD_MAX_X) planktons[i].pos.x = WORLD_MIN_X;
    if(planktons[i].pos.x < WORLD_MIN_X) planktons[i].pos.x = WORLD_MAX_X;
  }
}

void updateShockwaves() {
  for(int w=0; w<MAX_WAVES; w++) {
    if(shockwaves[w].active) {
      shockwaves[w].radius += 0.8f;
      shockwaves[w].life -= 0.05f;
      if(shockwaves[w].radius >= shockwaves[w].maxRadius || shockwaves[w].life <= 0) {
        shockwaves[w].active = false;
      }
    }
  }
}

void applyTouchAttraction() {
  if (isFingerTouching) {
    auto applyTouchAttractor = [&](Entity3D &e) {
      if (!e.active) return;
      Vec3 diff = fingerWorldPos - e.pos;
      float distSq = diff.lengthSq();
      if (distSq > 4.0f) {
        float invDist = Q_rsqrt(distSq);
        e.vel = e.vel * 0.80f + diff * (invDist * 1.35f);
      }
    };
    for (int i = 0; i < MAX_HERBS; i++) applyTouchAttractor(herbs[i]);
    for (int i = 0; i < MAX_CARNS; i++) applyTouchAttractor(carns[i]);
    for (int i = 0; i < MAX_APEX; i++) applyTouchAttractor(apex[i]);
    for (int i = 0; i < MAX_DECOMPS; i++) applyTouchAttractor(decomps[i]);
  }
}

void updateParticles() {
  for(int p=0; p<MAX_PARTICLES; p++) {
    if(particles[p].active) {
      float zFactor = getDepthSpeedFactor(particles[p].pos.z);
      particles[p].pos += particles[p].vel * zFactor;
      particles[p].vel *= 0.92f;
      particles[p].life -= 0.035f;
      if(particles[p].life <= 0) particles[p].active = false;
    }
  }
}

void updateDecomps() {
  for(int i=0; i<MAX_DECOMPS; i++) {
    if(!decomps[i].active) continue;
    updateEntityCommon(decomps[i]);
    
    float minDist = 999999.0f;
    int targetG = -1, targetS = -1;

    for(int g=0; g<MAX_GARBAGES; g++) {
      if(garbages[g].active) {
        float dist = (garbages[g].pos - decomps[i].pos).lengthSq();
        if(dist < minDist) { minDist = dist; targetG = g; targetS = -1; }
      }
    }
    for(int s=0; s<MAX_SPORES; s++) {
      if(spores[s].active) {
        float dist = (spores[s].pos - decomps[i].pos).lengthSq();
        if(dist < minDist) { minDist = dist; targetS = s; targetG = -1; }
      }
    }

    if(targetG != -1) {
      Vec3 diff = garbages[targetG].pos - decomps[i].pos;
      decomps[i].vel = (decomps[i].vel * 0.95f) + (diff.normalized(minDist) * 0.18f);
      if(minDist < 36.0f) { garbages[targetG].active = false; decomps[i].energy += 25; decomps[i].flash = 1.0f; }
    } else if(targetS != -1) {
      Vec3 diff = spores[targetS].pos - decomps[i].pos;
      decomps[i].vel = (decomps[i].vel * 0.95f) + (diff.normalized(minDist) * 0.18f);
      if(minDist < 30.0f) { spores[targetS].active = false; decomps[i].energy += 25; decomps[i].flash = 1.0f; }
    } else {
      decomps[i].vel += Vec3(random(-50, 51)*0.002f, random(-50, 51)*0.002f, random(-50, 51)*0.002f);
    }
    
    // 風の影響
    decomps[i].vel += getAttractorForce(decomps[i].pos) * 0.02f;

    // 速度制限
    decomps[i].vel = decomps[i].vel.limitedTo(decomps[i].speedLimit);

    float zSpeedFactor = getDepthSpeedFactor(decomps[i].pos.z);
    decomps[i].pos += decomps[i].vel * zSpeedFactor;
    
    applyBoundingBox(decomps[i].pos, decomps[i].vel);
    
    decomps[i].energy -= 0.02f;
    if(decomps[i].energy <= 0) {
      decomps[i].active = false;
      activeDecompCount--;
      spawnDeathEffects(decomps[i].pos, 100, 255, 100, 8, 0.6f);
    } else if(decomps[i].energy > 130) {
      if (spawnPlant()) {
        decomps[i].energy -= 70;
      }
    }
  }
}

void updateSpores() {
  for(int i=0; i<MAX_SPORES; i++) {
    if(!spores[i].active) continue;
    float zSpeedFactor = getDepthSpeedFactor(spores[i].pos.z);
    spores[i].pos += spores[i].vel * zSpeedFactor;
    applyBoundingBox(spores[i].pos, spores[i].vel);
    
    for(int h=0; h<MAX_HERBS; h++) {
      if(herbs[h].active && !herbs[h].infected) {
        if((herbs[h].pos - spores[i].pos).lengthSq() < 36.0f) { 
          spores[i].active = false;
          if (random(0, 100) >= herbs[h].immunity * 100.0f) {
            herbs[h].infected = true; herbs[h].flash = 1.0f;
            spawnExplosion(herbs[h].pos, 180, 0, 255, 10, 1.0f);
          }
          break;
        }
      }
    }
  }
}

void updateHerbs() {
  for(int i=0; i<MAX_HERBS; i++) {
    if(!herbs[i].active) continue;
    updateEntityCommon(herbs[i]);
    
    Vec3 align(0,0,0), coh(0,0,0);
    int flockCount = 0;
    herbs[i].targetId = -1;

    for(int j=0; j<MAX_HERBS; j++) {
      if (i != j && herbs[j].active) {
        Vec3 diff = herbs[j].pos - herbs[i].pos;
        float distSq = diff.lengthSq();
        if (distSq < 1600.0f) { 
          align += herbs[j].vel;
          coh += herbs[j].pos;
          flockCount++;
          if (distSq < 144.0f && distSq > 0.01f) {
            // 除算と正規化の高速化
            herbs[i].vel -= diff.normalized(distSq) * (40.0f * Q_rcp(distSq));
          }
          
          // 利他行動
          if (distSq < 400.0f && !herbs[i].infected && !herbs[j].infected) {
            if (herbs[i].energy > 60.0f && herbs[j].energy < 30.0f) {
              if ((random(0, 100) * 0.01f) < herbs[i].altruism) {
                herbs[i].energy -= 1.0f;
                herbs[j].energy += 1.0f;
              }
            }
          }
        }
      }
    }
    if (flockCount > 0 && !herbs[i].infected) { 
      // 正規化
      align = align.normalized();
      herbs[i].vel += align * 0.06f;
      // 中心方向
      coh = ((coh / (float)flockCount) - herbs[i].pos).normalized();
      herbs[i].vel += coh * 0.03f;
    }

    float minDist = 999999.0f;
    int target = findClosestTarget(herbs[i].pos, plants, MAX_PLANTS, minDist);
    if(target != -1) {
      herbs[i].targetId = target;
      Vec3 diff = plants[target].pos - herbs[i].pos;
      if (!herbs[i].infected) {
        herbs[i].vel = (herbs[i].vel * 0.96f) + (diff.normalized(minDist) * 0.1f);
      }
      if(minDist < 25.0f) { 
        plants[target].active = false;
        herbs[i].energy += 30; herbs[i].flash = 1.0f; 
        spawnExplosion(plants[target].pos, 150, 255, 150, 8, 0.6f);
      }
    }

    // 肉食動物回避・おとり行動
    for(int c=0; c<MAX_CARNS; c++) {
      if(carns[c].active) {
        Vec3 diff = herbs[i].pos - carns[c].pos;
        float distSq = diff.lengthSq();
        if(distSq < 3600.0f && distSq > 0.01f) { 
          Vec3 normDiff = diff.normalized(distSq);
          if (herbs[i].altruism > 0.65f && !herbs[i].infected) {
            herbs[i].vel -= normDiff * 0.15f; 
          } else {
            herbs[i].vel += normDiff * 0.25f;
          }
        }
      }
    }
    
    // 感染隔離
    if (herbs[i].infected) {
      if (herbs[i].altruism > 0.6f) {
        float edge_x = (herbs[i].pos.x < 0.0f) ? -1.0f : 1.0f;
        float edge_y = (herbs[i].pos.y < 0.0f) ? -1.0f : 1.0f;
        float edge_z = (herbs[i].pos.z < 150.0f) ? -1.0f : 1.0f;
        herbs[i].vel.x = herbs[i].vel.x * 0.9f + edge_x * 0.1f;
        herbs[i].vel.y = herbs[i].vel.y * 0.9f + edge_y * 0.1f;
        herbs[i].vel.z = herbs[i].vel.z * 0.9f + edge_z * 0.1f;
      } else {
        herbs[i].vel.x += random(-50, 51) * 0.01f;
        herbs[i].vel.y += random(-50, 51) * 0.01f;
        herbs[i].vel.z += random(-50, 51) * 0.01f;
      }
    }
    
    // 風の影響
    herbs[i].vel += getAttractorForce(herbs[i].pos) * 0.04f;

    float speedLimit = herbs[i].infected ? herbs[i].speedLimit + 0.5f : herbs[i].speedLimit;
    // 速度制限
    herbs[i].vel = herbs[i].vel.limitedTo(speedLimit);

    float zSpeedFactor = getDepthSpeedFactor(herbs[i].pos.z);
    herbs[i].pos += herbs[i].vel * zSpeedFactor;
    
    applyBoundingBox(herbs[i].pos, herbs[i].vel);
    
    herbs[i].energy -= 0.02f;
    if(herbs[i].energy <= 0) {
      herbs[i].active = false;
      activeHerbCount--;
      spawnDeathEffects(herbs[i].pos, 0, 255, 255, 8, 0.7f);
    } else if (herbs[i].energy > 160 && !herbs[i].infected) {
      if (spawnHerb(herbs[i].pos, herbs[i].speedLimit, herbs[i].altruism, herbs[i].immunity)) {
        herbs[i].energy -= 80;
      }
    }
  }
}

void updateCarns() {
  for(int i=0; i<MAX_CARNS; i++) {
    if(!carns[i].active) continue;
    updateEntityCommon(carns[i]);
    
    float minDist = 999999.0f;
    carns[i].targetId = findClosestTarget(carns[i].pos, herbs, MAX_HERBS, minDist);
    
    if(carns[i].targetId != -1) {
      Vec3 diff = herbs[carns[i].targetId].pos - carns[i].pos;
      carns[i].vel = (carns[i].vel * 0.97f) + (diff.normalized(minDist) * 0.14f);
      if(minDist < 49.0f) { 
        herbs[carns[i].targetId].energy -= 4.5f; 
        carns[i].energy += 4.5f; 
        carns[i].flash = 1.0f; 
      }
    }

    // 風の影響
    carns[i].vel += getAttractorForce(carns[i].pos) * 0.06f;

    // 速度制限
    carns[i].vel = carns[i].vel.limitedTo(carns[i].speedLimit);

    float zSpeedFactor = getDepthSpeedFactor(carns[i].pos.z);
    carns[i].pos += carns[i].vel * zSpeedFactor;
    
    applyBoundingBox(carns[i].pos, carns[i].vel);
    
    carns[i].energy -= 0.12f;
    if(carns[i].energy <= 0) {
      carns[i].active = false;
      activeCarnCount--;
      spawnDeathEffects(carns[i].pos, 255, 50, 150, 12, 0.9f);
    } else if (carns[i].energy > 150) {
      if (spawnCarn(carns[i].pos, carns[i].speedLimit)) {
        carns[i].energy -= 60;
      }
    }
  }
}

void updateApex() {
  for(int i=0; i<MAX_APEX; i++) {
    if(!apex[i].active) continue;
    updateEntityCommon(apex[i]);
    
    float minDist = 999999.0f;
    apex[i].targetId = findClosestTarget(apex[i].pos, carns, MAX_CARNS, minDist);
    
    if(apex[i].targetId != -1) {
      Vec3 diff = carns[apex[i].targetId].pos - apex[i].pos;
      apex[i].vel = (apex[i].vel * 0.97f) + (diff.normalized(minDist) * 0.18f);
      if(minDist < 64.0f) { 
        carns[apex[i].targetId].energy -= 5.0f; 
        apex[i].energy += 5.0f;
        apex[i].flash = 1.0f;
      }
    }
    
    // 風の影響
    apex[i].vel += getAttractorForce(apex[i].pos) * 0.08f;

    // 速度制限
    apex[i].vel = apex[i].vel.limitedTo(apex[i].speedLimit);

    float zSpeedFactor = getDepthSpeedFactor(apex[i].pos.z);
    apex[i].pos += apex[i].vel * zSpeedFactor;
    
    applyBoundingBox(apex[i].pos, apex[i].vel);
    
    apex[i].energy -= 0.25f; 
    if(apex[i].energy <= 0) {
      apex[i].active = false;
      activeApexCount--;
      spawnDeathEffects(apex[i].pos, 255, 215, 0, 18, 1.2f);
    } else if (apex[i].energy > 550) { 
      if (spawnApex(apex[i].pos, apex[i].speedLimit)) {
        apex[i].energy -= 220; 
      }
    }
  }
}

// 物理演算スレッド
void core0Task(void * pvParameters) {
  for(;;) {
    unsigned long startMicros = micros();

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    // アクティブ数を取得
    int herbCount = activeHerbCount;
    int carnCount = activeCarnCount;
    int apexCount = activeApexCount;
    int decompCount = activeDecompCount;

    if (random(0, 1000) < 40) spawnPlant();
    if (random(0, 10000) < 50) spawnSpore(Vec3(random((int)WORLD_MIN_X, (int)WORLD_MAX_X), random((int)WORLD_MIN_Y, (int)WORLD_MAX_Y), random((int)WORLD_MIN_Z, (int)WORLD_MAX_Z)));

    if(decompCount < 5) spawnDecomp();
    if(herbCount < 8 && random(0, 1000) < 20) spawnHerb();
    if(carnCount == 0 && herbCount > 10 && random(0, 1000) < 30) spawnCarn();
    if(apexCount == 0 && carnCount > 3 && random(0, 1000) < 20) spawnApex();

    updatePlanktons();
    updateShockwaves();
    applyTouchAttraction();
    updateParticles();
    updateDecomps();
    updateSpores();
    updateHerbs();
    updateCarns();
    updateApex();
    
    xSemaphoreGive(dataMutex);

    // 60FPS制御
    static unsigned long nextFrameMicros0 = 0;
    if (nextFrameMicros0 == 0) nextFrameMicros0 = micros() + FRAME_TIME_US;

    unsigned long nowUs = micros();
    if ((long)(nextFrameMicros0 - nowUs) > 0) {
      delayMicroseconds(nextFrameMicros0 - nowUs);
    }
    nextFrameMicros0 += FRAME_TIME_US;

    // WDT対策
    static int frameCounter0 = 0;
    if (++frameCounter0 >= 60) {
      vTaskDelay(1);
      frameCounter0 = 0;
      nextFrameMicros0 = micros() + FRAME_TIME_US; // ズレ吸収
    }
  }
}

// 3D射影(invZ計算込み)
inline bool project3D(const Vec3& p, int& sx, int& sy, float& outInvZ) {
  if (p.z <= 10.0f) return false;
  // 高速近似逆数
  float invZ = 220.0f * Q_rcp(p.z);
  outInvZ = invZ;
  float projX = p.x * invZ;
  float projY = p.y * invZ;

  sx = (int)(projX + (TFT_WIDTH / 2));
  sy = (int)(projY + (TFT_HEIGHT / 2));
  return (sx >= -60 && sx < TFT_WIDTH + 60 && sy >= -60 && sy < TFT_HEIGHT + 60);
}

inline bool project3D(const Vec3& p, int& sx, int& sy) {
  float unusedInvZ;
  return project3D(p, sx, sy, unusedInvZ);
}

inline uint16_t depthFadeColor(uint8_t r, uint8_t g, uint8_t b, float z, float flashIntensity = 0.0f) {
  // 深度減衰カーブ
  float normZ = constrain((z - VISUAL_MIN_Z) * VISUAL_Z_RANGE_INV, 0.0f, 1.0f);
  float depthFactor = constrain(1.0f - (normZ * normZ * 0.8f), 0.2f, 1.0f);
  uint8_t fr = (uint8_t)constrain(r * depthFactor + (255.0f * flashIntensity), 0.0f, 255.0f);
  uint8_t fg = (uint8_t)constrain(g * depthFactor + (255.0f * flashIntensity), 0.0f, 255.0f);
  uint8_t fb = (uint8_t)constrain(b * depthFactor + (255.0f * flashIntensity), 0.0f, 255.0f);
  return packRGB565(fr, fg, fb);
}

// 3Dメッシュ球描画
bool drawMeshLatticeSphere(const Vec3& center, float radius, uint8_t r, uint8_t g, uint8_t b, float flash = 0.0f, int* outSx = nullptr, int* outSy = nullptr) {
  int cx, cy;
  float invZ; // 再計算防止
  if (!project3D(center, cx, cy, invZ)) return false;
  if (outSx) *outSx = cx;
  if (outSy) *outSy = cy;

  uint16_t color = depthFadeColor(r, g, b, center.z, flash);
  float projectedRadius = radius * invZ;
  
  if (projectedRadius < 1.2f) {
    img.drawPixel(cx, cy, color);
    return true;
  }

  img.drawCircle(cx, cy, (int)projectedRadius, color);

  int rx = (int)projectedRadius;
  int ry1 = max(1, (int)(projectedRadius * 0.70f));
  int ry2 = max(1, (int)(projectedRadius * 0.35f));

  if (projectedRadius > 3.0f) {
    img.drawEllipse(cx, cy, rx, ry1, color);
    img.drawEllipse(cx, cy, ry1, rx, color);
  }
  if (projectedRadius > 6.0f) { // 詳細ラティス描画
    img.drawEllipse(cx, cy, rx, ry2, color);
    img.drawEllipse(cx, cy, ry2, rx, color);
  }
  return true;
}



// ワイヤーフレーム尾
void drawTaperedWireframeTail3D(const Entity3D& e, uint8_t r, uint8_t g, uint8_t b, float headWidth) {
  if (!e.active) return;

  Vec3 prevPos = e.pos;
  int prevSx, prevSy;
  float invZPrev; // 計算済みinvZ
  if (!project3D(prevPos, prevSx, prevSy, invZPrev)) return;

  for (int h = 0; h < HISTORY_LEN - 1; h++) {
    int idx = (e.histIdx - 1 - h + HISTORY_LEN) & HISTORY_LEN_MASK;
    Vec3 currPos = e.hist[idx];
    if (currPos.z < 10.0f) continue;

    int currSx, currSy;
    float invZCurr; // 計算済みinvZ
    if (!project3D(currPos, currSx, currSy, invZCurr)) continue;

    float taperPrev = 1.0f - ((float)h / (float)HISTORY_LEN);
    float taperCurr = 1.0f - ((float)(h + 1) / (float)HISTORY_LEN);

    float wPrev = max(0.6f, headWidth * taperPrev * invZPrev);
    float wCurr = max(0.4f, headWidth * taperCurr * invZCurr);

    uint16_t cPrev = depthFadeColor((uint8_t)(r * taperPrev), (uint8_t)(g * taperPrev), (uint8_t)(b * taperPrev), prevPos.z, e.flash * taperPrev);

    float dx = (float)(currSx - prevSx);
    float dy = (float)(currSy - prevSy);
    float lenSq = dx * dx + dy * dy;

    if (lenSq > 0.01f) {
      float invLen = Q_rsqrt(lenSq);
      float nx = -dy * invLen;
      float ny =  dx * invLen;

      int p1x_L = (int)(prevSx + nx * wPrev);
      int p1y_L = (int)(prevSy + ny * wPrev);
      int p1x_R = (int)(prevSx - nx * wPrev);
      int p1y_R = (int)(prevSy - ny * wPrev);

      int p2x_L = (int)(currSx + nx * wCurr);
      int p2y_L = (int)(currSy + ny * wCurr);
      int p2x_R = (int)(currSx - nx * wCurr);
      int p2y_R = (int)(currSy - ny * wCurr);

      img.drawLine(p1x_L, p1y_L, p2x_L, p2y_L, cPrev);
      img.drawLine(p1x_R, p1y_R, p2x_R, p2y_R, cPrev);
      img.drawLine(prevSx, prevSy, currSx, currSy, cPrev);

      if (h % 3 == 0 && wPrev > 1.2f) {
        img.drawLine(p1x_L, p1y_L, p1x_R, p1y_R, cPrev);
      }
    } else {
      img.drawLine(prevSx, prevSy, currSx, currSy, cPrev);
    }

    prevPos = currPos;
    prevSx = currSx;
    prevSy = currSy;
    invZPrev = invZCurr;
  }
}

struct RenderItem {
  uint8_t type;
  uint16_t index;
  float z;
  int16_t sx, sy; // 画面座標
};

void renderWireframeScene() {
  img.fillSprite(TFT_BLACK);

  // staticバッファ確保
  static RenderItem items[MAX_TOTAL_ENTITIES];
  int count = 0;

  // 画面外カリング
  auto pushIfVisible = [&](uint8_t type, uint16_t idx, const Vec3& pos) {
    int sx, sy;
    if (project3D(pos, sx, sy)) {
      items[count++] = {type, idx, pos.z, (int16_t)sx, (int16_t)sy};
    }
  };

  for(int i=0; i<MAX_PLANKTON; i++) {
    pushIfVisible(0, (uint16_t)i, snap_planktons[i].pos);
  }
  for(int i=0; i<MAX_PLANTS; i++) {
    if(snap_plants[i].active) pushIfVisible(1, (uint16_t)i, snap_plants[i].pos);
  }
  for(int i=0; i<MAX_GARBAGES; i++) {
    if(snap_garbages[i].active) pushIfVisible(2, (uint16_t)i, snap_garbages[i].pos);
  }
  for(int i=0; i<MAX_DECOMPS; i++) {
    if(snap_decomps[i].active) pushIfVisible(3, (uint16_t)i, snap_decomps[i].pos);
  }
  for(int i=0; i<MAX_SPORES; i++) {
    if(snap_spores[i].active) pushIfVisible(4, (uint16_t)i, snap_spores[i].pos);
  }
  for(int i=0; i<MAX_HERBS; i++) {
    if(snap_herbs[i].active) pushIfVisible(5, (uint16_t)i, snap_herbs[i].pos);
  }
  for(int i=0; i<MAX_CARNS; i++) {
    if(snap_carns[i].active) pushIfVisible(6, (uint16_t)i, snap_carns[i].pos);
  }
  for(int i=0; i<MAX_APEX; i++) {
    if(snap_apex[i].active) pushIfVisible(7, (uint16_t)i, snap_apex[i].pos);
  }
  for(int i=0; i<MAX_PARTICLES; i++) {
    if(snap_particles[i].active) pushIfVisible(8, (uint16_t)i, snap_particles[i].pos);
  }
  for(int w=0; w<MAX_WAVES; w++) {
    if(snap_shockwaves[w].active) pushIfVisible(9, (uint16_t)w, snap_shockwaves[w].pos);
  }

  // Z降順ソート
  std::sort(items, items + count, [](const RenderItem& a, const RenderItem& b) {
    return a.z > b.z;
  });

  // 描画
  for(int k=0; k<count; k++) {
    int type = items[k].type;
    int idx = items[k].index;
    int cachedSx = items[k].sx, cachedSy = items[k].sy; // 計算済座標

    if (type == 0) {
      uint16_t pCol;
      if (snap_planktons[idx].layer == 1) pCol = depthFadeColor(25, 45, 80, snap_planktons[idx].pos.z);
      else if (snap_planktons[idx].layer == 2) pCol = depthFadeColor(45, 90, 140, snap_planktons[idx].pos.z);
      else pCol = depthFadeColor(80, 160, 220, snap_planktons[idx].pos.z);
      img.drawPixel(cachedSx, cachedSy, pCol);
    } else if (type == 1) {
      int sx, sy;
      if (drawMeshLatticeSphere(snap_plants[idx].pos, snap_plants[idx].radius, 40, 220, 60, 0.0f, &sx, &sy)) {
        img.drawPixel(sx, sy, TFT_WHITE);
      }
    } else if (type == 2) {
      uint16_t gc = depthFadeColor(snap_garbages[idx].r, snap_garbages[idx].g, snap_garbages[idx].b, snap_garbages[idx].pos.z);
      img.drawLine(cachedSx-2, cachedSy-2, cachedSx+2, cachedSy+2, gc);
      img.drawLine(cachedSx+2, cachedSy-2, cachedSx-2, cachedSy+2, gc);
    } else if (type == 3) {
      drawTaperedWireframeTail3D(snap_decomps[idx], 160, 255, 60, 2.5f);
      drawMeshLatticeSphere(snap_decomps[idx].pos, snap_decomps[idx].radius, 160, 255, 60, snap_decomps[idx].flash);
    } else if (type == 4) {
      drawMeshLatticeSphere(snap_spores[idx].pos, 1.5f, 255, 100, 255);
    } else if (type == 5) {
      float ageFactor = min(snap_herbs[idx].age / 4000.0f, 1.0f);
      uint8_t r = (uint8_t)(0   + 50 * ageFactor);
      uint8_t g = (uint8_t)(230 + 25 * ageFactor);
      uint8_t b = (uint8_t)(255 - 180 * ageFactor);
      if (snap_herbs[idx].infected) {
        drawTaperedWireframeTail3D(snap_herbs[idx], 200, 0, 255, 3.0f);
        drawMeshLatticeSphere(snap_herbs[idx].pos, snap_herbs[idx].radius, 200, 0, 255, snap_herbs[idx].flash);
      } else {
        drawTaperedWireframeTail3D(snap_herbs[idx], r, g, b, 3.0f);
        drawMeshLatticeSphere(snap_herbs[idx].pos, snap_herbs[idx].radius, r, g, b, snap_herbs[idx].flash);
      }
    } else if (type == 6) {
      float ageFactor = min(snap_carns[idx].age / 4000.0f, 1.0f);
      uint8_t r = 255;
      uint8_t g = (uint8_t)(40 - 40 * ageFactor);
      uint8_t b = (uint8_t)(140 - 100 * ageFactor);
      drawTaperedWireframeTail3D(snap_carns[idx], r, g, b, 4.5f);
      drawMeshLatticeSphere(snap_carns[idx].pos, snap_carns[idx].radius, r, g, b, snap_carns[idx].flash);
    } else if (type == 7) {
      drawTaperedWireframeTail3D(snap_apex[idx], 255, 215, 0, 6.0f);
      drawMeshLatticeSphere(snap_apex[idx].pos, snap_apex[idx].radius, 255, 215, 0, snap_apex[idx].flash);
    } else if (type == 8) {
      uint16_t c = depthFadeColor((uint8_t)(snap_particles[idx].r * snap_particles[idx].life),
                                  (uint8_t)(snap_particles[idx].g * snap_particles[idx].life),
                                  (uint8_t)(snap_particles[idx].b * snap_particles[idx].life),
                                  snap_particles[idx].pos.z);
      img.drawPixel(cachedSx, cachedSy, c);
    } else if (type == 9) {
      float invZ; // 計算済みinvZ
      int wxTmp, wyTmp;
      project3D(snap_shockwaves[idx].pos, wxTmp, wyTmp, invZ);
      int rProj = (int)(snap_shockwaves[idx].radius * invZ);
      if(rProj > 0) {
        uint16_t wc = depthFadeColor((uint8_t)(snap_shockwaves[idx].r * snap_shockwaves[idx].life),
                                    (uint8_t)(snap_shockwaves[idx].g * snap_shockwaves[idx].life),
                                    (uint8_t)(snap_shockwaves[idx].b * snap_shockwaves[idx].life),
                                    snap_shockwaves[idx].pos.z);
        img.drawCircle(cachedSx, cachedSy, rProj, wc);
      }
    }
  }

  // 11. 最前面のタッチ波紋効果
  if (snap_isFingerTouching) {
    int fx, fy;
    if (project3D(snap_fingerWorldPos, fx, fy)) {
      float wave = (sinf(millis() * 0.008f) + 1.0f) * 0.5f;
      img.drawCircle(fx, fy, (int)(8 + wave * 6), TFT_WHITE);
      img.drawCircle(fx, fy, (int)(16 + wave * 10), packRGB565(0, 255, 255));
    }
  }
}

// HUD描画
void drawHUD() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastClockTick >= 1000) {
    unsigned long elapsed = (now - lastClockTick) / 1000;
    lastClockTick += elapsed * 1000;
    clockSec += elapsed;
    if (clockSec >= 60) {
      clockMin += clockSec / 60;
      clockSec %= 60;
      if (clockMin >= 60) {
        clockHour += clockMin / 60;
        clockMin %= 60;
        if (clockHour >= 24) {
          clockHour %= 24;
        }
      }
    }
  }

  static unsigned long lastTouchPoll = 0;
  if (now - lastTouchPoll > 30) {
    lastTouchPoll = now;
    if (ts.touched()) {
      TS_Point p = ts.getPoint();
      int tx = map(p.x, TOUCH_MIN_X, TOUCH_MAX_X, 0, 320);
      int ty = map(p.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, 240);

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      if (ty >= 30 && ty < 195) {
        isFingerTouching = true;
        float normX = ((float)tx / 320.0f - 0.5f) * 1.8f;
        float normY = ((float)(ty - OFFSET_Y) / 170.0f - 0.5f) * 1.8f;
        fingerWorldPos = Vec3(normX * 90.0f, normY * 50.0f, 70.0f);
      } else {
        isFingerTouching = false;
      }
      xSemaphoreGive(dataMutex);

      static unsigned long lastTouchBtnTime = 0;
      if (now - lastTouchBtnTime > 220) {
        if (ty >= 195) {
          lastTouchBtnTime = now;
          if (tx >= 165 && tx <= 210) {
            clockHour = (clockHour + 1) % 24;
          } else if (tx >= 215 && tx <= 260) {
            clockMin = (clockMin + 1) % 60;
          } else if (tx >= 265 && tx <= 315) {
            clockSec = 0;
          }
        }
      }
    } else {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      isFingerTouching = false;
      xSemaphoreGive(dataMutex);
    }
  }

  if (now - lastUpdate < 100) return;
  lastUpdate = now;

  int pCount = 0, hCount = 0, cCount = 0, aCount = 0;
  for(int i=0; i<MAX_PLANTS; i++) if(snap_plants[i].active) pCount++;
  for(int i=0; i<MAX_HERBS; i++) if(snap_herbs[i].active) hCount++;
  for(int i=0; i<MAX_CARNS; i++) if(snap_carns[i].active) cCount++;
  for(int i=0; i<MAX_APEX; i++) if(snap_apex[i].active) aCount++;

  histPlants[histCursor] = (uint8_t)pCount;
  histHerbs[histCursor] = (uint8_t)hCount;
  histCarns[histCursor] = (uint8_t)cCount;
  histApex[histCursor] = (uint8_t)aCount;
  histCursor = (histCursor + 1) % 320;

  topHud.fillSprite(TFT_BLACK);
  
  auto drawHist = [](uint8_t* hist, uint16_t color, int maxVal) {
    int prevX = -1, prevY = -1;
    // 乗算化
    float scale = 33.0f / maxVal;
    for(int i=0; i<320; i++) {
      // 剰余回避
      int idx = histCursor - 1 - i;
      if (idx < 0) idx += 320;
      int val = hist[idx];
      int y = 33 - (int)(val * scale);
      if (y < 0) y = 0;
      if (prevX != -1) {
        topHud.drawLine(prevX, prevY, i, y, color);
      }
      prevX = i; prevY = y;
    }
  };

  drawHist(histPlants, packRGB565(40, 220, 60), MAX_PLANTS);
  drawHist(histHerbs, packRGB565(0, 230, 255), MAX_HERBS);
  drawHist(histCarns, packRGB565(255, 40, 140), MAX_CARNS);
  drawHist(histApex, packRGB565(255, 215, 0), MAX_APEX);

  topHud.pushSprite(0, 0);

  // 7セグ時計描画
  static int lastSec = -1, lastMin = -1, lastHour = -1;
  if (clockSec != lastSec || clockMin != lastMin || clockHour != lastHour) {
    lastSec = clockSec; lastMin = clockMin; lastHour = clockHour;

    botHud.fillSprite(TFT_BLACK);
    uint16_t wireColor = packRGB565(0, 230, 255); // サイバーネオンブルー

    int h1 = clockHour / 10;
    int h2 = clockHour % 10;
    int m1 = clockMin / 10;
    int m2 = clockMin % 10;
    int s1 = clockSec / 10;
    int s2 = clockSec % 10;

    int dw = 11, dh = 19;
    int startX = 10, startY = 8;

    // 時
    drawWireframeDigit(botHud, startX, startY, h1, dw, dh, wireColor);
    drawWireframeDigit(botHud, startX + 15, startY, h2, dw, dh, wireColor);

    // コロン1
    drawWireframeColon(botHud, startX + 31, startY, dh, wireColor);

    // 分
    drawWireframeDigit(botHud, startX + 37, startY, m1, dw, dh, wireColor);
    drawWireframeDigit(botHud, startX + 52, startY, m2, dw, dh, wireColor);

    // コロン2
    drawWireframeColon(botHud, startX + 68, startY, dh, wireColor);

    // 秒
    drawWireframeDigit(botHud, startX + 74, startY, s1, dw, dh, wireColor);
    drawWireframeDigit(botHud, startX + 89, startY, s2, dw, dh, wireColor);

    // 操作ボタン
    botHud.drawRoundRect(165, 5, 42, 25, 4, wireColor);
    botHud.drawRoundRect(215, 5, 42, 25, 4, wireColor);
    botHud.drawRoundRect(265, 5, 46, 25, 4, wireColor);

    botHud.setTextDatum(MC_DATUM);
    botHud.setTextColor(wireColor, TFT_BLACK);
    botHud.setTextSize(1);
    botHud.drawString("H+", 186, 17, 1);
    botHud.drawString("M+", 236, 17, 1);
    botHud.drawString("00s", 288, 17, 1);

    botHud.pushSprite(0, 205);
  }

}

void setup() {
  randomSeed(esp_random()); // 乱数初期化
  initCydColorLUT(); // LUT初期化

  tft.init();
  tft.initDMA();
  tft.setRotation(1);
  tft.invertDisplay(true); 
  tft.fillScreen(TFT_BLACK);
  
  img.setColorDepth(16); 
  void* ptrImg = img.createSprite(TFT_WIDTH, TFT_HEIGHT);

  topHud.setColorDepth(16);
  void* ptrTop = topHud.createSprite(320, 34);

  botHud.setColorDepth(16);
  void* ptrBot = botHud.createSprite(320, 35);

  if (ptrImg == nullptr || ptrTop == nullptr || ptrBot == nullptr) {
    Serial.begin(115200);
    Serial.println("Sprite memory allocation failed!");
    while(1) { delay(100); } // 停止
  }

  touchSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchSpi);
  ts.setRotation(1);

  int ch = 0, cm = 0, cs = 0;
  if (sscanf(__TIME__, "%d:%d:%d", &ch, &cm, &cs) == 3) {
    clockHour = ch;
    clockMin  = cm;
    clockSec  = cs;
  }
  lastClockTick = millis();

  pinMode(CYD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(CYD_BACKLIGHT_PIN, HIGH);
  
  dataMutex = xSemaphoreCreateMutex();

  for(int i=0; i<35; i++) spawnPlant();
  for(int i=0; i<20; i++) spawnHerb();
  for(int i=0; i<3; i++) spawnCarn();
  
  for(int i=0; i<MAX_PLANKTON; i++) {
    planktons[i].pos = Vec3(random((int)WORLD_MIN_X, (int)WORLD_MAX_X), random((int)WORLD_MIN_Y, (int)WORLD_MAX_Y), random((int)WORLD_MIN_Z, (int)WORLD_MAX_Z));
    planktons[i].layer = random(1, 4);
    planktons[i].vel = Vec3((planktons[i].layer * random(5, 15)) * 0.025f, 0, 0); 
  }
  
  xTaskCreatePinnedToCore(core0Task, "Apex3DTask", 10000, NULL, 1, &Task1, 0); 
}

void loop() {
  // データコピー
  captureSnapshot();

  // 描画処理
  renderWireframeScene();



  img.pushSprite(0, OFFSET_Y);
  drawHUD();

  // 60FPS制御
  static unsigned long nextFrameMicros1 = 0;
  if (nextFrameMicros1 == 0) nextFrameMicros1 = micros() + FRAME_TIME_US;

  unsigned long nowUs = micros();
  if ((long)(nextFrameMicros1 - nowUs) > 0) {
    delayMicroseconds(nextFrameMicros1 - nowUs);
  }
  nextFrameMicros1 += FRAME_TIME_US;

  // WDT対策
  static int frameCounter1 = 0;
  if (++frameCounter1 >= 60) {
    delay(1);
    frameCounter1 = 0;
    nextFrameMicros1 = micros() + FRAME_TIME_US; // ズレ吸収
  }
}
