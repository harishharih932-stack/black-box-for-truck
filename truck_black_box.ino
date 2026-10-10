/*
 ============================================================================
  TRUCK BLACK BOX  -  ESP32-S3
  Display: ST7735 128x160, NO display library (only SPI.h + own driver)
  Storage: internal flash (LittleFS, built into the ESP32 core)
  Data   : fake ECU on the PC (fake_ecu.py) sends data over USB serial
  Alerts : Telegram bot (WiFi) with emojis
 ============================================================================

  ARDUINO IDE SETTINGS
    Board            : ESP32S3 Dev Module
    Partition Scheme : pick one that has "SPIFFS" in its name (LittleFS uses it)
    USB CDC On Boot  : Enabled if you plug the cable into the port named "USB",
                       Disabled if you plug into the port named "UART"
    No extra library needs to be installed.

  WIRING (module pin -> ESP32-S3 GPIO)
    TFT  VCC->3V3  GND->GND  CS->10  RESET->8  A0/DC->9  SDA/MOSI->11  SCK->12  LED->14
    Touch (separate pins, NOT shared with the TFT):
         T_CLK->15  T_DIN->16  T_DO->17  T_CS->18  T_IRQ->21
    Buttons (other leg to GND): LEFT->4   RIGHT->5   MORSE->6
    Buzzer (optional, active buzzer): GPIO 2

  BUTTONS
    Driver screen : hold LEFT+RIGHT 0.5 s = mechanic menu (only when vehicle stopped)
                    MORSE short = mute buzzer
    Other screens : LEFT = up / previous      RIGHT = down / next
                    MORSE short = OK / open   MORSE hold (0.7 s) = back
    Search bar    : select the "Find" row, press MORSE = typing mode
                    MORSE short = dot, MORSE long = dash (letter is added after a pause)
                    LEFT = delete, RIGHT = done
  TOUCH: tap rows / tiles, "<" at top-left = back, drag on the graph to scrub.

  TELEGRAM: put WiFi name/password, bot token and chat id below.
    Chat id: send any message to your bot, then open
    https://api.telegram.org/bot<TOKEN>/getUpdates in a browser and look for "chat":{"id":...
 ============================================================================
*/

#include <SPI.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ============================ 1. YOUR SETTINGS ============================
#define WIFI_SSID     "YOUR_WIFI_NAME"
#define WIFI_PASS     "YOUR_WIFI_PASSWORD"
#define TG_BOT_TOKEN  "PASTE_BOT_TOKEN_HERE"
#define TG_CHAT_ID    "PASTE_CHAT_ID_HERE"

// ============================ 2. PINS =====================================
#define PIN_TFT_CS     10
#define PIN_TFT_DC      9
#define PIN_TFT_RST     8
#define PIN_TFT_SCK    12
#define PIN_TFT_MOSI   11
#define PIN_TFT_LED    14
#define TFT_LED_ON   HIGH      // backlight dark? change to LOW

#define PIN_BTN_LEFT    4
#define PIN_BTN_RIGHT   5
#define PIN_BTN_MORSE   6
#define PIN_BUZZER      2      // set to -1 if no buzzer

#define USE_TOUCH       1      // safe to leave 1 even if there is no touch (pin has pull-up)
#define PIN_T_CLK      15
#define PIN_T_DIN      16
#define PIN_T_DO       17
#define PIN_T_CS       18
#define PIN_T_IRQ      21

// ---- display settings (same as the TFT test sketch) ----
#define SCREEN_W      128
#define SCREEN_H      160
#define SPI_HZ   16000000      // picture garbled? try 10000000
#define MADCTL_VALUE 0xC0      // red/blue swapped? 0xC8. upside down / mirrored? 0x00 0x08 0x40 0x80
#define COL_OFFSET      0
#define ROW_OFFSET      0
#define USE_INVERT      0      // colours negative? set 1

// ---- touch calibration (turn TOUCH_DEBUG on to see raw values in Serial Monitor) ----
#define TOUCH_DEBUG     0
#define TOUCH_SWAP_XY   0
#define TOUCH_X_MIN   300
#define TOUCH_X_MAX  3800
#define TOUCH_Y_MIN   300
#define TOUCH_Y_MAX  3800
#define TOUCH_INVERT_X  0
#define TOUCH_INVERT_Y  0

// ============================ 3. RULES / LIMITS ===========================
#define COOLANT_WARN   100.0f   // deg C
#define COOLANT_CRIT   110.0f
#define OIL_WARN       150      // kPa (engine running)
#define OIL_CRIT       100
#define BAT_WARN       12.0f    // volts, engine running (12 V system. 24 V truck: use 24.0 / 22.0 / 30.0)
#define BAT_CRIT       11.0f
#define BAT_HIGH       15.0f
#define FUEL_WARN      10       // percent
#define RPM_OVERREV    4200
#define RUNNING_RPM    600
#define SENSOR_JUMP_C  25.0f    // coolant jumping more than this in 0.1 s = sensor suspect
#define LINK_TIMEOUT_MS   3000  // no ECU data for this long = link lost
#define NO_DATA_BOOT_MS  15000  // never got data after boot for this long = link lost
#define CLEAR_SAMPLES    30     // fault must be gone this many samples (3 s) to clear

#define MECH_SPEED_LIMIT 5.0f   // km/h, mechanic mode only below this
#define ALLOW_MENU_WHILE_MOVING 0   // set 1 only for testing on the desk

// ============================ 4. STORAGE SIZES ============================
#define PRE_N           600     // 60 s before the fault  (at 10 samples/s)
#define POST_N          300     // 30 s after the fault
#define INC_MAX   (PRE_N + POST_N)
#define MAX_INC          64     // incidents kept (oldest deleted when full)
#define FS_RESERVE    20000     // keep this many bytes free
#define TRIP_EVERY       10     // trip log keeps 1 of every 10 samples (1 per second)
#define TRIP_FLUSH       30     // write trip log to flash every 30 samples
#define TRIP_MAX_BYTES 150000   // trip.bin rotates after this size
#define VIEW_TRIP_N     900     // samples shown in trip log viewer (15 min)

// ============================ 5. TYPES (keep above all functions) =========
#define TRANSP 0x0001
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_BLUE    0x001F
#define C_YELLOW  0xFFE0
#define C_CYAN    0x07FF
#define C_MAGENTA 0xF81F
#define C_ORANGE  0xFD20
#define C_GREY    0x7BEF
#define C_DGREY   0x2104
#define C_NAVY    0x0010
#define C_DKGREEN 0x0320
#define C_SKY     0x3D7F

struct __attribute__((packed)) Sample {
  uint32_t t;        // ms since boot
  uint16_t spd10;    // km/h x10
  uint16_t rpm;
  int16_t  clt10;    // coolant C x10
  uint16_t oil;      // kPa
  uint16_t bat100;   // volts x100
  uint8_t  fuel;     // %
  uint8_t  load;     // %
  uint8_t  thr;      // %
  uint8_t  flags;    // bit0 MIL, bit1 derate
};
static_assert(sizeof(Sample) == 18, "Sample must be 18 bytes");
#define FLAG_MIL    1
#define FLAG_DERATE 2

#define BB_MAGIC 0x58424F42UL
struct __attribute__((packed)) IncHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t id;
  uint32_t epoch;      // wall clock at fault (0 = unknown)
  uint32_t trigMs;     // uptime at fault
  uint32_t mask;       // bit per Fault seen in this incident
  uint16_t trigIndex;  // index of the fault sample
  uint16_t count;      // samples in file
  uint8_t  first;      // first fault
  uint8_t  sev;        // worst severity 1 warn, 2 critical
  uint16_t trigSpd10;
  uint16_t reserved;
};

struct IncMeta {
  uint16_t id; uint32_t epoch; uint32_t trigMs; uint32_t mask;
  uint8_t first; uint8_t sev; uint16_t spd10; uint16_t count;
};

enum Fault : uint8_t {
  F_NONE = 0, F_COOLANT_HIGH, F_OIL_LOW, F_BATT_LOW, F_BATT_HIGH, F_FUEL_LOW,
  F_SENSOR_COOLANT, F_DERATE, F_OVERREV, F_LINK_LOST, F_COUNT
};

struct FaultInfo { const char* shortName; const char* title; const char* advice; const char* emoji; };
static const FaultInfo FI[F_COUNT] = {
  {"NONE",    "No fault",         "",                                             ""},
  {"COOLANT", "COOLANT HOT",      "Engine too hot. Pull over safely, let it idle.", "\xF0\x9F\x8C\xA1\xEF\xB8\x8F"},
  {"OIL",     "OIL PRESSURE LOW", "Stop engine soon. Check oil level.",           "\xF0\x9F\x9B\xA2\xEF\xB8\x8F"},
  {"BATTERY", "BATTERY LOW",      "Not charging. Switch off extra loads.",        "\xF0\x9F\x94\x8B"},
  {"CHARGE",  "CHARGING HIGH",    "Alternator overcharging. Get it checked.",     "\xE2\x9A\xA1"},
  {"FUEL",    "FUEL LOW",         "Refuel at the next pump.",                     "\xE2\x9B\xBD"},
  {"SENSOR",  "COOLANT SENSOR?",  "Reading jumped. Sensor or wiring suspect.",    "\xF0\x9F\x94\xA7"},
  {"POWER",   "POWER REDUCED",    "Engine in limp mode. Do NOT overtake.",        "\xE2\x9A\xA0\xEF\xB8\x8F"},
  {"OVERREV", "ENGINE OVER-REV",  "Shift up / ease off the throttle.",            "\xF0\x9F\x8F\x8E\xEF\xB8\x8F"},
  {"ECU",     "ECU LINK LOST",    "No data from ECU. Check cable and ECU power.", "\xF0\x9F\x93\xA1"}
};
static const uint8_t DEB_ON[F_COUNT] = {0, 5, 5, 5, 10, 20, 1, 3, 5, 0};   // samples before a fault is raised

enum Screen : uint8_t { S_DRIVER, S_MENU, S_INC_LIST, S_INC_DETAIL, S_REPLAY, S_LIVE, S_FAULTS, S_STATUS };
enum IncState : uint8_t { INC_IDLE, INC_REC };

struct Btn { uint8_t pin; bool raw; bool down; uint32_t tEdge; uint32_t tDown; uint32_t lastRep; };
struct TgMsg { char text[512]; };

#define NSIG 8
static const char* const SIG_NAME[NSIG] = {"SPEED", "RPM", "COOLANT", "OIL", "BATTERY", "FUEL", "LOAD", "THROTTLE"};
static const char* const SIG_UNIT[NSIG] = {"km/h", "rpm", "C", "kPa", "V", "%", "%", "%"};
static const float SIG_LO[NSIG]   = {0, 0, 40, 0, 9, 0, 0, 0};
static const float SIG_HI[NSIG]   = {140, 4500, 130, 600, 16, 100, 100, 100};
static const float SIG_WARN[NSIG] = {-1, -1, COOLANT_WARN, OIL_WARN, BAT_WARN, -1, -1, -1};
static const float SIG_CRIT[NSIG] = {-1, -1, COOLANT_CRIT, OIL_CRIT, BAT_CRIT, -1, -1, -1};
static const uint16_t SIG_COL[NSIG] = {C_CYAN, C_ORANGE, C_MAGENTA, C_GREEN, C_YELLOW, C_SKY, C_WHITE, C_GREY};

static const char* const MORSE_TAB[36] = {
  ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--",
  "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..",
  "-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----."
};

static const uint8_t font5x7[95][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x56,0x20,0x50}, {0x00,0x08,0x07,0x03,0x00},
  {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x2A,0x1C,0x7F,0x1C,0x2A}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x80,0x70,0x30,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x00,0x60,0x60,0x00}, {0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x72,0x49,0x49,0x49,0x46}, {0x21,0x41,0x49,0x4D,0x33},
  {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x31}, {0x41,0x21,0x11,0x09,0x07},
  {0x36,0x49,0x49,0x49,0x36}, {0x46,0x49,0x49,0x29,0x1E}, {0x00,0x00,0x14,0x00,0x00}, {0x00,0x40,0x34,0x00,0x00},
  {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x59,0x09,0x06},
  {0x3E,0x41,0x5D,0x59,0x4E}, {0x7C,0x12,0x11,0x12,0x7C}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x41,0x3E}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x41,0x51,0x73},
  {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x1C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x26,0x49,0x49,0x49,0x32},
  {0x03,0x01,0x7F,0x01,0x03}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x59,0x49,0x4D,0x43}, {0x00,0x7F,0x41,0x41,0x41},
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x41,0x7F}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
  {0x00,0x03,0x07,0x08,0x00}, {0x20,0x54,0x54,0x78,0x40}, {0x7F,0x28,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x28},
  {0x38,0x44,0x44,0x28,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x00,0x08,0x7E,0x09,0x02}, {0x18,0xA4,0xA4,0x9C,0x78},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x40,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x78,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
  {0xFC,0x18,0x24,0x24,0x18}, {0x18,0x24,0x24,0x18,0xFC}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x24},
  {0x04,0x04,0x3F,0x44,0x24}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44}, {0x4C,0x90,0x90,0x90,0x7C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x77,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x02,0x01,0x02,0x04,0x02}
};

// ============================ 6. GLOBALS ==================================
static uint16_t* fb = nullptr;                 // frame buffer (byte-swapped RGB565)

// ECU data + rules
static Sample   latest;
static bool     haveData = false;
static uint32_t lastDataMs = 0;
static uint32_t epochBase = 0;                 // local epoch seconds when millis()==0 (0 = unknown)
static Sample   ring[PRE_N];
static int      ringHead = 0, ringCount = 0;
static bool     fActive[F_COUNT];
static uint8_t  fSev[F_COUNT];
static uint16_t fCount[F_COUNT];
static uint8_t  rCnt[F_COUNT], rClr[F_COUNT];
static float    prevClt = 0; static bool havePrevClt = false; static uint32_t glitchUntil = 0;
static bool     fMuted = false;
static uint32_t sampleCounter = 0;

// incident recording
static Sample    incBuf[INC_MAX];
static IncHeader incHdr;
static int       incCount = 0, incRemain = 0;
static IncState  incState = INC_IDLE;
static bool      justStarted = false;
static IncMeta   meta[MAX_INC];
static int       metaN = 0;
static uint16_t  nextId = 1;

// trip log
static Sample tripBuf[TRIP_FLUSH];
static int    tripN = 0;

// viewer data (incident or trip)
static Sample viewBuf[INC_MAX];
static int    viewCount = 0, viewTrig = -1;
static bool   viewIsTrip = false;
static char   viewTitle[24] = "";

// UI
static Screen   screen = S_DRIVER;
static bool     dirty = true;
static uint32_t lastRender = 0;
static int      menuCur = 0;
static int      lcur = -1, ltop = 0;           // incident list cursor (-1 = search bar)
static int      viewIdx[MAX_INC]; static int viewN = 0;
static int      detMeta = 0;
static bool     typing = false;
static char     search[13] = "";
static char     mbuf[8] = ""; static int mlen = 0; static uint32_t mLast = 0;
static int      vsig = 0, curCol = 0;
static bool     eraseArmed = false;
static char     toastMsg[28] = ""; static uint32_t toastUntil = 0;

// inputs
static Btn bL = {PIN_BTN_LEFT, false, false, 0, 0, 0};
static Btn bR = {PIN_BTN_RIGHT, false, false, 0, 0, 0};
static Btn bM = {PIN_BTN_MORSE, false, false, 0, 0, 0};
static bool comboArmed = false, comboFired = false, supL = false, supR = false;
static uint32_t comboStart = 0;
static bool tDown = false, tMoved = false; static int tx0 = 0, ty0 = 0, tCnt = 0, tUp = 0; static uint32_t tStart = 0, tLastPoll = 0;

// telegram
static QueueHandle_t tgQ = nullptr;
static bool tgEnabled = false;
static volatile int tgState = 0;               // 0 off, 1 no wifi, 2 ok, 3 last send failed
static volatile int tgCode = 0;
static volatile uint16_t tgSent = 0, tgDropped = 0;

// serial line buffer
static char lineBuf[160]; static int lineLen = 0;

// forward declarations
static void faultRaise(uint8_t f, uint8_t sev, const Sample& ref);
static void faultClear(uint8_t f, const Sample& ref);
static void finishIncident();
static void rebuildFilter();

// ============================ 7. TFT DRIVER (no library) ==================
static inline uint16_t swap16(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }

static void sendCmd(uint8_t c) {
  digitalWrite(PIN_TFT_DC, LOW); digitalWrite(PIN_TFT_CS, LOW);
  SPI.transfer(c);
  digitalWrite(PIN_TFT_CS, HIGH);
}
static void sendData(const uint8_t* d, uint32_t n) {
  digitalWrite(PIN_TFT_DC, HIGH); digitalWrite(PIN_TFT_CS, LOW);
  SPI.writeBytes(d, n);
  digitalWrite(PIN_TFT_CS, HIGH);
}
static void sendCmdData(uint8_t c, const uint8_t* d, uint32_t n) { sendCmd(c); sendData(d, n); }

static void setWindow(int x0, int y0, int x1, int y1) {
  uint8_t d[4];
  x0 += COL_OFFSET; x1 += COL_OFFSET; y0 += ROW_OFFSET; y1 += ROW_OFFSET;
  d[0] = 0; d[1] = (uint8_t)x0; d[2] = 0; d[3] = (uint8_t)x1; sendCmdData(0x2A, d, 4);
  d[1] = (uint8_t)y0; d[3] = (uint8_t)y1; sendCmdData(0x2B, d, 4);
  sendCmd(0x2C);
}

static void tftInit() {
  pinMode(PIN_TFT_CS, OUTPUT); pinMode(PIN_TFT_DC, OUTPUT);
  pinMode(PIN_TFT_RST, OUTPUT); pinMode(PIN_TFT_LED, OUTPUT);
  digitalWrite(PIN_TFT_LED, TFT_LED_ON);
  digitalWrite(PIN_TFT_CS, HIGH);
  SPI.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI, -1);
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));

  digitalWrite(PIN_TFT_RST, HIGH); delay(10);
  digitalWrite(PIN_TFT_RST, LOW);  delay(20);
  digitalWrite(PIN_TFT_RST, HIGH); delay(150);
  sendCmd(0x01); delay(150);
  sendCmd(0x11); delay(255);

  const uint8_t frm1[] = {0x01, 0x2C, 0x2D};
  const uint8_t frm3[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
  const uint8_t invc[] = {0x07};
  const uint8_t pw1[]  = {0xA2, 0x02, 0x84};
  const uint8_t pw2[]  = {0xC5};
  const uint8_t pw3[]  = {0x0A, 0x00};
  const uint8_t pw4[]  = {0x8A, 0x2A};
  const uint8_t pw5[]  = {0x8A, 0xEE};
  const uint8_t vm1[]  = {0x0E};
  const uint8_t madctl[] = {MADCTL_VALUE};
  const uint8_t colmod[] = {0x05};
  const uint8_t gam1[] = {0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};
  const uint8_t gam2[] = {0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};
  sendCmdData(0xB1, frm1, 3); sendCmdData(0xB2, frm1, 3); sendCmdData(0xB3, frm3, 6);
  sendCmdData(0xB4, invc, 1);
  sendCmdData(0xC0, pw1, 3); sendCmdData(0xC1, pw2, 1); sendCmdData(0xC2, pw3, 2);
  sendCmdData(0xC3, pw4, 2); sendCmdData(0xC4, pw5, 2); sendCmdData(0xC5, vm1, 1);
  sendCmd(USE_INVERT ? 0x21 : 0x20);
  sendCmdData(0x36, madctl, 1); sendCmdData(0x3A, colmod, 1);
  sendCmdData(0xE0, gam1, 16); sendCmdData(0xE1, gam2, 16);
  sendCmd(0x13); delay(10);
  sendCmd(0x29); delay(100);
}

// push the whole frame buffer to the screen
static void flushScreen() {
  setWindow(0, 0, SCREEN_W - 1, SCREEN_H - 1);
  digitalWrite(PIN_TFT_DC, HIGH); digitalWrite(PIN_TFT_CS, LOW);
  const uint8_t* p = (const uint8_t*)fb;
  uint32_t n = (uint32_t)SCREEN_W * SCREEN_H * 2;
  while (n) { uint32_t c = n > 2048 ? 2048 : n; SPI.writeBytes(p, c); p += c; n -= c; }
  digitalWrite(PIN_TFT_CS, HIGH);
}

// ---- drawing into the frame buffer ----
static void fillRect(int x, int y, int w, int h, uint16_t color) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > SCREEN_W) w = SCREEN_W - x;
  if (y + h > SCREEN_H) h = SCREEN_H - y;
  if (w <= 0 || h <= 0) return;
  uint16_t v = swap16(color);
  for (int yy = y; yy < y + h; yy++) {
    uint16_t* p = fb + yy * SCREEN_W + x;
    for (int i = 0; i < w; i++) p[i] = v;
  }
}
static void drawRect(int x, int y, int w, int h, uint16_t c) {
  fillRect(x, y, w, 1, c); fillRect(x, y + h - 1, w, 1, c);
  fillRect(x, y, 1, h, c); fillRect(x + w - 1, y, 1, h, c);
}
static void drawChar(int x, int y, char ch, int s, uint16_t fg, uint16_t bg) {
  if (ch < 32 || ch > 126) ch = '?';
  const uint8_t* g = font5x7[ch - 32];
  for (int gx = 0; gx < 6; gx++) {
    uint8_t col = (gx < 5) ? g[gx] : 0;
    for (int gy = 0; gy < 8; gy++) {
      bool on = ((col >> gy) & 1);
      if (on) fillRect(x + gx * s, y + gy * s, s, s, fg);
      else if (bg != TRANSP) fillRect(x + gx * s, y + gy * s, s, s, bg);
    }
  }
}
static void drawText(int x, int y, const char* str, int s, uint16_t fg, uint16_t bg) {
  while (*str) { drawChar(x, y, *str, s, fg, bg); x += 6 * s; str++; }
}
static int textW(const char* str, int s) { return (int)strlen(str) * 6 * s; }
static void drawTextC(int cx, int y, const char* str, int s, uint16_t fg, uint16_t bg) {
  drawText(cx - textW(str, s) / 2, y, str, s, fg, bg);
}
static void drawWrapped(int x, int y, const char* text, int maxChars, int maxLines, uint16_t fg, uint16_t bg) {
  char line[32]; int ln = 0; const char* p = text;
  if (maxChars > 30) maxChars = 30;
  while (*p && ln < maxLines) {
    int len = 0, lastSpace = -1;
    while (p[len] && len < maxChars) { if (p[len] == ' ') lastSpace = len; len++; }
    if (p[len] && len == maxChars && lastSpace > 0) len = lastSpace;
    memcpy(line, p, len); line[len] = 0;
    drawText(x, y + ln * 10, line, 1, fg, bg);
    p += len; while (*p == ' ') p++;
    ln++;
  }
}

// ============================ 8. TOUCH (XPT2046, bit-banged) ==============
#if USE_TOUCH
static uint16_t xptRead(uint8_t cmd) {
  digitalWrite(PIN_T_CS, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(PIN_T_DIN, (cmd >> i) & 1);
    digitalWrite(PIN_T_CLK, HIGH); digitalWrite(PIN_T_CLK, LOW);
  }
  uint16_t v = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PIN_T_CLK, HIGH); digitalWrite(PIN_T_CLK, LOW);
    v <<= 1; if (digitalRead(PIN_T_DO)) v |= 1;
  }
  digitalWrite(PIN_T_CS, HIGH);
  return (v >> 3) & 0x0FFF;
}
static bool touchRead(int& x, int& y) {
  if (digitalRead(PIN_T_IRQ) != LOW) return false;
  xptRead(0xD0);
  long sx = 0, sy = 0;
  for (int i = 0; i < 4; i++) { sx += xptRead(0xD0); sy += xptRead(0x90); }
  int rx = (int)(sx / 4), ry = (int)(sy / 4);
  if (rx == 0 && ry == 0) return false;
#if TOUCH_DEBUG
  Serial.printf("#TOUCH raw %d %d\n", rx, ry);
#endif
  if (TOUCH_SWAP_XY) { int t = rx; rx = ry; ry = t; }
  long px = (long)(rx - TOUCH_X_MIN) * (SCREEN_W - 1) / (TOUCH_X_MAX - TOUCH_X_MIN);
  long py = (long)(ry - TOUCH_Y_MIN) * (SCREEN_H - 1) / (TOUCH_Y_MAX - TOUCH_Y_MIN);
  if (TOUCH_INVERT_X) px = SCREEN_W - 1 - px;
  if (TOUCH_INVERT_Y) py = SCREEN_H - 1 - py;
  if (px < 0) px = 0; if (px > SCREEN_W - 1) px = SCREEN_W - 1;
  if (py < 0) py = 0; if (py > SCREEN_H - 1) py = SCREEN_H - 1;
  x = (int)px; y = (int)py;
  return true;
}
#endif

// ============================ 9. SMALL HELPERS ============================
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float sigVal(const Sample& s, int i) {
  switch (i) {
    case 0: return s.spd10 / 10.0f;
    case 1: return (float)s.rpm;
    case 2: return s.clt10 / 10.0f;
    case 3: return (float)s.oil;
    case 4: return s.bat100 / 100.0f;
    case 5: return (float)s.fuel;
    case 6: return (float)s.load;
    default: return (float)s.thr;
  }
}
static void toast(const char* m) {
  strncpy(toastMsg, m, sizeof(toastMsg) - 1); toastMsg[sizeof(toastMsg) - 1] = 0;
  toastUntil = millis() + 2200; dirty = true;
}
static uint32_t epochAt(uint32_t ms) { return epochBase ? epochBase + ms / 1000 : 0; }
static void fmtClock(uint32_t epoch, uint32_t ms, char* out, size_t n) {
  if (epoch) {
    time_t t = (time_t)epoch; struct tm tmv; gmtime_r(&t, &tmv);
    strftime(out, n, "%d %b %H:%M:%S", &tmv);
  } else {
    uint32_t s = ms / 1000;
    snprintf(out, n, "boot+%02u:%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
  }
}
static uint8_t overallSev() {
  uint8_t m = 0;
  for (int f = 1; f < F_COUNT; f++) if (fActive[f] && fSev[f] > m) m = fSev[f];
  return m;
}
static int topFault() {
  int best = 0; uint8_t bs = 0;
  for (int f = 1; f < F_COUNT; f++) if (fActive[f] && fSev[f] > bs) { bs = fSev[f]; best = f; }
  return best;
}
static bool linkOk() { return haveData && !fActive[F_LINK_LOST]; }
static bool canOpenMenu() { return ALLOW_MENU_WHILE_MOVING || !haveData || (latest.spd10 / 10.0f) < MECH_SPEED_LIMIT; }

// ============================ 10. TELEGRAM ================================
static void tgEnqueue(const char* text) {
  if (!tgEnabled || !tgQ) return;
  TgMsg m; strncpy(m.text, text, sizeof(m.text) - 1); m.text[sizeof(m.text) - 1] = 0;
  if (xQueueSend(tgQ, &m, 0) != pdTRUE) {          // queue full: drop the oldest
    TgMsg old; xQueueReceive(tgQ, &old, 0);
    xQueueSend(tgQ, &m, 0); tgDropped++;
  }
}

static bool tgSendNow(const char* text) {
  static char body[1300];
  int n = snprintf(body, sizeof(body), "{\"chat_id\":\"%s\",\"text\":\"", TG_CHAT_ID);
  for (const char* p = text; *p && n < (int)sizeof(body) - 8; ++p) {
    unsigned char c = (unsigned char)*p;
    if (c == '"' || c == '\\') { body[n++] = '\\'; body[n++] = (char)c; }
    else if (c == '\n') { body[n++] = '\\'; body[n++] = 'n'; }
    else if (c < 0x20) { body[n++] = ' '; }
    else body[n++] = (char)c;                       // UTF-8 (emojis) passes through
  }
  n += snprintf(body + n, sizeof(body) - n, "\"}");
  WiFiClientSecure client; client.setInsecure();    // no certificate check (demo)
  HTTPClient http; http.setTimeout(10000);
  String url = String("https://api.telegram.org/bot") + TG_BOT_TOKEN + "/sendMessage";
  if (!http.begin(client, url)) return false;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t*)body, (size_t)n);
  tgCode = code;
  http.end();
  return code == 200;
}

static void tgTask(void*) {
  WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true); WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t lastTry = millis(); TgMsg cur; bool have = false; int tries = 0;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      tgState = 1;
      if (millis() - lastTry > 15000) { WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASS); lastTry = millis(); }
      vTaskDelay(pdMS_TO_TICKS(500)); continue;
    }
    if (tgState == 1) tgState = 2;
    if (!have) {
      if (xQueueReceive(tgQ, &cur, pdMS_TO_TICKS(500)) == pdTRUE) { have = true; tries = 0; }
      else continue;
    }
    if (tgSendNow(cur.text)) { have = false; tgSent++; tgState = 2; }
    else {
      tries++; tgState = 3;
      if (tries >= 4) { have = false; tgDropped++; }
      vTaskDelay(pdMS_TO_TICKS(4000));
    }
  }
}

static void nowClock(char* out, size_t n) { uint32_t m = millis(); fmtClock(epochAt(m), m, out, n); }

static void notifyRaise(uint8_t f, uint8_t sev, const Sample& s, uint16_t id) {
  char msg[512], tm[28], incLine[48] = "";
  nowClock(tm, sizeof(tm));
  if (id) snprintf(incLine, sizeof(incLine), "\xF0\x9F\x92\xBE Incident #%u recorded on device\n", (unsigned)id);
  if (f == F_LINK_LOST) {
    snprintf(msg, sizeof(msg),
      "\xF0\x9F\x93\xA1\xE2\x9D\x8C DEVICE ALERT\nNo data from the ECU.\n\xF0\x9F\x94\x8C Check ECU cable / ECU power.\n"
      "\xF0\x9F\x95\x92 %s\n%s", tm, incLine);
  } else {
    snprintf(msg, sizeof(msg),
      "\xF0\x9F\x9A\xA8 TRUCK BLACK BOX ALERT\n%s %s\n%s %s\n\n"
      "\xF0\x9F\x9A\x9A Speed: %.0f km/h\n\xE2\x9A\x99\xEF\xB8\x8F RPM: %u\n"
      "\xF0\x9F\x8C\xA1\xEF\xB8\x8F Coolant: %.1f \xC2\xB0""C\n\xF0\x9F\x9B\xA2\xEF\xB8\x8F Oil: %u kPa\n"
      "\xF0\x9F\x94\x8B Battery: %.1f V\n\xE2\x9B\xBD Fuel: %u%%\n\n"
      "\xF0\x9F\x95\x92 %s\n%s\xF0\x9F\x92\xA1 %s",
      sev >= 2 ? "\xF0\x9F\x94\xB4" : "\xF0\x9F\x9F\xA1", sev >= 2 ? "CRITICAL" : "WARNING",
      FI[f].emoji, FI[f].title,
      s.spd10 / 10.0f, (unsigned)s.rpm, s.clt10 / 10.0f, (unsigned)s.oil, s.bat100 / 100.0f, (unsigned)s.fuel,
      tm, incLine, FI[f].advice);
  }
  tgEnqueue(msg);
}

static void notifyClear(uint8_t f) {
  char msg[160], tm[28]; nowClock(tm, sizeof(tm));
  if (f == F_LINK_LOST) snprintf(msg, sizeof(msg), "\xF0\x9F\x93\xA1\xE2\x9C\x85 ECU link restored\n\xF0\x9F\x95\x92 %s", tm);
  else snprintf(msg, sizeof(msg), "\xE2\x9C\x85 CLEARED: %s %s is back to normal\n\xF0\x9F\x95\x92 %s", FI[f].emoji, FI[f].title, tm);
  tgEnqueue(msg);
}

// ============================ 11. FLASH STORAGE ===========================
static void incPath(uint16_t id, char* out, size_t n) { snprintf(out, n, "/inc/%04u.bin", (unsigned)id); }
static uint32_t fsFree() { return LittleFS.totalBytes() - LittleFS.usedBytes(); }

static void deleteOldestIncident() {
  if (metaN <= 0) return;
  char p[24]; incPath(meta[0].id, p, sizeof(p)); LittleFS.remove(p);
  for (int i = 1; i < metaN; i++) meta[i - 1] = meta[i];
  metaN--;
}
static void ensureSpace(uint32_t need) {
  while (fsFree() < need + FS_RESERVE && metaN > 0) deleteOldestIncident();
}

static void scanIncidents() {
  metaN = 0;
  File root = LittleFS.open("/inc");
  if (root && root.isDirectory()) {
    File f = root.openNextFile();
    while (f) {
      if (!f.isDirectory() && metaN < MAX_INC) {
        IncHeader h;
        if (f.size() >= (int)sizeof(h) && f.read((uint8_t*)&h, sizeof(h)) == sizeof(h) && h.magic == BB_MAGIC) {
          IncMeta m = {h.id, h.epoch, h.trigMs, h.mask, h.first, h.sev, h.trigSpd10, h.count};
          meta[metaN++] = m;
        }
      }
      f.close(); f = root.openNextFile();
    }
  }
  for (int i = 1; i < metaN; i++) {                // sort by id (oldest first)
    IncMeta k = meta[i]; int j = i - 1;
    while (j >= 0 && meta[j].id > k.id) { meta[j + 1] = meta[j]; j--; }
    meta[j + 1] = k;
  }
  nextId = metaN ? (uint16_t)(meta[metaN - 1].id + 1) : 1;
  memset(fCount, 0, sizeof(fCount));
  for (int i = 0; i < metaN; i++)
    for (int f = 1; f < F_COUNT; f++) if (meta[i].mask & (1UL << f)) fCount[f]++;
}

static bool writeIncident() {
  uint32_t bytes = sizeof(IncHeader) + (uint32_t)incCount * sizeof(Sample);
  ensureSpace(bytes + 4096);
  char p[24]; incPath(incHdr.id, p, sizeof(p));
  File f = LittleFS.open(p, FILE_WRITE);
  if (!f) return false;
  size_t w = f.write((const uint8_t*)&incHdr, sizeof(incHdr));
  w += f.write((const uint8_t*)incBuf, (size_t)incCount * sizeof(Sample));
  f.close();
  if (w != bytes) { LittleFS.remove(p); return false; }
  if (metaN >= MAX_INC) deleteOldestIncident();
  IncMeta m = {incHdr.id, incHdr.epoch, incHdr.trigMs, incHdr.mask, incHdr.first, incHdr.sev, incHdr.trigSpd10, incHdr.count};
  meta[metaN++] = m;
  return true;
}

static void tripFlush() {
  if (tripN == 0) return;
  if (fsFree() > FS_RESERVE) {
    File f = LittleFS.open("/trip.bin", FILE_APPEND);
    if (f) {
      f.write((const uint8_t*)tripBuf, (size_t)tripN * sizeof(Sample));
      size_t sz = f.size(); f.close();
      if (sz > TRIP_MAX_BYTES) { LittleFS.remove("/trip_old.bin"); LittleFS.rename("/trip.bin", "/trip_old.bin"); }
    }
  }
  tripN = 0;
}
static void tripAdd(const Sample& s) {
  tripBuf[tripN++] = s;
  if (tripN >= TRIP_FLUSH) tripFlush();
}

static bool loadIncidentToView(int mi) {
  char p[24]; incPath(meta[mi].id, p, sizeof(p));
  File f = LittleFS.open(p, FILE_READ);
  if (!f) return false;
  IncHeader h;
  if (f.read((uint8_t*)&h, sizeof(h)) != sizeof(h) || h.magic != BB_MAGIC || h.count > INC_MAX) { f.close(); return false; }
  int n = (int)f.read((uint8_t*)viewBuf, (size_t)h.count * sizeof(Sample)) / (int)sizeof(Sample);
  f.close();
  viewCount = n; viewTrig = h.trigIndex < n ? h.trigIndex : n - 1; viewIsTrip = false;
  snprintf(viewTitle, sizeof(viewTitle), "INCIDENT #%u", (unsigned)h.id);
  curCol = (int)((long)viewTrig * SCREEN_W / (n > 0 ? n : 1));
  return n > 1;
}

static bool loadTripToView() {
  tripFlush();
  viewCount = 0; viewTrig = -1; viewIsTrip = true;
  File f = LittleFS.open("/trip.bin", FILE_READ);
  if (f) {
    int total = (int)(f.size() / sizeof(Sample));
    int take = total > VIEW_TRIP_N ? VIEW_TRIP_N : total;
    f.seek((size_t)(total - take) * sizeof(Sample));
    viewCount = (int)f.read((uint8_t*)viewBuf, (size_t)take * sizeof(Sample)) / (int)sizeof(Sample);
    f.close();
  }
  strcpy(viewTitle, "TRIP LOG");
  curCol = SCREEN_W - 1;
  return viewCount > 1;
}

static void eraseAllLogs() {
  for (int i = 0; i < metaN; i++) { char p[24]; incPath(meta[i].id, p, sizeof(p)); LittleFS.remove(p); }
  LittleFS.remove("/trip.bin"); LittleFS.remove("/trip_old.bin");
  metaN = 0; tripN = 0; nextId = 1; memset(fCount, 0, sizeof(fCount));
}

// ============================ 12. INCIDENT RECORDER =======================
static uint16_t startIncident(uint8_t f, uint8_t sev, const Sample& ref) {
  int start = (ringHead - ringCount + PRE_N) % PRE_N;
  incCount = 0;
  for (int i = 0; i < ringCount; i++) incBuf[incCount++] = ring[(start + i) % PRE_N];
  memset(&incHdr, 0, sizeof(incHdr));
  incHdr.magic = BB_MAGIC; incHdr.version = 1; incHdr.id = nextId++;
  incHdr.trigMs = ref.t; incHdr.epoch = epochAt(ref.t);
  incHdr.mask = 1UL << f; incHdr.first = f; incHdr.sev = sev;
  incHdr.trigIndex = (uint16_t)(incCount - 1); incHdr.trigSpd10 = ref.spd10;
  incRemain = POST_N; incState = INC_REC; justStarted = true;
  uint16_t id = incHdr.id;
  if (f == F_LINK_LOST) finishIncident();          // no more samples will come
  return id;
}

static void finishIncident() {
  if (incState != INC_REC) return;
  incState = INC_IDLE;
  incHdr.count = (uint16_t)incCount;
  bool ok = writeIncident();
  Serial.printf("#INCIDENT %u %s (%d samples, %u KB free)\n", (unsigned)incHdr.id, ok ? "saved" : "SAVE FAILED", incCount, (unsigned)(fsFree() / 1024));
  char msg[200];
  if (ok) {
    snprintf(msg, sizeof(msg), "\xF0\x9F\x92\xBE Incident #%u saved: %d s of data (before + after the fault).",
             (unsigned)incHdr.id, incCount / 10);
  } else {
    snprintf(msg, sizeof(msg), "\xF0\x9F\x92\xBE\xE2\x9D\x8C Storage error: incident #%u could NOT be saved!", (unsigned)incHdr.id);
  }
  tgEnqueue(msg);
  if (screen == S_INC_LIST) { rebuildFilter(); dirty = true; }
}

static void appendIncidentSample(const Sample& s) {
  if (incCount < INC_MAX) incBuf[incCount++] = s;
  if (--incRemain <= 0 || incCount >= INC_MAX) finishIncident();
}

// ============================ 13. RULE ENGINE =============================
static void faultRaise(uint8_t f, uint8_t sev, const Sample& ref) {
  bool was = fActive[f];
  fActive[f] = true; fSev[f] = sev; if (!was) fCount[f]++;
  fMuted = false;
  uint16_t id = 0;
  if (incState == INC_IDLE) {
    if (ringCount > 0) id = startIncident(f, sev, ref);
  } else {
    id = incHdr.id;
    incHdr.mask |= (1UL << f); if (sev > incHdr.sev) incHdr.sev = sev;
    if (f == F_LINK_LOST) finishIncident(); else incRemain = POST_N;
  }
  Serial.printf("#FAULT %s sev=%u inc=%u\n", FI[f].shortName, (unsigned)sev, (unsigned)id);
  notifyRaise(f, sev, ref, id);
  dirty = true;
}

static void faultClear(uint8_t f, const Sample& ref) {
  (void)ref;
  fActive[f] = false; fSev[f] = 0;
  Serial.printf("#CLEAR %s\n", FI[f].shortName);
  notifyClear(f);
  dirty = true;
}

static void evalRules(const Sample& s, uint32_t now) {
  float clt = s.clt10 / 10.0f, bat = s.bat100 / 100.0f;
  bool running = s.rpm > RUNNING_RPM;
  uint8_t want[F_COUNT]; memset(want, 0, sizeof(want));
  if (clt >= COOLANT_CRIT) want[F_COOLANT_HIGH] = 2; else if (clt >= COOLANT_WARN) want[F_COOLANT_HIGH] = 1;
  if (running) {
    if (s.oil < OIL_CRIT) want[F_OIL_LOW] = 2; else if (s.oil < OIL_WARN) want[F_OIL_LOW] = 1;
    if (bat < BAT_CRIT) want[F_BATT_LOW] = 2; else if (bat < BAT_WARN) want[F_BATT_LOW] = 1;
    if (bat > BAT_HIGH) want[F_BATT_HIGH] = 1;
  }
  if (s.fuel < FUEL_WARN) want[F_FUEL_LOW] = 1;
  if (s.rpm > RPM_OVERREV) want[F_OVERREV] = 1;
  if (s.flags & FLAG_DERATE) want[F_DERATE] = 2;

  bool glitch = (clt < -20.0f || clt > 150.0f);
  if (havePrevClt && fabsf(clt - prevClt) > SENSOR_JUMP_C) glitch = true;
  prevClt = clt; havePrevClt = true;
  if (glitch) glitchUntil = now + 4000;
  if (now < glitchUntil) want[F_SENSOR_COOLANT] = 1;

  for (uint8_t f = 1; f < F_COUNT; f++) {
    if (f == F_LINK_LOST) continue;
    if (want[f]) {
      rClr[f] = 0; if (rCnt[f] < 255) rCnt[f]++;
      if (rCnt[f] >= DEB_ON[f]) {
        if (!fActive[f] || want[f] > fSev[f]) faultRaise(f, want[f], s);
      }
    } else {
      rCnt[f] = 0; if (rClr[f] < 255) rClr[f]++;
      if (fActive[f] && rClr[f] >= CLEAR_SAMPLES) faultClear(f, s);
    }
  }
}

static void handleSample(const Sample& s) {
  uint32_t now = millis();
  justStarted = false;
  latest = s; haveData = true; lastDataMs = now;
  ring[ringHead] = s; ringHead = (ringHead + 1) % PRE_N; if (ringCount < PRE_N) ringCount++;
  if (sampleCounter++ % TRIP_EVERY == 0) tripAdd(s);
  evalRules(s, now);
  if (incState == INC_REC && !justStarted) appendIncidentSample(s);
  if (screen == S_DRIVER || screen == S_LIVE) dirty = true;
}

static void linkCheck(uint32_t now) {
  bool lost = haveData ? (now - lastDataMs > LINK_TIMEOUT_MS) : (now > NO_DATA_BOOT_MS);
  if (lost && !fActive[F_LINK_LOST]) {
    Sample ref; memset(&ref, 0, sizeof(ref)); ref.t = now;
    if (ringCount > 0) ref = ring[(ringHead - 1 + PRE_N) % PRE_N];
    faultRaise(F_LINK_LOST, 2, ref);
  } else if (!lost && fActive[F_LINK_LOST]) {
    faultClear(F_LINK_LOST, latest);
  }
}

// ============================ 14. SERIAL INPUT ============================
static bool parseData(char* line, Sample& s) {
  // D,speed,rpm,coolant,oil,battery,fuel,load,throttle,mil,derate
  float v[10]; int n = 0; char* save = nullptr;
  char* tok = strtok_r(line + 2, ",", &save);
  while (tok && n < 10) { v[n++] = (float)atof(tok); tok = strtok_r(nullptr, ",", &save); }
  if (n < 10) return false;
  s.t = millis();
  s.spd10  = (uint16_t)clampf(v[0] * 10.0f, 0, 6553);
  s.rpm    = (uint16_t)clampf(v[1], 0, 65000);
  s.clt10  = (int16_t)clampf(v[2] * 10.0f, -3000, 3000);
  s.oil    = (uint16_t)clampf(v[3], 0, 65000);
  s.bat100 = (uint16_t)clampf(v[4] * 100.0f, 0, 6500);
  s.fuel   = (uint8_t)clampf(v[5], 0, 100);
  s.load   = (uint8_t)clampf(v[6], 0, 100);
  s.thr    = (uint8_t)clampf(v[7], 0, 100);
  s.flags  = (uint8_t)((v[8] > 0.5f ? FLAG_MIL : 0) | (v[9] > 0.5f ? FLAG_DERATE : 0));
  return true;
}

static void processLine(char* line) {
  if (line[0] == 'D' && line[1] == ',') {
    Sample s; if (parseData(line, s)) handleSample(s);
  } else if (line[0] == 'T' && line[1] == ',') {   // time sync from PC (local epoch seconds)
    uint32_t e = (uint32_t)strtoul(line + 2, nullptr, 10);
    if (e > 1700000000UL) { epochBase = e - millis() / 1000; dirty = true; }
  }
}

static void serialPump() {
  int guard = 0;
  while (Serial.available() && guard++ < 600) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = 0; processLine(lineBuf); lineLen = 0; }
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                                   // too long, throw away
    }
  }
}

// ============================ 15. SCREENS (drawing) =======================
static uint16_t valColor(int sig, float v) {
  if (sig == 2) return v >= COOLANT_CRIT ? C_RED : (v >= COOLANT_WARN ? C_YELLOW : C_GREEN);
  if (sig == 3) return v < OIL_CRIT ? C_RED : (v < OIL_WARN ? C_YELLOW : C_GREEN);
  if (sig == 4) return v < BAT_CRIT ? C_RED : (v < BAT_WARN ? C_YELLOW : C_GREEN);
  if (sig == 5) return v < FUEL_WARN ? C_YELLOW : C_WHITE;
  return C_WHITE;
}

static void drawHeader(const char* title, bool back) {
  fillRect(0, 0, SCREEN_W, 13, C_NAVY);
  if (back) drawText(3, 3, "<", 1, C_WHITE, TRANSP);
  drawText(back ? 14 : 4, 3, title, 1, C_WHITE, TRANSP);
  fillRect(SCREEN_W - 9, 3, 6, 6, linkOk() ? C_GREEN : C_RED);
}

static void drawToast() {
  if (millis() < toastUntil && toastMsg[0]) {
    int w = textW(toastMsg, 1) + 8;
    fillRect((SCREEN_W - w) / 2, SCREEN_H - 24, w, 14, C_WHITE);
    drawText((SCREEN_W - w) / 2 + 4, SCREEN_H - 21, toastMsg, 1, C_BLACK, TRANSP);
  }
}

static void renderDriver() {
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  uint8_t sev = overallSev();
  bool ok = linkOk();
  uint16_t bg = !ok ? C_DGREY : (sev == 2 ? C_RED : (sev == 1 ? C_YELLOW : C_DKGREEN));
  uint16_t fg = (ok && sev == 1) ? C_BLACK : C_WHITE;
  fillRect(0, 0, SCREEN_W, 32, bg);
  const char* st = !haveData ? "WAITING" : (!ok ? "NO ECU" : (sev == 2 ? "DANGER" : (sev == 1 ? "WARNING" : "ALL GOOD")));
  drawTextC(SCREEN_W / 2, 4, st, 2, fg, TRANSP);
  int tf = topFault();
  const char* sub = !haveData ? "waiting for ECU data" : (tf ? FI[tf].title : "Engine normal");
  drawTextC(SCREEN_W / 2, 22, sub, 1, fg, TRANSP);

  char b[40];
  float spd = latest.spd10 / 10.0f;
  snprintf(b, sizeof(b), "%d", (int)(spd + 0.5f));
  drawText(6, 38, b, 4, C_WHITE, TRANSP);
  drawText(6 + textW(b, 4) + 4, 62, "km/h", 1, C_GREY, TRANSP);

  float clt = latest.clt10 / 10.0f, bat = latest.bat100 / 100.0f;
  snprintf(b, sizeof(b), "RPM %u", (unsigned)latest.rpm);                 drawText(4, 78, b, 1, C_WHITE, TRANSP);
  snprintf(b, sizeof(b), "COOL %.0fC", clt);                              drawText(65, 78, b, 1, valColor(2, clt), TRANSP);
  snprintf(b, sizeof(b), "OIL %u", (unsigned)latest.oil);              drawText(4, 90, b, 1, valColor(3, latest.oil), TRANSP);
  snprintf(b, sizeof(b), "BATT %.1fV", bat);                              drawText(65, 90, b, 1, valColor(4, bat), TRANSP);
  snprintf(b, sizeof(b), "FUEL %u%%", (unsigned)latest.fuel);             drawText(4, 102, b, 1, valColor(5, latest.fuel), TRANSP);
  snprintf(b, sizeof(b), "LOAD %u%%", (unsigned)latest.load);             drawText(65, 102, b, 1, C_WHITE, TRANSP);

  uint16_t abg = sev == 2 ? C_RED : (sev == 1 ? C_YELLOW : C_DGREY);
  uint16_t afg = sev == 1 ? C_BLACK : C_WHITE;
  fillRect(0, 114, SCREEN_W, 36, abg);
  const char* adv = tf ? FI[tf].advice : (ok ? "Drive safe. All systems normal." : "Check ECU connection.");
  drawWrapped(4, 118, adv, 20, 3, afg, TRANSP);

  drawText(4, 152, canOpenMenu() ? "Hold L+R: menu" : "Moving: menu locked", 1, C_GREY, TRANSP);
  drawToast();
}

static const char* const MENU_ITEMS[6] = {"INCIDENTS", "LIVE DATA", "FAULT CODES", "TRIP LOG", "STATUS", "EXIT TO DRIVER"};

static void renderMenu() {
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  drawHeader("MECHANIC MODE", false);
  for (int i = 0; i < 6; i++) {
    int y = 18 + i * 22;
    bool sel = (i == menuCur);
    fillRect(4, y, SCREEN_W - 8, 19, sel ? C_WHITE : C_DGREY);
    drawText(12, y + 6, MENU_ITEMS[i], 1, sel ? C_BLACK : C_WHITE, TRANSP);
  }
  drawText(4, 152, "L/R move  OK select", 1, C_GREY, TRANSP);
  drawToast();
}

static bool metaMatches(const IncMeta& m) {
  if (!search[0]) return true;
  char hay[160]; int n = snprintf(hay, sizeof(hay), "%u %04u ", (unsigned)m.id, (unsigned)m.id);
  for (int f = 1; f < F_COUNT; f++) if (m.mask & (1UL << f)) n += snprintf(hay + n, sizeof(hay) - n, "%s ", FI[f].shortName);
  snprintf(hay + n, sizeof(hay) - n, "%s", m.sev >= 2 ? "CRIT RED" : "WARN YELLOW");
  return strstr(hay, search) != nullptr;
}
static void rebuildFilter() {
  viewN = 0;
  for (int i = metaN - 1; i >= 0; i--) if (metaMatches(meta[i])) viewIdx[viewN++] = i;
  if (lcur >= viewN) lcur = viewN - 1;
  if (ltop > lcur) ltop = lcur < 0 ? 0 : lcur;
}

static void renderIncList() {
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  char b[40];
  snprintf(b, sizeof(b), "INCIDENTS %d", viewN);
  drawHeader(b, true);
  bool sbSel = (lcur == -1);
  fillRect(0, 14, SCREEN_W, 12, typing ? C_WHITE : (sbSel ? C_SKY : C_DGREY));
  uint16_t sfg = typing ? C_BLACK : C_WHITE;
  snprintf(b, sizeof(b), "Find: %s%s", search, typing ? "_" : "");
  drawText(3, 17, b, 1, sfg, TRANSP);
  if (typing && mlen) drawText(SCREEN_W - 6 * (mlen + 1), 17, mbuf, 1, C_RED, TRANSP);
  if (viewN == 0) {
    drawText(8, 60, metaN ? "No match" : "No incidents yet", 1, C_GREY, TRANSP);
  }
  for (int r = 0; r < 5; r++) {
    int idx = ltop + r;
    if (idx >= viewN) break;
    const IncMeta& m = meta[viewIdx[idx]];
    int y = 27 + r * 24;
    bool sel = (idx == lcur);
    fillRect(0, y, SCREEN_W, 23, sel ? C_NAVY : C_BLACK);
    if (sel) drawRect(0, y, SCREEN_W, 23, C_WHITE);
    fillRect(2, y + 2, 4, 19, m.sev >= 2 ? C_RED : C_YELLOW);
    snprintf(b, sizeof(b), "#%03u %s", (unsigned)m.id, FI[m.first].shortName);
    drawText(9, y + 3, b, 1, C_WHITE, TRANSP);
    snprintf(b, sizeof(b), "%.0fkm/h", m.spd10 / 10.0f);
    drawText(SCREEN_W - textW(b, 1) - 3, y + 3, b, 1, C_CYAN, TRANSP);
    char tm[28]; fmtClock(m.epoch, m.trigMs, tm, sizeof(tm));
    drawText(9, y + 13, tm, 1, C_GREY, TRANSP);
  }
  if (typing) drawText(2, 150, "./- morse L:del R:ok", 1, C_GREY, TRANSP);
  else drawText(2, 150, "L/R move  OK open", 1, C_GREY, TRANSP);
  drawToast();
}

static void renderIncDetail() {
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  const IncMeta& m = meta[detMeta];
  char b[48], tm[28];
  snprintf(b, sizeof(b), "INCIDENT #%u", (unsigned)m.id);
  drawHeader(b, true);
  fillRect(0, 14, SCREEN_W, 11, m.sev >= 2 ? C_RED : C_YELLOW);
  fmtClock(m.epoch, m.trigMs, tm, sizeof(tm));
  drawText(3, 16, m.sev >= 2 ? "CRIT" : "WARN", 1, m.sev >= 2 ? C_WHITE : C_BLACK, TRANSP);
  drawText(33, 16, tm, 1, m.sev >= 2 ? C_WHITE : C_BLACK, TRANSP);
  char names[64] = ""; int n = 0;
  for (int f = 1; f < F_COUNT; f++) if (m.mask & (1UL << f)) n += snprintf(names + n, sizeof(names) - n, "%s%s", n ? "," : "", FI[f].shortName);
  drawWrapped(2, 28, names, 21, 2, C_YELLOW, TRANSP);
  drawText(2, 50, "-- AT FAULT TIME --", 1, C_GREY, TRANSP);
  if (viewCount > 0 && viewTrig >= 0) {
    const Sample& s = viewBuf[viewTrig];
    int y = 60;
    for (int i = 0; i < NSIG; i++) {
      float v = sigVal(s, i);
      char val[24];
      if (i == 2 || i == 0 || i == 4) snprintf(val, sizeof(val), "%.1f %s", v, SIG_UNIT[i]);
      else snprintf(val, sizeof(val), "%.0f %s", v, SIG_UNIT[i]);
      drawText(4, y, SIG_NAME[i], 1, C_GREY, TRANSP);
      drawText(70, y, val, 1, valColor(i, v), TRANSP);
      y += 10;
    }
  } else {
    drawText(4, 70, "Data not readable", 1, C_RED, TRANSP);
  }
  fillRect(10, 142, SCREEN_W - 20, 16, C_SKY);
  drawTextC(SCREEN_W / 2, 146, "REPLAY GRAPH", 1, C_BLACK, TRANSP);
  drawToast();
}

static void renderReplay() {
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  drawHeader(viewTitle, true);
  if (viewCount < 2) { drawText(10, 60, "No data yet", 1, C_WHITE, TRANSP); drawToast(); return; }
  const int GY = 14, GH = 74;
  uint16_t col = SIG_COL[vsig];
  float lo = SIG_LO[vsig], hi = SIG_HI[vsig];
  fillRect(0, GY, SCREEN_W, GH, C_DGREY);
  for (int k = 0; k < 2; k++) {                       // warn / critical lines
    float t = k == 0 ? SIG_WARN[vsig] : SIG_CRIT[vsig];
    if (t < 0) continue;
    int y = GY + GH - 1 - (int)((clampf(t, lo, hi) - lo) / (hi - lo) * (GH - 1));
    for (int x = 0; x < SCREEN_W; x += 4) fillRect(x, y, 2, 1, k == 0 ? C_YELLOW : C_RED);
  }
  for (int c = 0; c < SCREEN_W; c++) {
    int i0 = (int)((long)c * viewCount / SCREEN_W), i1 = (int)((long)(c + 1) * viewCount / SCREEN_W);
    if (i1 <= i0) i1 = i0 + 1;
    if (i1 > viewCount) i1 = viewCount;
    if (i0 >= viewCount) i0 = viewCount - 1;
    float mn = 1e9f, mx = -1e9f;
    for (int i = i0; i < i1; i++) { float v = sigVal(viewBuf[i], vsig); if (v < mn) mn = v; if (v > mx) mx = v; }
    int y1 = GY + GH - 1 - (int)((clampf(mx, lo, hi) - lo) / (hi - lo) * (GH - 1));
    int y2 = GY + GH - 1 - (int)((clampf(mn, lo, hi) - lo) / (hi - lo) * (GH - 1));
    fillRect(c, y1, 1, y2 - y1 + 1, col);
  }
  if (viewTrig >= 0) {
    int tc = (int)((long)viewTrig * SCREEN_W / viewCount);
    fillRect(tc, GY, 1, GH, C_RED);
  }
  fillRect(curCol, GY, 1, GH, C_WHITE);
  char b[40];
  snprintf(b, sizeof(b), "%s %s", SIG_NAME[vsig], SIG_UNIT[vsig]);
  drawText(2, GY + 2, b, 1, col, TRANSP);
  snprintf(b, sizeof(b), "%.0f", hi); drawText(SCREEN_W - textW(b, 1) - 2, GY + 2, b, 1, C_GREY, TRANSP);
  snprintf(b, sizeof(b), "%.0f", lo); drawText(SCREEN_W - textW(b, 1) - 2, GY + GH - 9, b, 1, C_GREY, TRANSP);

  int ci = (int)((long)curCol * viewCount / SCREEN_W); if (ci >= viewCount) ci = viewCount - 1;
  const Sample& s = viewBuf[ci];
  float dt = viewTrig >= 0 ? (float)((int32_t)(s.t - viewBuf[viewTrig].t)) / 1000.0f
                           : (float)(ci - (viewCount - 1)) * (float)TRIP_EVERY / 10.0f;
  snprintf(b, sizeof(b), "%s%+.1fs", viewTrig >= 0 ? "FAULT " : "NOW ", dt);
  drawText(2, 92, b, 1, C_WHITE, TRANSP);
  snprintf(b, sizeof(b), "SPD %.0f  RPM %u", s.spd10 / 10.0f, (unsigned)s.rpm);          drawText(2, 102, b, 1, C_CYAN, TRANSP);
  snprintf(b, sizeof(b), "CLT %.1f  OIL %u", s.clt10 / 10.0f, (unsigned)s.oil);          drawText(2, 112, b, 1, valColor(2, s.clt10 / 10.0f), TRANSP);
  snprintf(b, sizeof(b), "BAT %.1f  FUEL %u", s.bat100 / 100.0f, (unsigned)s.fuel);      drawText(2, 122, b, 1, valColor(4, s.bat100 / 100.0f), TRANSP);
  snprintf(b, sizeof(b), "LOAD %u  THR %u%s%s", (unsigned)s.load, (unsigned)s.thr,
           (s.flags & FLAG_DERATE) ? " DERATE" : "", (s.flags & FLAG_MIL) ? " MIL" : "");
  drawText(2, 132, b, 1, C_WHITE, TRANSP);
  drawText(2, 144, "L/R scrub  OK signal", 1, C_GREY, TRANSP);
  drawText(2, 152, "hold OK: back", 1, C_GREY, TRANSP);
  drawToast();
}

static void renderLive() {
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  drawHeader("LIVE DATA", true);
  for (int i = 0; i < NSIG; i++) {
    int y = 17 + i * 14;
    float v = sigVal(latest, i);
    char b[24];
    drawText(3, y, SIG_NAME[i], 1, C_GREY, TRANSP);
    if (i == 2 || i == 0 || i == 4) snprintf(b, sizeof(b), "%.1f", v); else snprintf(b, sizeof(b), "%.0f", v);
    drawText(58, y, b, 1, valColor(i, v), TRANSP);
    int bw = (int)((clampf(v, SIG_LO[i], SIG_HI[i]) - SIG_LO[i]) / (SIG_HI[i] - SIG_LO[i]) * 40.0f);
    fillRect(86, y + 1, 40, 6, C_DGREY);
    fillRect(86, y + 1, bw, 6, SIG_COL[i]);
  }
  char b[40];
  snprintf(b, sizeof(b), "MIL:%s  DERATE:%s", (latest.flags & FLAG_MIL) ? "ON" : "off", (latest.flags & FLAG_DERATE) ? "ON" : "off");
  drawText(3, 132, b, 1, (latest.flags & (FLAG_MIL | FLAG_DERATE)) ? C_YELLOW : C_GREY, TRANSP);
  uint32_t age = haveData ? (millis() - lastDataMs) : 0;
  snprintf(b, sizeof(b), "ECU %s  age %ums", linkOk() ? "OK" : "LOST", (unsigned)age);
  drawText(3, 144, b, 1, linkOk() ? C_GREEN : C_RED, TRANSP);
  drawToast();
}

static void renderFaults() {
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  drawHeader("FAULT CODES", true);
  for (int f = 1; f < F_COUNT; f++) {
    int y = 17 + (f - 1) * 14;
    uint16_t c = fActive[f] ? (fSev[f] >= 2 ? C_RED : C_YELLOW) : C_DGREY;
    fillRect(3, y, 8, 8, c);
    drawText(15, y, FI[f].title, 1, fActive[f] ? C_WHITE : C_GREY, TRANSP);
    char b[12]; snprintf(b, sizeof(b), "x%u", (unsigned)fCount[f]);
    drawText(SCREEN_W - textW(b, 1) - 3, y, b, 1, fCount[f] ? C_YELLOW : C_GREY, TRANSP);
  }
  drawText(3, 146, "box=active  xN=times", 1, C_GREY, TRANSP);
  drawToast();
}

static void renderStatus() {
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  drawHeader("STATUS", true);
  char b[40]; int y = 18;
  snprintf(b, sizeof(b), "FLASH %u/%u KB", (unsigned)(LittleFS.usedBytes() / 1024), (unsigned)(LittleFS.totalBytes() / 1024));
  drawText(3, y, b, 1, C_WHITE, TRANSP); y += 12;
  snprintf(b, sizeof(b), "INCIDENTS %d/%d", metaN, MAX_INC);
  drawText(3, y, b, 1, C_WHITE, TRANSP); y += 12;
  snprintf(b, sizeof(b), "ECU: %s", !haveData ? "no data yet" : (linkOk() ? "link OK (SIM)" : "LINK LOST"));
  drawText(3, y, b, 1, linkOk() ? C_GREEN : C_RED, TRANSP); y += 12;
  drawText(3, y, epochBase ? "TIME: synced from PC" : "TIME: since boot only", 1, epochBase ? C_GREEN : C_YELLOW, TRANSP); y += 12;
  const char* ts = !tgEnabled ? "TG: off (no token)" : (tgState == 2 ? "TG: online" : (tgState == 3 ? "TG: send failed" : "TG: no WiFi"));
  drawText(3, y, ts, 1, tgState == 2 ? C_GREEN : C_YELLOW, TRANSP); y += 12;
  if (tgEnabled) { snprintf(b, sizeof(b), "TG sent %u drop %u", (unsigned)tgSent, (unsigned)tgDropped); drawText(3, y, b, 1, C_GREY, TRANSP); }
  y += 14;
  if (eraseArmed) {
    fillRect(3, 112, SCREEN_W - 6, 34, C_RED);
    drawTextC(SCREEN_W / 2, 116, "ERASE ALL LOGS?", 1, C_WHITE, TRANSP);
    drawTextC(SCREEN_W / 2, 128, "OK again = YES", 1, C_WHITE, TRANSP);
    drawTextC(SCREEN_W / 2, 137, "any other key = no", 1, C_WHITE, TRANSP);
  } else {
    drawText(3, 150, "OK: erase all logs", 1, C_GREY, TRANSP);
  }
  drawToast();
}

static void render() {
  switch (screen) {
    case S_DRIVER:     renderDriver(); break;
    case S_MENU:       renderMenu(); break;
    case S_INC_LIST:   renderIncList(); break;
    case S_INC_DETAIL: renderIncDetail(); break;
    case S_REPLAY:     renderReplay(); break;
    case S_LIVE:       renderLive(); break;
    case S_FAULTS:     renderFaults(); break;
    case S_STATUS:     renderStatus(); break;
  }
  flushScreen();
}

// ============================ 16. UI ACTIONS ==============================
static void goScreen(Screen s) { screen = s; eraseArmed = false; typing = false; dirty = true; }

static void openIncidentList() {
  lcur = -1; ltop = 0; typing = false; mlen = 0; mbuf[0] = 0;
  rebuildFilter();
  goScreen(S_INC_LIST);
}
static void openDetail(int viewPos) {
  if (viewPos < 0 || viewPos >= viewN) return;
  int mi = viewIdx[viewPos];
  if (!loadIncidentToView(mi)) { toast("Cannot read file"); return; }
  detMeta = mi; lcur = viewPos;
  goScreen(S_INC_DETAIL);
}
static void openReplay() { if (viewCount > 1) { vsig = 0; goScreen(S_REPLAY); } else toast("No data"); }

static void menuSelect() {
  switch (menuCur) {
    case 0: openIncidentList(); break;
    case 1: goScreen(S_LIVE); break;
    case 2: goScreen(S_FAULTS); break;
    case 3: if (loadTripToView()) { vsig = 0; goScreen(S_REPLAY); } else toast("Trip log empty"); break;
    case 4: goScreen(S_STATUS); break;
    default: goScreen(S_DRIVER); break;
  }
}

static void uiBack() {
  switch (screen) {
    case S_MENU:       goScreen(S_DRIVER); break;
    case S_INC_LIST:   goScreen(S_MENU); break;
    case S_INC_DETAIL: goScreen(S_INC_LIST); break;
    case S_REPLAY:     goScreen(viewIsTrip ? S_MENU : S_INC_DETAIL); break;
    case S_LIVE: case S_FAULTS: case S_STATUS: goScreen(S_MENU); break;
    default: break;
  }
}

static void uiLeft(bool rep) {
  switch (screen) {
    case S_MENU: if (menuCur > 0) menuCur--; break;
    case S_INC_LIST:
      if (typing) {                                   // backspace
        if (mlen) { mlen = 0; mbuf[0] = 0; }
        else { int n = (int)strlen(search); if (n) { search[n - 1] = 0; rebuildFilter(); } }
      } else if (lcur > -1) {
        lcur--; if (lcur < ltop) ltop = lcur < 0 ? 0 : lcur;
      }
      break;
    case S_INC_DETAIL: {                              // previous (newer) incident
      int pos = lcur - 1; if (pos >= 0) openDetail(pos);
      return;
    }
    case S_REPLAY: curCol -= rep ? 4 : 1; if (curCol < 0) curCol = 0; break;
    default: break;
  }
  dirty = true;
}

static void uiRight(bool rep) {
  switch (screen) {
    case S_MENU: if (menuCur < 5) menuCur++; break;
    case S_INC_LIST:
      if (typing) { typing = false; mlen = 0; mbuf[0] = 0; }
      else if (lcur < viewN - 1) { lcur++; if (lcur >= ltop + 5) ltop = lcur - 4; }
      break;
    case S_INC_DETAIL: {                              // next (older) incident
      int pos = lcur + 1; if (pos < viewN) openDetail(pos);
      return;
    }
    case S_REPLAY: curCol += rep ? 4 : 1; if (curCol > SCREEN_W - 1) curCol = SCREEN_W - 1; break;
    default: break;
  }
  dirty = true;
}

static void uiOk() {
  switch (screen) {
    case S_MENU: menuSelect(); break;
    case S_INC_LIST:
      if (lcur == -1) { typing = true; mlen = 0; mbuf[0] = 0; dirty = true; }
      else openDetail(lcur);
      break;
    case S_INC_DETAIL: openReplay(); break;
    case S_REPLAY: vsig = (vsig + 1) % NSIG; dirty = true; break;
    case S_STATUS:
      if (eraseArmed) { eraseAllLogs(); eraseArmed = false; toast("All logs erased"); }
      else eraseArmed = true;
      dirty = true; break;
    default: break;
  }
}

static void morseKey(uint32_t dur) {
  if (dur >= 1500) { typing = false; mlen = 0; mbuf[0] = 0; dirty = true; return; }
  if (mlen < 6) { mbuf[mlen++] = dur < 300 ? '.' : '-'; mbuf[mlen] = 0; }
  mLast = millis(); dirty = true;
}
static void morseTick(uint32_t now) {
  if (!(screen == S_INC_LIST && typing && mlen > 0 && now - mLast > 800)) return;
  int found = -1;
  for (int i = 0; i < 36; i++) if (strcmp(MORSE_TAB[i], mbuf) == 0) { found = i; break; }
  if (found >= 0) {
    int n = (int)strlen(search);
    if (n < 12) { search[n] = found < 26 ? (char)('A' + found) : (char)('0' + found - 26); search[n + 1] = 0; }
    rebuildFilter();
  } else toast("Bad morse code");
  mlen = 0; mbuf[0] = 0; dirty = true;
}

static void uiTap(int x, int y) {
  if (screen != S_DRIVER && y < 13 && x < 28) { uiBack(); return; }
  if (eraseArmed && screen == S_STATUS) { eraseArmed = false; dirty = true; return; }
  switch (screen) {
    case S_DRIVER:
      if (canOpenMenu()) { menuCur = 0; goScreen(S_MENU); } else toast("Stop vehicle first");
      break;
    case S_MENU: {
      int r = (y - 18) / 22;
      if (y >= 18 && r >= 0 && r < 6) { menuCur = r; menuSelect(); }
      break;
    }
    case S_INC_LIST:
      if (y >= 14 && y < 26) { lcur = -1; typing = true; mlen = 0; mbuf[0] = 0; dirty = true; }
      else if (y >= 27 && y < 147) {
        int idx = ltop + (y - 27) / 24;
        if (idx < viewN) openDetail(idx);
      }
      break;
    case S_INC_DETAIL: if (y >= 138) openReplay(); break;
    case S_REPLAY:
      if (y >= 14 && y < 88) { curCol = x; dirty = true; }
      else if (y >= 88) { vsig = (vsig + 1) % NSIG; dirty = true; }
      break;
    case S_STATUS: if (y >= 140) uiOk(); break;
    default: break;
  }
}
static void uiDrag(int x, int y) {
  if (screen == S_REPLAY && y >= 10 && y < 92) { curCol = x < 0 ? 0 : (x > SCREEN_W - 1 ? SCREEN_W - 1 : x); dirty = true; }
}

// ============================ 17. INPUT POLLING ===========================
static int btnUpdate(Btn& b, uint32_t now) {         // 1 = pressed, 2 = released
  bool r = (digitalRead(b.pin) == LOW);
  if (r != b.raw) { b.raw = r; b.tEdge = now; }
  if ((now - b.tEdge) >= 30 && b.down != b.raw) {
    b.down = b.raw;
    if (b.down) { b.tDown = now; b.lastRep = now; return 1; }
    return 2;
  }
  return 0;
}

static void handleButtons(uint32_t now) {
  int el = btnUpdate(bL, now), er = btnUpdate(bR, now), em = btnUpdate(bM, now);
  if (supL && !bL.down) supL = false;
  if (supR && !bR.down) supR = false;

  if (screen == S_DRIVER) {
    if (bL.down && bR.down) {
      if (!comboArmed) { comboArmed = true; comboStart = now; }
      else if (!comboFired && now - comboStart > 500) {
        comboFired = true;
        if (canOpenMenu()) { supL = supR = true; menuCur = 0; goScreen(S_MENU); }
        else toast("Stop vehicle first");
      }
    } else { comboArmed = false; comboFired = false; }
    if (em == 2) { fMuted = true; toast("Buzzer muted"); }
    return;
  }

  bool repL = false, repR = false;
  if (bL.down && !supL && now - bL.tDown > 400 && now - bL.lastRep > 80) { bL.lastRep = now; repL = true; }
  if (bR.down && !supR && now - bR.tDown > 400 && now - bR.lastRep > 80) { bR.lastRep = now; repR = true; }
  if (!supL && (el == 1 || repL)) { eraseArmed = false; uiLeft(repL); }
  if (!supR && (er == 1 || repR)) { eraseArmed = false; uiRight(repR); }
  if (em == 2) {
    uint32_t d = now - bM.tDown;
    if (screen == S_INC_LIST && typing) morseKey(d);
    else if (d >= 700) uiBack();
    else uiOk();
  }
}

static void touchPoll(uint32_t now) {
#if USE_TOUCH
  if (now - tLastPoll < 20) return;
  tLastPoll = now;
  int x = 0, y = 0;
  bool p = touchRead(x, y);
  if (p) {
    tUp = 0; if (tCnt < 10) tCnt++;
    if (tCnt >= 2) {
      if (!tDown) { tDown = true; tStart = now; tx0 = x; ty0 = y; tMoved = false; }
      else if (abs(x - tx0) > 6 || abs(y - ty0) > 6) { tMoved = true; uiDrag(x, y); }
    }
  } else {
    tCnt = 0;
    if (tDown && ++tUp >= 2) {
      tDown = false; tUp = 0;
      if (!tMoved && now - tStart < 900) uiTap(tx0, ty0);
    }
  }
#else
  (void)now;
#endif
}

static void buzzerTick(uint32_t now) {
#if PIN_BUZZER >= 0
  uint8_t sev = overallSev();
  bool on = false;
  if (sev && !fMuted) on = (sev == 2) ? (((now / 150) % 2) == 0) : ((now % 2000) < 100);
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
#else
  (void)now;
#endif
}

// ============================ 18. SETUP / LOOP ============================
void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_MORSE, INPUT_PULLUP);
#if PIN_BUZZER >= 0
  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);
#endif
#if USE_TOUCH
  pinMode(PIN_T_CS, OUTPUT); digitalWrite(PIN_T_CS, HIGH);
  pinMode(PIN_T_CLK, OUTPUT); digitalWrite(PIN_T_CLK, LOW);
  pinMode(PIN_T_DIN, OUTPUT);
  pinMode(PIN_T_DO, INPUT);
  pinMode(PIN_T_IRQ, INPUT_PULLUP);
#endif

  fb = (uint16_t*)malloc((size_t)SCREEN_W * SCREEN_H * 2);
  if (!fb) { Serial.println("#ERROR: not enough RAM for frame buffer"); while (true) delay(1000); }
  tftInit();
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_BLACK);
  drawTextC(SCREEN_W / 2, 60, "TRUCK", 2, C_WHITE, TRANSP);
  drawTextC(SCREEN_W / 2, 80, "BLACK BOX", 2, C_YELLOW, TRANSP);
  drawTextC(SCREEN_W / 2, 110, "starting...", 1, C_GREY, TRANSP);
  flushScreen();

  bool fsOk = LittleFS.begin(true);
  if (fsOk) { LittleFS.mkdir("/inc"); scanIncidents(); }
  Serial.printf("#BOOT flash %s, %d incidents, %u KB free\n", fsOk ? "OK" : "FAILED", metaN, fsOk ? (unsigned)(fsFree() / 1024) : 0u);

  tgEnabled = (strncmp(TG_BOT_TOKEN, "PASTE", 5) != 0);
  if (tgEnabled) {
    tgQ = xQueueCreate(5, sizeof(TgMsg));
    xTaskCreatePinnedToCore(tgTask, "tg", 16384, NULL, 1, NULL, 0);
    char msg[160];
    snprintf(msg, sizeof(msg), "\xE2\x9C\x85 Truck Black Box is online\n\xF0\x9F\x92\xBE %d incidents stored, %u KB flash free",
             metaN, fsOk ? (unsigned)(fsFree() / 1024) : 0u);
    tgEnqueue(fsOk ? msg : "\xF0\x9F\x92\xBE\xE2\x9D\x8C Truck Black Box started but FLASH STORAGE FAILED");
  } else {
    Serial.println("#Telegram is OFF (token not set)");
  }
  memset(&latest, 0, sizeof(latest));
  delay(800);
  dirty = true;
}

void loop() {
  uint32_t now = millis();
  serialPump();
  linkCheck(now);
  if (incState == INC_REC && fActive[F_LINK_LOST]) finishIncident();
  handleButtons(now);
  touchPoll(now);
  morseTick(now);
  buzzerTick(now);

  // mechanic screens are only for a stopped vehicle
  if (screen != S_DRIVER && !ALLOW_MENU_WHILE_MOVING && haveData && (latest.spd10 / 10.0f) >= MECH_SPEED_LIMIT + 3.0f) {
    goScreen(S_DRIVER); toast("Vehicle moving");
  }
  if (toastMsg[0] && millis() > toastUntil) { toastMsg[0] = 0; dirty = true; }

  uint32_t period = (screen == S_DRIVER || screen == S_LIVE) ? 250 : 1000;
  if (screen == S_STATUS || screen == S_FAULTS) period = 500;
  if ((dirty && now - lastRender >= 60) || (now - lastRender >= period && (screen == S_DRIVER || screen == S_LIVE || screen == S_STATUS))) {
    render(); lastRender = now; dirty = false;
  }
  delay(1);
}
