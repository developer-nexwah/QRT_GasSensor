//Version 23/03/2026 (rogue detection for O2 & CO2, ignore timeouts, new COZIR sensor)
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_log.h"
#include <SPI.h>
#include "esp_system.h"  // for esp_restart()

Preferences prefs;
#define ENABLE_DISPLAY 1   // set to 0 to disable the OLED display

// 硬件配置
#define DEBUG_BAUDRATE 9600
#define SENSOR_BAUDRATE 9600  
#define VOC_BAUDRATE 9600
#define MAX_SMA_PERIOD 50
#define SENSOR_TIMEOUT 2000  // 传感器响应超时(ms)

// Plausibility limits
#define O2_MIN 0.0
#define O2_MAX 30.0        // % (normal air ~20.9%)
#define CO2_MIN 0.0
#define CO2_MAX 99999.0     // ppm (raw sensor value, later divided by 1000 for %)
#define CO2_MODE0_PPM_MULTIPLIER 10.0f
#define COZIR_MODE2_PPM_MULTIPLIER 1.0f

// DAC
#define O2_DAC 25
#define CO2_DAC 26
#define VOC_DAC 25  //25 use back O2 pin for sharing port
// UART config
#define U1_TX 14
#define U1_RX 13
#define U2_TX 17
#define U2_RX 16
// Display config
#define I2C_SDA1 21
#define I2C_SCL1 22

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool display_ok;
bool command_mode = false; // Type CMD before readings, modes or configuration
bool config_mode = false;
// Operation mode: 0 = O2/CO2 mode, 1 = VOC mode, 2 = COZIR
int operation_mode = 0;
#define MODE_COZIR 2

// 硬件串口定义
#define CO2_UART Serial1
#define O2_UART Serial2
#define VOC_UART Serial2  // Using the same UART as O2 for VOC
#define COZIR_UART Serial2 // Using the same UART as O2 for COZIR-LP-5000

enum OutputMode { RAW,
                  SMA,
                  EMA };

struct SensorConfig {
  bool enabled;  // 启用状态
  float offset;  // 校准偏移
  float gain;
  float alpha;
  float p;
  unsigned long sample_interval;  // 采样间隔
  unsigned long output_interval;  // 输出间隔
  unsigned long last_sample;
  unsigned long last_output;
  float current_value;
  String name;  // 传感器名称

  OutputMode output_mode;  // 输出模式
  int sma_period;          // SMA窗口
  float sma_buffer[MAX_SMA_PERIOD];
  int buffer_index;
  int buffer_count;

  int ema_period;   // EMA周期
  float ema_alpha;  // 平滑系数
  float current_ema;

  // Error tracking for O2/CO2
  int consecutiveErrors;
  int resetThreshold;
};

// Structure to hold parsed VOC data
struct VOCResponseData {
  uint8_t command_type;
  uint8_t sensor_type;
  uint16_t range;
  float concentration1;
  float concentration2;
  float temperature;
  float humidity;
  uint8_t unit_type;
  uint8_t decimal_places;
  uint8_t led_status;
  bool valid;
};

// 传感器全局实例
SensorConfig co2 = { true, 0, 1.0, 0.0, 0.0, 3000, 1000, 0, 0, 0, "CO2", RAW, 5, {}, 0, 0, 10, 0.18, 0, 0, 0 };
SensorConfig o2 = { true, 0, 1.0, 0.0, 1000, 1000, 1000, 0, 0, 0, "O2", RAW, 5, {}, 0, 0, 10, 0.18, 0, 0, 0 };
SensorConfig voc = { true, 0, 1.0, 0.0, 0.0, 1000, 1000, 0, 0, 0, "VOC", RAW, 5, {}, 0, 0, 10, 0.18, 0, 0, 0 };
SensorConfig cozir = { true, 0, 1.0, 0.0, 0.0, 1000, 1000, 0, 0, 0, "COZIR", RAW, 5, {}, 0, 0, 10, 0.18, 0, 0, 0 };

// VOC sensor specific variables
uint8_t voc_response_buffer[20];
int voc_response_index = 0;
bool voc_auto_reporting = false;
float co2_display = 0.0;
float o2_display = 0.0;
// Separate validity flags: a missing sensor must not stop the other channel.
bool co2_has_reading = false;
bool o2_has_reading = false;
uint8_t co2_failed_reads = 0;
uint8_t o2_failed_reads = 0;
const uint8_t SENSOR_MISSING_AFTER = 2;
// Polling a disconnected UART can otherwise block the other sensor for 2 s.
// Keep the original SENSOR_TIMEOUT for SST calibration acknowledgement.
const unsigned long LIVE_POLL_TIMEOUT_MS = 750UL;

// ESP32 DACs are 8-bit outputs (GPIO25 O2, GPIO26 CO2).
// Preserve the original 0..25% => 0..199 DAC counts mapping.
void updateGasDac(const String &sensor, float percent) {
  if (!isfinite(percent)) return;
  const uint8_t code = (uint8_t)constrain(lroundf(percent * 199.0f / 25.0f), 0L, 199L);
  if (sensor == "O2") dacWrite(O2_DAC, code);
  else if (sensor == "CO2") dacWrite(CO2_DAC, code);
}

void handleLiveSensorFailure(const String &sensor) {
  uint8_t &errors = (sensor == "O2") ? o2_failed_reads : co2_failed_reads;
  bool &valid = (sensor == "O2") ? o2_has_reading : co2_has_reading;
  if (errors < 255) ++errors;
  if (errors >= SENSOR_MISSING_AFTER && valid) {
    valid = false;
    // Safe low output on missing data; DO NOT interpret as a valid 0% measurement.
    if (sensor == "O2") dacWrite(O2_DAC, 0);
    else dacWrite(CO2_DAC, 0);
    Serial.printf("[%s] NO SENSOR DATA; DAC output set to 0 (invalid). Other channel continues.\n", sensor.c_str());
  }
}


// ============================================
// CO2 TIMED CALIBRATION (SST X command)
// ============================================
#define TCAL_MAX_SAMPLES 120
bool tcal_enabled = false;
bool tcal_known_set = true;
// Guard SST zero-point writes until the physical known gas is confirmed.
unsigned int tcal_target_samples = 5;  // Factory default: 5 readings; saved CMD overrides take precedence
float var_Tolerance = 2.0f;             // relative percent
float var_Known_CO2 = 20.9f;            // % (CUSTOMER REQUEST; VERIFY - 20.9% is normally ambient O2)
const float CO2_PPM_MULTIPLIER = 10.0f; // verified response: . 00010
float co2_samples[TCAL_MAX_SAMPLES];
int sample_count = 0;
unsigned long sample_start_time = 0;
unsigned long last_tcal_sample = 0;
bool tcal_first_check_pending = false;
bool tcal_countdown_active = false;
unsigned long tcal_last_countdown_print = 0;
unsigned int tcal_cycle_number = 0;

// Requirement 4: repeating automatic calibration scheduler.
// Automatic calibration always follows the live SST calibration sequence.
bool tcal_auto_enabled = true;  // Factory default ON; saved customer ON/OFF selection overrides this
bool tcal_auto_use_seconds = false;
bool tcal_auto_use_minutes = true;  // Default: 1 minute for testing
uint32_t tcal_auto_seconds = 10;
uint32_t tcal_auto_minutes = 1;
uint16_t tcal_auto_days = 3;
uint32_t tcal_auto_started_ms = 0;
bool tcal_auto_cycle_running = false;
// Batch qualification: N valid readings, ALL compared to the known reference.
// After a failed batch, wait for the configurable delay before trying again.
bool tcal_retry_waiting = false;
bool tcal_batch_passed = false;
uint32_t tcal_delay_seconds = 10UL;  // default retry delay: 10 seconds
uint32_t tcal_delay_started_ms = 0;
bool tcal_auto_start_now = false;
String tcal_last_sample_label = "";
uint32_t tcal_last_sample_notice_until = 0;
// OLED status shown briefly after a calibration result.
String tcal_oled_notice = "";
uint32_t tcal_oled_notice_until = 0;

// Store each CMD override independently; other settings keep their current values.
// TCAL settings use a dedicated Preferences instance and NVS namespace.
// Each changed CMD setting is committed immediately to ESP32 flash.
void saveTcalSettings() {
  Preferences tcalNvs;
  if (!tcalNvs.begin("tcal_cfg", false)) {
    Serial.println("[TCAL NVS] ERROR: cannot open settings for writing.");
    return;
  }

  bool ok = true;
  ok &= (tcalNvs.putFloat("tol", var_Tolerance) == sizeof(float));
  ok &= (tcalNvs.putFloat("known", var_Known_CO2) == sizeof(float));
  ok &= (tcalNvs.putBool("known_set", tcal_known_set) > 0);
  ok &= (tcalNvs.putUInt("samples", tcal_target_samples) == sizeof(uint32_t));
  ok &= (tcalNvs.putULong("delay_s", tcal_delay_seconds) == sizeof(uint32_t));
  ok &= (tcalNvs.putULong("auto_min", tcal_auto_minutes) == sizeof(uint32_t));
  ok &= (tcalNvs.putULong("auto_sec", tcal_auto_seconds) == sizeof(uint32_t));
  ok &= (tcalNvs.putUInt("auto_days", tcal_auto_days) == sizeof(uint32_t));
  ok &= (tcalNvs.putBool("use_min", tcal_auto_use_minutes) > 0);
  ok &= (tcalNvs.putBool("use_sec", tcal_auto_use_seconds) > 0);
  ok &= (tcalNvs.putBool("auto_on", tcal_auto_enabled) > 0);

  // Verify ALL saved calibration parameters, not only tolerance and samples.
  ok &= (fabsf(tcalNvs.getFloat("tol", NAN) - var_Tolerance) < 0.0001f);
  ok &= (fabsf(tcalNvs.getFloat("known", NAN) - var_Known_CO2) < 0.0001f);
  ok &= (tcalNvs.getBool("known_set", !tcal_known_set) == tcal_known_set);
  ok &= (tcalNvs.getUInt("samples", 0) == tcal_target_samples);
  ok &= (tcalNvs.getULong("delay_s", 0) == tcal_delay_seconds);
  ok &= (tcalNvs.getULong("auto_min", 0) == tcal_auto_minutes);
  ok &= (tcalNvs.getULong("auto_sec", 0) == tcal_auto_seconds);
  ok &= (tcalNvs.getUInt("auto_days", 0) == tcal_auto_days);
  ok &= (tcalNvs.getBool("use_min", !tcal_auto_use_minutes) == tcal_auto_use_minutes);
  ok &= (tcalNvs.getBool("use_sec", !tcal_auto_use_seconds) == tcal_auto_use_seconds);
  ok &= (tcalNvs.getBool("auto_on", !tcal_auto_enabled) == tcal_auto_enabled);
  tcalNvs.end();
  Serial.println(ok ? "[TCAL NVS] Settings saved to flash."
                    : "[TCAL NVS] WARNING: settings could not be fully verified.");
}

void loadTcalSettings() {
  Preferences tcalNvs;
  if (!tcalNvs.begin("tcal_cfg", true)) {
    Serial.println("[TCAL NVS] No saved settings available; using defaults.");
    // The global initial values are the factory defaults.
    tcal_auto_enabled = true;
    return;
  }
  var_Tolerance = tcalNvs.getFloat("tol", 2.0f);
  var_Known_CO2 = tcalNvs.getFloat("known", 20.9f);
  tcal_known_set = tcalNvs.getBool("known_set", true);
  tcal_target_samples = tcalNvs.getUInt("samples", 5U);
  tcal_delay_seconds = tcalNvs.getULong("delay_s", 10UL);
  tcal_auto_minutes = tcalNvs.getULong("auto_min", 1UL);
  tcal_auto_seconds = tcalNvs.getULong("auto_sec", 10UL);
  tcal_auto_days = (uint16_t)tcalNvs.getUInt("auto_days", 3U);
  tcal_auto_use_minutes = tcalNvs.getBool("use_min", true);
  tcal_auto_use_seconds = tcalNvs.getBool("use_sec", false);
  // New NVS key: existing boards without a saved ON/OFF preference default to ON.
  tcal_auto_enabled = tcalNvs.getBool("auto_on", true);
  tcalNvs.end();

  // Guard against invalid/corrupt saved settings.
  if (!isfinite(var_Tolerance) || var_Tolerance <= 0 || var_Tolerance > 100) var_Tolerance = 2.0f;
  if (!isfinite(var_Known_CO2) || var_Known_CO2 <= 0 || var_Known_CO2 > 99.999f) {
    var_Known_CO2 = 20.9f;
    tcal_known_set = true;
  }
  if (tcal_target_samples < 1 || tcal_target_samples > TCAL_MAX_SAMPLES) tcal_target_samples = 5;
  if (tcal_delay_seconds == 0 || tcal_delay_seconds > 86400UL) tcal_delay_seconds = 10;
  if (tcal_auto_minutes == 0 || tcal_auto_minutes > 1440UL) tcal_auto_minutes = 1;
  if (tcal_auto_seconds == 0 || tcal_auto_seconds > 86400UL) tcal_auto_seconds = 10;
  if (tcal_auto_days == 0 || tcal_auto_days > 30) tcal_auto_days = 3;
  if (tcal_auto_use_minutes) tcal_auto_use_seconds = false;

  Serial.printf("[TCAL NVS] Restored: tolerance %.2f%%, known CO2 %.3f%%, samples %u, retry %lu sec.\n",
                var_Tolerance, var_Known_CO2, tcal_target_samples,
                (unsigned long)tcal_delay_seconds);
  Serial.printf("[TCAL NVS] Restored auto interval: %lu %s (scheduler %s).\n",
                (unsigned long)(tcal_auto_use_minutes ? tcal_auto_minutes :
                    (tcal_auto_use_seconds ? tcal_auto_seconds : tcal_auto_days)),
                tcal_auto_use_minutes ? "minutes" : (tcal_auto_use_seconds ? "seconds" : "days"),
                tcal_auto_enabled ? "ON" : "OFF");
}

uint32_t tcalAutoPeriodMs() {
  if (tcal_auto_use_minutes) return tcal_auto_minutes * 60000UL;
  if (tcal_auto_use_seconds) return tcal_auto_seconds * 1000UL;
  return (uint32_t)tcal_auto_days * 86400000UL;
}

unsigned long tcalAutoPeriodValue() {
  return tcal_auto_use_minutes ? tcal_auto_minutes
       : tcal_auto_use_seconds ? tcal_auto_seconds : tcal_auto_days;
}

const char *tcalAutoPeriodUnit() {
  return tcal_auto_use_minutes ? "minutes"
       : tcal_auto_use_seconds ? "seconds" : "days";
}

void resetSampleTimer(); // used by startTcalCycle below

void startTcalCycle() {
  // No first-reading shortcut: EVERY batch collects exactly tcal_target_samples.
  tcal_enabled = true;
  tcal_first_check_pending = false;
  tcal_countdown_active = true;
  tcal_cycle_number++;
  tcal_batch_passed = false;
  tcal_last_sample_label = "";
  tcal_oled_notice = "";
  resetSampleTimer();
  // Allow first valid sample as soon as the CO2 sensor responds.
  last_tcal_sample = millis() - co2.sample_interval;
  co2.last_sample = millis() - co2.sample_interval;
  Serial.printf("[TCAL] Collecting %u CO2 samples at sensor interval %lu ms (batch %u).\n",
                tcal_target_samples, co2.sample_interval, tcal_cycle_number);
}

void updateTcalAuto(unsigned long now) {
  if (!tcal_auto_enabled) return;
  // CMD/CONFIG are settings-only. Suspend automatic TCAL transitions there.
  // On EXIT the scheduler resumes; a due cycle starts in NORMAL mode.
  if (command_mode || config_mode) return;

  if (tcal_auto_cycle_running) {
    if (tcal_enabled) return;
    tcal_auto_cycle_running = false;
    if (tcal_batch_passed) {
      tcal_retry_waiting = false;
      tcal_auto_started_ms = millis();
      Serial.printf("[TCAL AUTO] Batch PASS. Next automatic check in %lu %s.\n",
                    tcalAutoPeriodValue(), tcalAutoPeriodUnit());
    } else {
      tcal_retry_waiting = true;
      tcal_delay_started_ms = millis();
      Serial.printf("[TCAL AUTO] Batch FAIL. Retry after %lu seconds.\n",
                    (unsigned long)tcal_delay_seconds);
    }
    return;
  }

  if (tcal_enabled) return;  // Don't interrupt manual TCAL cycle.

  if (tcal_retry_waiting) {
    if ((uint32_t)(now - tcal_delay_started_ms) < tcal_delay_seconds * 1000UL) return;
    tcal_retry_waiting = false;
    Serial.println("[TCAL AUTO] Delay elapsed; collecting another batch.");
  } else if (!tcal_auto_start_now &&
             (uint32_t)(now - tcal_auto_started_ms) < tcalAutoPeriodMs()) {
    return;
  }
  tcal_auto_start_now = false;

  if (operation_mode != 0 || !co2.enabled || !tcal_known_set ||
      co2.sample_interval == 0 || tcal_target_samples < 1 ||
      tcal_target_samples > TCAL_MAX_SAMPLES) {
    Serial.printf("[TCAL AUTO] Cannot start: Mode=%d CO2=%s Reference=%s Interval=%lu ms Samples=%u. Retrying after delay.\n",
                  operation_mode, co2.enabled ? "YES" : "NO",
                  tcal_known_set ? "YES" : "NO", co2.sample_interval, tcal_target_samples);
    tcal_retry_waiting = true;
    tcal_delay_started_ms = millis();
    return;
  }
  tcal_auto_cycle_running = true;
  startTcalCycle();
}



void resetSampleTimer();
void sample_timer(float co2_percent, unsigned long current);
void timed_auto_calibration();
void handleTimedCalibrationCommand(String cmd);
void updateCalibrationCountdown(unsigned long current);
void cmdReadSensor(const String &name);
void cmdReadActiveMode();

// 前向声明
void loadSensorConfig(SensorConfig &cfg);
void printHelp();
void sensorRoutine(SensorConfig &cfg, HardwareSerial &uart, unsigned long current);
bool readSensor(HardwareSerial &uart, float &value, String sensor, unsigned long timeoutMs = SENSOR_TIMEOUT);
void vocSensorRoutine(SensorConfig &cfg, HardwareSerial &uart, unsigned long current);
void display_init();
void displayO2CO2Data(float o2_value, float co2_value);
void updateOledCountdown(unsigned long now);
void updateNormalAutoCountdown(unsigned long now);
void displayVOCData(float con1, float con2, float range);
void processVOCCommand(String cmd);
bool parseVOCResponse(uint8_t *data, int length, VOCResponseData &response);
void sendVOCCommand(uint8_t *cmd, int length, const char *cmd_name, bool parse_response);
void printHexBuffer(uint8_t *buffer, int length);
bool isPlausible(float value, const String &sensor);

void setup() {
  Serial.begin(DEBUG_BAUDRATE);
  O2_UART.begin(SENSOR_BAUDRATE, SERIAL_8N1, U1_RX, U1_TX);
  CO2_UART.begin(SENSOR_BAUDRATE, SERIAL_8N1, U2_RX, U2_TX);

  prefs.begin("sensor_cfg", false);
  loadSensorConfig(co2);
  loadSensorConfig(o2);
  loadSensorConfig(voc);
  loadSensorConfig(cozir);   // new

  // Start in MODE 0 on every power-up so CO2 monitoring begins automatically.
  // Other modes can still be selected later using CMD -> MODE 1 / MODE 2.
  operation_mode = 0;
  prefs.end();

  // Load customer defaults, applying only previously saved CMD overrides.
  loadTcalSettings();
  tcal_auto_started_ms = millis();
  tcal_auto_start_now = false; // Always wait the full saved interval after power-up.
  tcal_retry_waiting = false;
  tcal_auto_cycle_running = false;
  tcal_enabled = false;
  Serial.printf("[TCAL] Startup defaults/settings: tolerance %.2f%%, known CO2 %.3f%%, samples %u, retry %lu sec, auto %lu %s (%s).\n",
                var_Tolerance, var_Known_CO2, tcal_target_samples,
                (unsigned long)tcal_delay_seconds, tcalAutoPeriodValue(),
                tcalAutoPeriodUnit(), tcal_auto_enabled ? "ON" : "OFF");
  if (fabsf(var_Known_CO2 - 20.9f) < 0.0001f) {
    Serial.println("[WARNING] Known CO2 default is 20.9%. This resembles ambient O2, NOT normal ambient CO2. Verify reference gas before SST X calibration.");
  }

  // Set reset thresholds (consecutive invalid readings = reboot)
  o2.resetThreshold = 0;
  co2.resetThreshold = 0;

  delay(500);

  if (operation_mode == 0) {
    while (CO2_UART.available()) CO2_UART.read();
        CO2_UART.print("K 2\r\n");
        CO2_UART.flush();
        delay(150);

    while (CO2_UART.available()) CO2_UART.read();

        Serial.println(
        "[CO2] Automatic CO2 monitoring enabled at power-up."
        );

    O2_UART.print("M 1\r\n");
    O2_UART.flush();
  } else if (operation_mode == 1) {
    // Existing VOC initialisation
    uint8_t qmode_cmd[] = { 0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46 };
    sendVOCCommand(qmode_cmd, sizeof(qmode_cmd), "Set Query Mode", 1);
  } else if (operation_mode == 2) {
    // CozIR initialisation (optional – set to streaming mode if needed)
    COZIR_UART.print("K 2\r\n");   // changed to polling mode
    COZIR_UART.flush();
  }

  display_init();
  pinMode(O2_DAC, OUTPUT);
  pinMode(CO2_DAC, OUTPUT);
  dacWrite(O2_DAC, 0);
  dacWrite(CO2_DAC, 0);
  pinMode(VOC_DAC, OUTPUT); //repeated 25 could remove  after test
  Serial.printf("O2/CO2 NORMAL mode started. TCAL AUTO %s; next check after %lu %s.\n",
                tcal_auto_enabled ? "ON" : "OFF", tcalAutoPeriodValue(), tcalAutoPeriodUnit());
}

void loop() {
  const unsigned long current = millis();
  updateTcalAuto(current);
  updateCalibrationCountdown(current);
  updateOledCountdown(current);  // Refresh the small OLED timer once per second

  // Normal mode owns continuous monitoring and TCAL sampling. CMD/CONFIG
  // are for inspecting/changing settings; no unsolicited sensor readings.
  // During TCAL, the same CO2 poll updates the live display/DAC AND
  // contributes the sample, avoiding duplicate UART requests.
  const bool normalMode = !command_mode && !config_mode;

  if (normalMode && operation_mode == 0 && co2.enabled &&
      (uint32_t)(millis() - co2.last_sample) >= co2.sample_interval) {
    co2.last_sample = millis();
    float raw = NAN;
    // Bound normal UART waits so a missing CO2 sensor doesn't stall O2.
    if (readSensor(CO2_UART, raw, "CO2", LIVE_POLL_TIMEOUT_MS) &&
        isfinite(raw) && isPlausible(raw, "CO2")) {
      co2.current_value = raw + co2.offset;
      co2.consecutiveErrors = 0;
      co2_has_reading = true;
      co2_failed_reads = 0;
      co2_display = processData(co2) / 1000.0f;
      updateGasDac("CO2", co2_display);  // DAC updates even while TCAL runs
      // Keep CO2 visible in Serial Monitor during TCAL AUTO, including CMD.
      // The same reading is also used for the TCAL sample when a batch is active.
      Serial.printf("[CO2] %.3f %%\n", co2_display);
      if (tcal_enabled) sample_timer(raw / 1000.0f, millis());
      if (display_ok) displayO2CO2Data(o2_display, co2_display);
    } else {
      handleLiveSensorFailure("CO2");
    }
  }

  // O2 runs continuously in normal mode, independently of CO2 and TCAL.
  // CMD/CONFIG temporarily pause unsolicited sensor measurements.
  if (normalMode && operation_mode == 0 && o2.enabled &&
      (uint32_t)(millis() - o2.last_sample) >= o2.sample_interval) {
    o2.last_sample = millis();
    float rawO2 = NAN;
    if (readSensor(O2_UART, rawO2, "O2", LIVE_POLL_TIMEOUT_MS) &&
        isfinite(rawO2) && isPlausible(rawO2, "O2")) {
      o2.current_value = rawO2 + o2.offset;
      o2.consecutiveErrors = 0;
      o2_has_reading = true;
      o2_failed_reads = 0;
      const float processedO2 = processData(o2);
      if (o2.p != 0.0f) {
        o2_display = o2.gain * (((processedO2 / o2.p) * 1000.0f) -
                                ((o2.alpha / o2.p) * 100.0f));
        updateGasDac("O2", o2_display);  // independent of CO2 and OLED
        Serial.printf("[O2] %.2f %%\n", o2_display);
        if (display_ok) displayO2CO2Data(o2_display, co2_display);
      } else {
        handleLiveSensorFailure("O2");
      }
    } else {
      handleLiveSensorFailure("O2");
    }
  }

  // Print the NEXT/RETRY countdown once per second in NORMAL mode only.
  updateNormalAutoCountdown(millis());

  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  String upperCmd = cmd;
  upperCmd.toUpperCase();

  if (upperCmd == "CMD") {
    command_mode = true;
    config_mode = false;
    Serial.println("CMD ready (settings mode). Automatic O2/CO2 readings paused until EXIT.");
    Serial.println("MODE 0/1/2 | READ | O2 READ | CO2 READ | VOC READ | COZIR READ");
    Serial.println("TCAL SAMPLES <count> | TCAL DELAY MINUTES <n> | TCAL STATUS | CONFIG -> SET_SAMPLE CO2 <ms> | EXIT");
    return;
  }
  if (!command_mode) {
    Serial.println("NORMAL mode active. Enter CMD to inspect or change TCAL settings.");
    return;
  }
  if (upperCmd == "EXIT") {
    command_mode = false;
    config_mode = false;
    Serial.println("NORMAL mode resumed: continuous O2/CO2 readings and TCAL scheduler.");
    return;
  }
  if (upperCmd == "BACK") {
    config_mode = false;
    Serial.println("Back to CMD. Enter EXIT to return to normal monitoring.");
    return;
  }
  if (upperCmd.startsWith("TCAL ")) {
    handleTimedCalibrationCommand(upperCmd);
    return;
  }
  if (upperCmd == "MODE" || upperCmd == "MODE STATUS") {
    Serial.printf("Current mode: %d (%s)\n", operation_mode,
                  operation_mode == 0 ? "O2/CO2" : (operation_mode == 1 ? "VOC" : "COZIR"));
    return;
  }
  if (upperCmd.startsWith("MODE ")) {
    if (upperCmd.length() != 6 || upperCmd[5] < '0' || upperCmd[5] > '2') {
      Serial.println("Use MODE 0, MODE 1, or MODE 2.");
      return;
    }
    int new_mode = upperCmd[5] - '0';
    if (new_mode != 0 && tcal_enabled) {
      tcal_enabled = false;
      tcal_first_check_pending = false;
      tcal_countdown_active = false;
      resetSampleTimer();
      Serial.println("[TCAL] Stopped: leaving O2/CO2 mode.");
    }
    operation_mode = new_mode;
    prefs.begin("sensor_cfg", false);
    prefs.putInt("operation_mode", operation_mode);
    prefs.end();
    if (operation_mode == 0) {
      CO2_UART.print("K 2\r\n");
      CO2_UART.flush();
      O2_UART.print("M 1\r\n");
      O2_UART.flush();
    } else if (operation_mode == 1) {
      uint8_t query_mode_cmd[] = {0xFF, 0x01, 0x78, 0x41, 0, 0, 0, 0, 0x46};
      sendVOCCommand(query_mode_cmd, sizeof(query_mode_cmd), "Set Query Mode", 1);
    } else {
      COZIR_UART.print("K 2\r\n");
      COZIR_UART.flush();
    }
    display_init();
    Serial.printf("Mode %d selected. Enter READ to request a measurement.\n", operation_mode);
    return;
  }
  if (upperCmd == "READ" || upperCmd == "READ ALL") {
    cmdReadActiveMode();
    return;
  }
  if (upperCmd == "O2 READ" || upperCmd == "READ O2") {
    if (operation_mode != 0) Serial.println("Select MODE 0 first.");
    else cmdReadSensor("O2");
    return;
  }
  if (upperCmd == "CO2 READ" || upperCmd == "READ CO2") {
    if (operation_mode != 0) Serial.println("Select MODE 0 first.");
    else cmdReadSensor("CO2");
    return;
  }
  if (upperCmd == "VOC READ" || upperCmd == "READ VOC") {
    if (operation_mode != 1) Serial.println("Select MODE 1 first.");
    else processVOCCommand("GAS");
    return;
  }
  if (upperCmd == "COZIR READ" || upperCmd == "READ COZIR") {
    if (operation_mode != 2) Serial.println("Select MODE 2 first.");
    else {
      float raw;
      if (readCozirSensor(COZIR_UART, raw) && isfinite(raw))
        Serial.printf("COZIR: %.3f %%\n", (raw * COZIR_MODE2_PPM_MULTIPLIER + cozir.offset) / 10000.0f);
      else Serial.println("COZIR: no valid response.");
    }
    return;
  }
  if (upperCmd == "CONFIG") {
    config_mode = true;
    printConfigHelp();
    Serial.println("Enter BACK to return to CMD.");
    return;
  }
  if (config_mode) {
    processCommand(cmd);
    return;
  }
  // Preserve the original manual low-level sensor commands under CMD.
  command_mode_handler(cmd);
}

void cmdReadSensor(const String &name) {
  SensorConfig &cfg = (name == "O2") ? o2 : co2;
  HardwareSerial &port = (name == "O2") ? O2_UART : CO2_UART;
  if (!cfg.enabled) {
    Serial.println(name + ": disabled in sensor configuration.");
    return;
  }
  float raw = NAN;
  if (!readSensor(port, raw, name) || !isfinite(raw) || !isPlausible(raw, name)) {
    handleLiveSensorFailure(name);
    Serial.println(name + ": no valid sensor reading.");
    return;
  }
  cfg.current_value = raw + cfg.offset;
  float processed_value = processData(cfg);
  if (name == "O2") {
    if (cfg.p == 0.0f) { Serial.println("O2: invalid calibration scale."); return; }
    o2_display = cfg.gain * (((processed_value / cfg.p) * 1000.0) - ((cfg.alpha / cfg.p) * 100.0));
    o2_has_reading = true;
    o2_failed_reads = 0;
    updateGasDac("O2", o2_display);
    Serial.printf("O2: %.2f %%\n", o2_display);
  } else {
    co2_display = processed_value / 1000.0f;
    co2_has_reading = true;
    co2_failed_reads = 0;
    updateGasDac("CO2", co2_display);
    Serial.printf("CO2: %.3f %%\n", co2_display);
  }
  if (display_ok && operation_mode == 0) displayO2CO2Data(o2_display, co2_display);
}

void cmdReadActiveMode() {
  if (operation_mode == 0) {
    cmdReadSensor("O2");
    cmdReadSensor("CO2");
  } else if (operation_mode == 1) {
    processVOCCommand("GAS");
  } else {
    float raw;
    if (readCozirSensor(COZIR_UART, raw) && isfinite(raw))
      Serial.printf("COZIR: %.3f %%\n", (raw * COZIR_MODE2_PPM_MULTIPLIER + cozir.offset) / 10000.0f);
    else Serial.println("COZIR: no valid response.");
  }
}

String command_mode_handler(String cmd) {
  static int uart_channel = 0;
  cmd.trim();

  if (cmd.equalsIgnoreCase("O2")) {
    uart_channel = 1;
    printHelp();
    Serial.println("O2 selected, Enter commands for O2 sensor.");
    return "";
  } else if (cmd.equalsIgnoreCase("CO2")) {
    uart_channel = 2;
    printHelp();
    Serial.println("CO2 selected, Enter commands for CO2 sensor.");
    return "";
  } else if (cmd.equalsIgnoreCase("VOC")) {
    uart_channel = 3;
    printHelp();
    Serial.println("VOC selected, Enter commands for VOC sensor.");
    return "";
  } else if (cmd.equalsIgnoreCase("COZIR")) {
    uart_channel = 4;
    printCozirHelp();          // new help function
    Serial.println("COZIR selected, Enter commands for CozIR-LP sensor.");
    return "";
  }
   else if (cmd.equalsIgnoreCase("CONFIG")) {
    config_mode = true;
    printConfigHelp();
    return "";
  }

  if (cmd.length() == 0) return "";

  if (uart_channel == 1) {
    O2_UART.println(cmd);
    Serial.println("Send to O2: " + cmd);
    O2_UART.flush();
    unsigned long start = millis();
    while (millis() - start < SENSOR_TIMEOUT) {
      while (O2_UART.available()) {
        String raw = O2_UART.readStringUntil('\n');
        raw.trim();
        Serial.printf("O2 Response: %s\n", raw.c_str());
        start = millis();
      }
    }
  } else if (uart_channel == 2) {
    while (CO2_UART.available()) CO2_UART.read();

    CO2_UART.print(cmd + "\r\n");
    Serial.println("Send to CO2: " + cmd);
    CO2_UART.flush();

    unsigned long start = millis();
    while (millis() - start < 10000UL) {
      while (CO2_UART.available()) {
        String raw = CO2_UART.readStringUntil('\n');
        raw.trim();

        if (raw.length() > 0) {
        Serial.println("CO2 Response: " + raw);
        }
      }
    }
  } else if (uart_channel == 3) {
    processVOCCommand(cmd);
  }
    else if (uart_channel == 4) {
    processCozirCommand(cmd);
  }

  return "";
}

void processVOCCommand(String cmd) {
  // (unchanged – full VOC command handling as in original)
  cmd.trim();

  if (cmd.equalsIgnoreCase("AUTO")) {
    uint8_t auto_mode_cmd[] = { 0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47 };
    sendVOCCommand(auto_mode_cmd, sizeof(auto_mode_cmd), "Set Auto Mode", 1);
    voc_auto_reporting = true;
  } else if (cmd.equalsIgnoreCase("QUERY")) {
    uint8_t query_mode_cmd[] = { 0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46 };
    sendVOCCommand(query_mode_cmd, sizeof(query_mode_cmd), "Set Query Mode", 1);
    voc_auto_reporting = false;
  } else if (cmd.equalsIgnoreCase("GAS")) {
    uint8_t gas_cmd[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
    sendVOCCommand(gas_cmd, sizeof(gas_cmd), "Read Concentration", 1);
  } else if (cmd.equalsIgnoreCase("SENSOR1")) {
    uint8_t sensor1_cmd[] = { 0xD1 };
    sendVOCCommand(sensor1_cmd, sizeof(sensor1_cmd), "Query sensor1 Params (Method 1)", 1);
  } else if (cmd.equalsIgnoreCase("SENSOR2")) {
    uint8_t sensor2_cmd[] = { 0xD7 };
    sendVOCCommand(sensor2_cmd, sizeof(sensor2_cmd), "Query sensor2 Params (Method 2)", 1);
  } else if (cmd.equalsIgnoreCase("GASTH")) {
    uint8_t gasth_cmd[] = { 0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78 };
    sendVOCCommand(gasth_cmd, sizeof(gasth_cmd), "Read Gas Concentration, Temp, Hum", 1);
  } else if (cmd.equalsIgnoreCase("TH1")) {
    uint8_t temp_hum_cmd1[] = { 0xD2 };
    sendVOCCommand(temp_hum_cmd1, sizeof(temp_hum_cmd1), "Read Temp, Hum", 1);
  } else if (cmd.equalsIgnoreCase("TH2")) {
    uint8_t temp_hum_cmd2[] = { 0xD6 };
    sendVOCCommand(temp_hum_cmd2, sizeof(temp_hum_cmd2), "Read Temp, Hum", 1);
  } else if (cmd.equalsIgnoreCase("SLEEP1")) {
    uint8_t sleep1cmd[] = { 0xAF, 0x53, 0x6C, 0x65, 0x65, 0x70 };
    sendVOCCommand(sleep1cmd, sizeof(sleep1cmd), "Sleep1", 0);
  } else if (cmd.equalsIgnoreCase("WAKE1")) {
    uint8_t wake1_cmd[] = { 0xAE, 0x45, 0x78, 0x69, 0x74 };
    sendVOCCommand(wake1_cmd, sizeof(wake1_cmd), "Wake1", 0);
  } else if (cmd.equalsIgnoreCase("SLEEP2")) {
    uint8_t sleep2cmd[] = { 0xA1, 0x53, 0x6C, 0x65, 0x65, 0x70, 0x32 };
    sendVOCCommand(sleep2cmd, sizeof(sleep2cmd), "Sleep2", 0);
  } else if (cmd.equalsIgnoreCase("WAKE2")) {
    uint8_t wake2_cmd[] = { 0xA2, 0x45, 0x78, 0x69, 0x74, 0x32 };
    sendVOCCommand(wake2_cmd, sizeof(wake2_cmd), "Wake2", 0);
  } else if (cmd.equalsIgnoreCase("LEDON")) {
    uint8_t ledon_cmd[] = { 0xFF, 0x01, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77 };
    sendVOCCommand(ledon_cmd, sizeof(ledon_cmd), "LED ON", 1);
  } else if (cmd.equalsIgnoreCase("LEDOFF")) {
    uint8_t ledoff_cmd[] = { 0xFF, 0x01, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76 };
    sendVOCCommand(ledoff_cmd, sizeof(ledoff_cmd), "LED OFF", 1);
  } else if (cmd.equalsIgnoreCase("LED")) {
    uint8_t led_cmd[] = { 0xFF, 0x01, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75 };
    sendVOCCommand(led_cmd, sizeof(led_cmd), "LED STATUS", 1);
  } else if (cmd.equalsIgnoreCase("FACTORYRST")) {
    uint8_t fr_cmd[] = { 0xFF, 0x01, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71 };
    sendVOCCommand(fr_cmd, sizeof(fr_cmd), "FACTORY RESET", 1);
  } else if (cmd.equalsIgnoreCase("CALI")) {
    uint8_t cali_cmd[] = { 0xFF, 0x01, 0x8D, 0x41, 0x20, 0x00, 0x00, 0x00, 0x11 };
    sendVOCCommand(cali_cmd, sizeof(cali_cmd), "CALI", 1);
  } else if (cmd.equalsIgnoreCase("SOFT")) {
    uint8_t soft_cmd[] = { 0xD3 };
    sendVOCCommand(soft_cmd, sizeof(soft_cmd), "SOFTWARE VERSION", 1);
  } else if (cmd.equalsIgnoreCase("HELP")) {
    printHelp();
  } else if (cmd.startsWith("0x")) {
    processRawHexCommand(cmd);
  } else {
    Serial.println("Unknown VOC command. Type 'HELP' for available commands or use raw hex format: 0xFF 0x01...");
  }
}

void sendVOCCommand(uint8_t *cmd, int length, const char *cmd_name, bool parse_response) {
  // (unchanged)
  Serial.print("Sending VOC Command (");
  Serial.print(cmd_name);
  Serial.print("): ");
  printHexBuffer(cmd, length);
  Serial.println();

  while (VOC_UART.available()) VOC_UART.read();

  VOC_UART.write(cmd, length);
  VOC_UART.flush();

  unsigned long start_time = millis();
  uint8_t response_buffer[20];
  int response_index = 0;

  Serial.print("Waiting for response... ");

  while (millis() - start_time < SENSOR_TIMEOUT) {
    if (VOC_UART.available()) {
      response_buffer[response_index++] = VOC_UART.read();
      start_time = millis();

      if (parse_response) {
        VOCResponseData response;
        if (parseVOCResponse(response_buffer, response_index, response)) {
          Serial.print("\nResponse: ");
          printHexBuffer(response_buffer, response_index);
          Serial.println();
          printVOCResponse(response);
          if (response.command_type == 0x86 || response.command_type == 0x87) {
            displayVOCData(response.concentration1, response.concentration2, response.range);
          }
          break;
        }
      }

      if (!parse_response && response_index >= 9) break;
      if (response_index >= sizeof(response_buffer)) break;
    }
  }

  if (response_index > 0) {
    Serial.print("\nRaw Response: ");
    printHexBuffer(response_buffer, response_index);
    Serial.println();
  } else {
    Serial.println("No response received");
  }
  Serial.println();
}

void printHexBuffer(uint8_t *buffer, int length) {
  for (int i = 0; i < length; i++) Serial.printf("%02X ", buffer[i]);
}

void processRawHexCommand(String hexCmd) {
  // (unchanged)
  hexCmd.trim();
  int count = 0;
  int startIndex = 0;
  int endIndex = hexCmd.indexOf(' ');
  while (endIndex != -1) {
    count++;
    startIndex = endIndex + 1;
    endIndex = hexCmd.indexOf(' ', startIndex);
  }
  count++;
  uint8_t cmd_buffer[count];
  startIndex = 0;
  for (int i = 0; i < count; i++) {
    endIndex = hexCmd.indexOf(' ', startIndex);
    if (endIndex == -1) endIndex = hexCmd.length();
    String hexByte = hexCmd.substring(startIndex, endIndex);
    hexByte.trim();
    if (hexByte.startsWith("0x")) hexByte = hexByte.substring(2);
    cmd_buffer[i] = strtol(hexByte.c_str(), NULL, 16);
    startIndex = endIndex + 1;
  }
  sendVOCCommand(cmd_buffer, count, "Raw Hex Command", false);
}

void processCozirCommand(String cmd) {
  cmd.trim();

  // High-level commands
  if (cmd.equalsIgnoreCase("MODE0")) {
    COZIR_UART.print("K 0\r\n");
    Serial.println("Send: K 0 (Command Mode)");
  } else if (cmd.equalsIgnoreCase("MODE1")) {
    COZIR_UART.print("K 1\r\n");
    Serial.println("Send: K 1 (Streaming Mode)");
  } else if (cmd.equalsIgnoreCase("MODE2")) {
    COZIR_UART.print("K 2\r\n");
    Serial.println("Send: K 2 (Polling Mode)");
  } else if (cmd.startsWith("FILTER ")) {
    String val = cmd.substring(7);
    COZIR_UART.print("A " + val + "\r\n");
    Serial.println("Send: A " + val);
  } else if (cmd.equalsIgnoreCase("FILTER?")) {
    COZIR_UART.print("a\r\n");
    Serial.println("Send: a (Read filter)");
  } else if (cmd.startsWith("ALTITUDE ")) {
    String val = cmd.substring(9);
    COZIR_UART.print("S " + val + "\r\n");
    Serial.println("Send: S " + val);
  } else if (cmd.equalsIgnoreCase("ALTITUDE?")) {
    COZIR_UART.print("s\r\n");
    Serial.println("Send: s (Read altitude)");
  } else if (cmd.startsWith("AUTOZERO ")) {
    String sub = cmd.substring(9);
    if (sub.equalsIgnoreCase("ON")) {
      COZIR_UART.print("@ 1.0 8.0\r\n");   // example
      Serial.println("Send: @ 1.0 8.0 (Enable auto-zero)");
    } else if (sub.equalsIgnoreCase("OFF")) {
      COZIR_UART.print("@ 0\r\n");
      Serial.println("Send: @ 0 (Disable auto-zero)");
    } else {
      int space = sub.indexOf(' ');
      if (space > 0) {
        String initial = sub.substring(0, space);
        String regular = sub.substring(space + 1);
        COZIR_UART.print("@ " + initial + " " + regular + "\r\n");
        Serial.println("Send: @ " + initial + " " + regular);
      } else {
        Serial.println("Usage: AUTOZERO ON|OFF or AUTOZERO <initial> <regular>");
      }
    }
  } else if (cmd.equalsIgnoreCase("AUTOZERO?")) {
    COZIR_UART.print("@\r\n");
    Serial.println("Send: @ (Read auto-zero config)");
  } else if (cmd.equalsIgnoreCase("ZERO_N2")) {
    COZIR_UART.print("U\r\n");
    Serial.println("Send: U (Zero in Nitrogen)");
  } else if (cmd.equalsIgnoreCase("ZERO_FRESH")) {
    COZIR_UART.print("G\r\n");
    Serial.println("Send: G (Zero in Fresh Air)");
  } else if (cmd.startsWith("ZERO_KNOWN ")) {
    String val = cmd.substring(11);
    COZIR_UART.print("X " + val + "\r\n");
    Serial.println("Send: X " + val);
  } else if (cmd.startsWith("ZERO_ADJUST ")) {
    int space = cmd.indexOf(' ', 12);
    if (space > 0) {
      String reported = cmd.substring(12, space);
      String actual = cmd.substring(space + 1);
      COZIR_UART.print("F " + reported + " " + actual + "\r\n");
      Serial.println("Send: F " + reported + " " + actual);
    } else {
      Serial.println("Usage: ZERO_ADJUST <reported> <actual>");
    }
  } else if (cmd.startsWith("FORCE_ZERO ")) {
    String val = cmd.substring(11);
    COZIR_UART.print("u " + val + "\r\n");
    Serial.println("Send: u " + val);
  } else if (cmd.equalsIgnoreCase("READ_CO2")) {
    COZIR_UART.print("Z\r\n");
    Serial.println("Send: Z (Filtered CO₂)");
  } else if (cmd.equalsIgnoreCase("READ_CO2_RAW")) {
    COZIR_UART.print("z\r\n");
    Serial.println("Send: z (Unfiltered CO₂)");
  } else if (cmd.startsWith("MASK ")) {
    String val = cmd.substring(5);
    COZIR_UART.print("M " + val + "\r\n");
    Serial.println("Send: M " + val);
  } else if (cmd.equalsIgnoreCase("SCALE?")) {
    COZIR_UART.print(".\r\n");
    Serial.println("Send: . (Read scaling factor)");
  } else if (cmd.equalsIgnoreCase("VERSION")) {
    COZIR_UART.print("Y\r\n");
    Serial.println("Send: Y (Firmware & serial)");
  } else if (cmd.equalsIgnoreCase("HELP")) {
    printCozirHelp();
  } else {
    // Pass through raw command
    COZIR_UART.print(cmd + "\r\n");
    Serial.println("Send raw: " + cmd);
  }

  // Read back response
  COZIR_UART.flush();
  unsigned long start = millis();
  while (millis() - start < SENSOR_TIMEOUT) {
    while (COZIR_UART.available()) {
      String raw = COZIR_UART.readStringUntil('\n');
      Serial.printf("COZIR Response: %s\n", raw.c_str());
      start = millis();
    }
  }
}

// Compact 128x64 OLED timer: automatic wait, first-read check, or sample countdown.
// Timekeeping is based on the same millis() timers used by the calibration logic.
String tcalOledTimerText(unsigned long now) {
  char line[24];

  if (operation_mode != 0) return "";

  // Retry countdown has priority so a short delay remains visible;
  // the FAIL result is still printed in Serial Monitor.
  if (tcal_auto_enabled && tcal_retry_waiting &&
      !tcal_enabled && !tcal_auto_cycle_running) {
    const uint32_t elapsed = (uint32_t)(now - tcal_delay_started_ms);
    const uint32_t target = tcal_delay_seconds * 1000UL;
    const uint32_t rem = elapsed >= target ? 0UL : (target - elapsed + 999UL) / 1000UL;
    snprintf(line, sizeof(line), "RETRY %02lu:%02lu:%02lu",
             (unsigned long)(rem / 3600UL),
             (unsigned long)((rem / 60UL) % 60UL),
             (unsigned long)(rem % 60UL));
    return String(line);
  }

  // Keep PASS/FAIL and the SST response visible during the notice period.
  if (tcal_oled_notice.length() &&
      (int32_t)(tcal_oled_notice_until - now) > 0) {
    return tcal_oled_notice;
  }

  if (tcal_enabled && tcal_countdown_active) {
    if (tcal_last_sample_label.length() &&
        (int32_t)(tcal_last_sample_notice_until - now) > 0)
      return tcal_last_sample_label;
    snprintf(line, sizeof(line), "SAMPLES %d/%u", sample_count, tcal_target_samples);
    return String(line);
  }
  if (tcal_enabled) return "CAL ACTIVE";

  if (!tcal_auto_enabled) return "AUTO OFF";
  if (tcal_auto_cycle_running) return "CAL FINISHING";
  if (tcal_auto_start_now) return "STARTING...";

  const uint32_t periodMs = tcalAutoPeriodMs();
  const uint32_t elapsedMs = (uint32_t)(now - tcal_auto_started_ms);
  const uint32_t remainingMs = elapsedMs >= periodMs ? 0 : periodMs - elapsedMs;
  const uint32_t seconds = (remainingMs + 999UL) / 1000UL;

  // Fits in the 21-character maximum of a 128-pixel wide SSD1306 at text size 1.
  if (seconds >= 86400UL) {
    snprintf(line, sizeof(line), "NEXT %lud %02lu:%02lu:%02lu",
             (unsigned long)(seconds / 86400UL),
             (unsigned long)((seconds / 3600UL) % 24UL),
             (unsigned long)((seconds / 60UL) % 60UL),
             (unsigned long)(seconds % 60UL));
  } else {
    snprintf(line, sizeof(line), "NEXT %02lu:%02lu:%02lu",
             (unsigned long)(seconds / 3600UL),
             (unsigned long)((seconds / 60UL) % 60UL),
             (unsigned long)(seconds % 60UL));
  }
  return String(line);
}

void display_init() {
#if ENABLE_DISPLAY
  // Initialize the I2C bus, then check for the OLED before using the
  // SSD1306 library. An absent OLED must not stop gas monitoring.
  Wire.begin(I2C_SDA1, I2C_SCL1);
  Wire.beginTransmission(SCREEN_ADDRESS);
  const uint8_t oledProbeResult = Wire.endTransmission();
  if (oledProbeResult != 0) {
    display_ok = false;
    Serial.printf("[OLED] No response at I2C address 0x%02X (error %u). Continuing without display.\n",
                  SCREEN_ADDRESS, oledProbeResult);
    return;
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("[OLED] SSD1306 initialization failed. Continuing without display.");
    display_ok = false;
  } else {
    display_ok = true;
    Serial.println("[OLED] SSD1306 initialized.");
  }

  if (display_ok) {
    display.clearDisplay();
    display.setFont(NULL);
    display.setTextColor(WHITE);
    if (operation_mode == 0) {
      displayO2CO2Data(o2_display, co2_display);
    } else if (operation_mode == 1) {
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("VOC Sensor");
      display.setTextSize(1);
      display.setCursor(0, 25);
      display.println("Concentration:");
      display.display();
    } else if (operation_mode == 2) {
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("COz Sensor");
      display.setCursor(0, 40);
      display.println("CO:");
      display.setTextSize(1);
      display.setCursor(22, 50);
      display.println("2");
      display.display();
    }
  }
#else
  display_ok = false;
  Serial.println("Display disabled (ENABLE_DISPLAY=0)");
#endif
}

void displayO2CO2Data(float o2_value, float co2_value) {
  if (!display_ok || operation_mode != 0) return;
  display.clearDisplay();
  display.setFont(NULL);
  display.setTextColor(WHITE);

  // Compact heading and two readings; keep the bottom 9 pixels for countdown.
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Gas Sensor");
  display.drawFastHLine(0, 10, SCREEN_WIDTH, WHITE);

  display.setTextSize(2);
  display.setCursor(0, 14);
  display.print("O2");
  display.setCursor(39, 14);
  display.print(o2_has_reading ? String(o2_value, 2) + "%" : "N/A");
  display.setCursor(0, 35);
  display.print("CO2");
  display.setCursor(39, 35);
  display.print(co2_has_reading ? String(co2_value, 3) + "%" : "N/A");

  display.drawFastHLine(0, 54, SCREEN_WIDTH, WHITE);
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(tcalOledTimerText(millis()));
  display.display();
}

void updateOledCountdown(unsigned long now) {
  static uint32_t lastRefresh = 0;
  if (!display_ok || operation_mode != 0) return;
  if ((uint32_t)(now - lastRefresh) < 1000UL) return;
  lastRefresh = now;
  displayO2CO2Data(o2_display, co2_display);
}


// Live NEXT/RETRY countdown in Serial Monitor while NORMAL mode is active.
// Sample readings are printed only by sample_timer(), not by this function.
void updateNormalAutoCountdown(unsigned long now) {
  static uint32_t lastShownSecond = UINT32_MAX;
  static int lastPhase = 0;

  if (command_mode || config_mode || operation_mode != 0 || !tcal_auto_enabled ||
      tcal_enabled || tcal_auto_cycle_running || tcal_auto_start_now) {
    lastShownSecond = UINT32_MAX;
    lastPhase = 0;
    return;
  }

  const bool retry = tcal_retry_waiting;
  const uint32_t periodMs = retry ? tcal_delay_seconds * 1000UL
                                  : tcalAutoPeriodMs();
  const uint32_t startedMs = retry ? tcal_delay_started_ms
                                   : tcal_auto_started_ms;
  const uint32_t elapsedMs = (uint32_t)(now - startedMs);
  const uint32_t remainingMs = elapsedMs >= periodMs ? 0UL : periodMs - elapsedMs;
  const uint32_t remainingSec = (remainingMs + 999UL) / 1000UL;
  const int phase = retry ? 2 : 1;

  if (remainingSec == lastShownSecond && phase == lastPhase) return;
  lastShownSecond = remainingSec;
  lastPhase = phase;

  if (remainingSec >= 86400UL) {
    Serial.printf("[TCAL AUTO] %s %lud %02lu:%02lu:%02lu\n",
                  retry ? "RETRY" : "NEXT",
                  (unsigned long)(remainingSec / 86400UL),
                  (unsigned long)((remainingSec / 3600UL) % 24UL),
                  (unsigned long)((remainingSec / 60UL) % 60UL),
                  (unsigned long)(remainingSec % 60UL));
  } else {
    Serial.printf("[TCAL AUTO] %s %02lu:%02lu:%02lu\n",
                  retry ? "RETRY" : "NEXT",
                  (unsigned long)(remainingSec / 3600UL),
                  (unsigned long)((remainingSec / 60UL) % 60UL),
                  (unsigned long)(remainingSec % 60UL));
  }
}


void displayVOCData(float con1, float con2, float range) {
  if (!display_ok) return;
  display.fillRect(0, 40, 128, 64, BLACK);
  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(String(con1, 1));
  display.print("ppb\n\n");
  display.display();
}

void displayCozirData(float co2_percent) {
  if (!display_ok) return;
  display.fillRect(0, 20, 88, 18, BLACK);
  display.fillRect(40, 40, 88, 18, BLACK);
  display.setTextSize(2);
  display.setCursor(40, 40);
  display.print(String(co2_percent, 3));
  display.print("%");
  display.display();
}

float extractFirstFloat(const String &str) {
  bool foundDigit = false;
  String numStr = "";
  for (unsigned int i = 0; i < str.length(); i++) {
    char c = str[i];
    if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') {
      numStr += c;
      foundDigit = true;
    } else if (foundDigit) break;
  }
  return numStr.toFloat();
}

bool isPlausible(float value, const String &sensor) {
  if (sensor == "O2") return (value >= O2_MIN && value <= O2_MAX);
  if (sensor == "CO2") return (value >= CO2_MIN && value <= CO2_MAX);
  return true; // VOC or unknown
}

void sensorRoutine(SensorConfig &cfg, HardwareSerial &uart, unsigned long current) {
  if (!cfg.enabled) return;

  if (current - cfg.last_sample >= cfg.sample_interval) {
    float raw_value;
    bool gotResponse = readSensor(uart, raw_value, cfg.name);

    if (gotResponse) {
      // Check plausibility for O2 and CO2
      if (cfg.name == "O2" || cfg.name == "CO2") {
        if (isPlausible(raw_value, cfg.name)) {
          cfg.current_value = raw_value + cfg.offset;
          cfg.last_sample = current;
          cfg.consecutiveErrors = 0;

          // CO2 calibration uses only a valid raw CO2 sensor measurement.
          // Keep O2 measurement and all display/DAC formulas unchanged.
          if (cfg.name == "CO2" && tcal_enabled &&
              isfinite(raw_value) && raw_value >= 0.0f) {
            // Only a successfully parsed, plausible sensor Z response is sampled.
            // Raw Z units have confirmed 10-ppm multiplier; display uses percent.
            sample_timer(raw_value / 1000.0f, millis());
          }
        } else {
          cfg.consecutiveErrors++;
          Serial.printf("[%s] Invalid reading #%d: %.2f\n",
                        cfg.name.c_str(), cfg.consecutiveErrors, raw_value);
          cfg.last_sample = current;
        }
      } else {
        cfg.current_value = raw_value + cfg.offset;
        cfg.last_sample = current;
      }
    } else {
      // Read failed (timeout) – set value to 0 to indicate no data
      if (cfg.name == "O2" || cfg.name == "CO2") {
        Serial.printf("[%s] Read timeout (no valid measurement)\n",
              cfg.name.c_str());
              cfg.consecutiveErrors++;
      // Keep last value rather than introducing an artificial zero.      
        cfg.last_sample = current;  
      }
    }

    // Reboot if too many consecutive invalid readings
    if ((cfg.name == "O2" || cfg.name == "CO2") &&
        cfg.resetThreshold > 0 &&
        cfg.consecutiveErrors >= cfg.resetThreshold) {
      Serial.printf("[%s] Too many invalid readings (%d). Rebooting...\n",
                    cfg.name.c_str(), cfg.consecutiveErrors);
      delay(500);
      esp_restart();
    }
  }

  float processed_value = processData(cfg);

  if (current - cfg.last_output >= cfg.output_interval) {
    if (cfg.name.equals("O2")) {
      o2_display = cfg.gain * (((processed_value / cfg.p) * 1000.0) - ((cfg.alpha / cfg.p * 100.0)));
    }
    if (cfg.name.equals("CO2")) {
      co2_display = processed_value / 1000;
    }

    Serial.printf("[%s] %s: %.2f %%\n", cfg.name.c_str(), getModeString(cfg.output_mode),
                  cfg.name.equals("O2") ? o2_display : co2_display);

    if (display_ok) {
      if (cfg.name.equals("CO2")) {
        dacWrite(CO2_DAC, ((co2_display) / 25) * 199);
      } else if (cfg.name.equals("O2")) {
        dacWrite(O2_DAC, (o2_display / 25) * 199);
      }
      if (operation_mode == 0) {
        displayO2CO2Data(o2_display, co2_display);
      }
    } else {
      if (cfg.name.equals("CO2")) {
        dacWrite(CO2_DAC, ((co2_display) / 25) * 199);
      } else if (cfg.name.equals("O2")) {
        dacWrite(O2_DAC, (o2_display / 25) * 199);
      }
    }
    cfg.last_output = current;
  }
}

void vocSensorRoutine(SensorConfig &cfg, HardwareSerial &uart, unsigned long current) {
  // (unchanged – full VOC routine as in original)
  if (!cfg.enabled) return;

  if (voc_auto_reporting) {
    while (uart.available()) {
      uint8_t byte = uart.read();
      if (voc_response_index < sizeof(voc_response_buffer)) {
        voc_response_buffer[voc_response_index++] = byte;
      }
      if (voc_response_index >= 9) {
        VOCResponseData response;
        if (parseVOCResponse(voc_response_buffer, voc_response_index, response)) {
          if (response.command_type == 0x86) {
            if (current - cfg.last_sample >= cfg.sample_interval) {
              cfg.current_value = (response.concentration1 * 1000) + cfg.offset;
              cfg.last_sample = current;
              processData(cfg);
            }
            voc_response_index = 0;
            break;
          }
        }
        voc_response_index = 0;
      }
    }
  } else {
    if (current - cfg.last_sample >= cfg.sample_interval) {
      uint8_t read_cmd[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
      while (uart.available()) uart.read();
      uart.write(read_cmd, sizeof(read_cmd));
      uart.flush();

      unsigned long start_time = millis();
      uint8_t response_buffer[20];
      int response_index = 0;
      VOCResponseData response;
      bool response_received = false;

      while (millis() - start_time < SENSOR_TIMEOUT && !response_received) {
        if (uart.available()) {
          response_buffer[response_index++] = uart.read();
          start_time = millis();
          if (parseVOCResponse(response_buffer, response_index, response)) {
            if (response.command_type == 0x86) {
              cfg.current_value = (response.concentration1 * 1000) + cfg.offset;
              cfg.last_sample = current;
              response_received = true;
              processData(cfg);
            }
          }
          if (response_index >= sizeof(response_buffer)) break;
        }
      }
    }
  }

  if (current - cfg.last_output >= cfg.output_interval) {
    float processed_value = processData(cfg);
    float concentration_ppb = processed_value;
    Serial.printf("[%s] %s: %.1f ppb (%.3f ppm)\n",
                  cfg.name.c_str(), getModeString(cfg.output_mode),
                  concentration_ppb, processed_value / 1000);
    if (display_ok) {
      displayVOCData(processed_value, 0, 0);
      int dacValue = map(processed_value / 1000, 0, 10, 0, 255);
      dacValue = constrain(dacValue, 0, 255);
      dacWrite(VOC_DAC, dacValue);
      Serial.printf("DAC Write: %d/255 (%.1f ppb)\n\n", dacValue, processed_value);
    }
    cfg.last_output = current;
  }
}

void cozirSensorRoutine(SensorConfig &cfg, HardwareSerial &uart, unsigned long current) {
  if (!cfg.enabled) return;

  if (current - cfg.last_sample >= cfg.sample_interval) {
    float raw_value;
    if (readCozirSensor(uart, raw_value)) {
      cfg.current_value = raw_value * COZIR_MODE2_PPM_MULTIPLIER + cfg.offset;
      cfg.last_sample = current;
      cfg.consecutiveErrors = 0;
    } else {
      cfg.consecutiveErrors++;
      Serial.printf("[COZIR] Read timeout #%d\n", cfg.consecutiveErrors);
      cfg.current_value = 0;
      cfg.last_sample = current;
    }

    if (cfg.resetThreshold > 0 && cfg.consecutiveErrors >= cfg.resetThreshold) {
      Serial.printf("[COZIR] Too many errors (%d). Rebooting...\n", cfg.consecutiveErrors);
      delay(500);
      esp_restart();
    }
  }

  float processed_value = processData(cfg);

   if (current - cfg.last_output >= cfg.output_interval) {
    float co2_percent = processed_value / 10000.0;
    Serial.printf("[COZIR] %s: %.3f %%\n",
                  getModeString(cfg.output_mode), co2_percent);

    // Output to DAC (pin 26) – map 0‑5000 ppm to 0‑255
    int dacValue = map(processed_value, 0, 5000, 0, 255);
    dacValue = constrain(dacValue, 0, 255);
    dacWrite(CO2_DAC, dacValue);

    if (display_ok) {
      displayCozirData(co2_percent);   // show CO2 in %
    }
    cfg.last_output = current;
  }
}

bool readCozirSensor(HardwareSerial &uart, float &value) {
  uart.print("Z\r\n");      // request filtered CO₂ reading
  uart.flush();

  unsigned long start = millis();
  while (millis() - start < SENSOR_TIMEOUT) {
    if (uart.available()) {
      String raw = uart.readStringUntil('\n');
      value = extractFirstFloat(raw);
      return !isnan(value);
    }
  }
  return false;
}

bool parseVOCResponse(uint8_t *data, int length, VOCResponseData &response) {
  // (unchanged – full parsing as in original)
  response = { 0 };
  response.valid = false;
  if (length < 2) return false;
  if (data[0] != 0xFF) {
    if (length >= 2 && data[0] == 0x4F && data[1] == 0x4B) {
      response.command_type = 0x4B;
      response.valid = true;
      return true;
    }
    return false;
  }
  uint8_t calculated_checksum = 0;
  int checksum_position = 0;
  switch (data[1]) {
    case 0x86: case 0x87:
      if (length < 9) return false;
      checksum_position = 8;
      for (int i = 1; i < checksum_position; i++) calculated_checksum += data[i];
      break;
    case 0xD7:
      if (length < 8) return false;
      checksum_position = 7;
      for (int i = 1; i < checksum_position; i++) calculated_checksum += data[i];
      break;
    case 0xA1: case 0xA2:
      if (length < 9) return false;
      checksum_position = 8;
      for (int i = 1; i < checksum_position; i++) calculated_checksum += data[i];
      break;
    case 0x8A:
      if (length < 9) return false;
      checksum_position = 8;
      for (int i = 1; i < checksum_position; i++) calculated_checksum += data[i];
      break;
    default:
      return false;
  }
  calculated_checksum = (-calculated_checksum) & 0xFF;
  if (calculated_checksum != data[checksum_position]) {
    Serial.printf("Checksum error: expected %02X, got %02X\n", calculated_checksum, data[checksum_position]);
    return false;
  }
  response.command_type = data[1];
  switch (data[1]) {
    case 0x86:
      response.concentration2 = (float)((data[2] << 8) | data[3]) / 1000.0;
      response.range = (data[4] << 8) | data[5];
      response.concentration1 = (float)((data[6] << 8) | data[7]) / 1000.0;
      response.valid = true;
      break;
    case 0x87:
      if (length < 13) return false;
      response.concentration2 = (float)((data[2] << 8) | data[3]) / 1000.0;
      response.range = (data[4] << 8) | data[5];
      response.concentration1 = (float)((data[6] << 8) | data[7]) / 1000.0;
      response.temperature = (float)((int16_t)((data[8] << 8) | data[9])) / 100.0;
      response.humidity = (float)((data[10] << 8) | data[11]) / 100.0;
      response.valid = true;
      break;
    case 0xD7:
      response.sensor_type = data[2];
      response.range = (data[3] << 8) | data[4];
      response.unit_type = data[5];
      response.decimal_places = data[6] >> 4;
      response.valid = true;
      break;
    case 0xA1: case 0xA2:
      response.valid = true;
      break;
    case 0x8A:
      response.led_status = data[2];
      response.valid = true;
      break;
    default:
      return false;
  }
  return response.valid;
}

void printVOCResponse(const VOCResponseData &response) {
  // (unchanged)
  if (!response.valid) { Serial.println("Invalid response"); return; }
  switch (response.command_type) {
    case 0x4B: Serial.println("Command executed successfully"); break;
    case 0x86: Serial.printf("Concentration1: %.3f, Concentration2: %.3f, Range: %d\n",
                response.concentration1, response.concentration2, response.range); break;
    case 0x87: Serial.printf("Concentration1: %.3f, Concentration2: %.3f, Range: %d, ",
                response.concentration1, response.concentration2, response.range);
               Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n",
                response.temperature, response.humidity); break;
    case 0xD7: Serial.printf("Sensor Type: 0x%02X, Range: %d, Unit Type: 0x%02X, Decimal Places: %d\n",
                response.sensor_type, response.range, response.unit_type, response.decimal_places); break;
    case 0xA1: Serial.println("Entered sleep mode"); break;
    case 0xA2: Serial.println("Exited sleep mode"); break;
    case 0x8A: Serial.printf("LED Status: %s\n", response.led_status ? "ON" : "OFF"); break;
    default: Serial.printf("Unknown response type: 0x%02X\n", response.command_type); break;
  }
}

bool readSensor(HardwareSerial &uart, float &value, String sensor, unsigned long timeoutMs) {
  const char expected = sensor == "CO2" ? 'Z' : '%';

  while (uart.available()) uart.read();

  if (sensor == "CO2") uart.print("Z\r\n");
  else if (sensor == "O2") uart.print("%\r\n");
  else return false;

  uart.flush();

  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (!uart.available()) continue;

    String raw = uart.readStringUntil('\n');
    raw.trim();

    if (raw.length() < 3 ||
        raw[0] != expected ||
        raw[1] != ' ') {
      continue;
    }

    String number = raw.substring(2);
    number.trim();

    if (number.length() == 0) continue;

    bool hasDigit = false;
    bool valid = true;

    for (unsigned int i = 0; i < number.length(); i++) {
      char c = number[i];

      if (c >= '0' && c <= '9') hasDigit = true;
      else if (c != '.' && c != '-' && c != '+') valid = false;
    }

    if (!valid || !hasDigit) continue;

    value = number.toFloat();
    return true;
  }

  return false;
}

float processData(SensorConfig &cfg) {
  switch (cfg.output_mode) {
    case SMA:
      cfg.sma_buffer[cfg.buffer_index] = cfg.current_value;
      cfg.buffer_index = (cfg.buffer_index + 1) % cfg.sma_period;
      cfg.buffer_count = min(cfg.buffer_count + 1, cfg.sma_period);
      return calculateSMA(cfg);
    case EMA:
      cfg.current_ema = (cfg.current_value * cfg.ema_alpha) + (cfg.current_ema * (1 - cfg.ema_alpha));
      return cfg.current_ema;
    default: return cfg.current_value;
  }
}

float calculateSMA(SensorConfig &cfg) {
  float sum = 0;
  for (int i = 0; i < cfg.buffer_count; i++) sum += cfg.sma_buffer[i];
  return sum / cfg.buffer_count;
}

const char *getModeString(OutputMode mode) {
  static const char *modes[] = { "RAW", "SMA", "EMA" };
  return modes[mode];
}

void saveConfig(SensorConfig &cfg) {
  prefs.begin("sensor_cfg", false);
  prefs.putBool((cfg.name + "_enabled").c_str(), cfg.enabled);
  prefs.putFloat((cfg.name + "_offset").c_str(), cfg.offset);
  prefs.putFloat((cfg.name + "_gain").c_str(), cfg.gain);
  prefs.putFloat((cfg.name + "_alpha").c_str(), cfg.alpha);
  prefs.putFloat((cfg.name + "_p").c_str(), cfg.p);
  prefs.putULong((cfg.name + "_sample").c_str(), cfg.sample_interval);
  prefs.putULong((cfg.name + "_output").c_str(), cfg.output_interval);
  prefs.putUInt((cfg.name + "_mode").c_str(), (uint8_t)cfg.output_mode);
  prefs.putInt((cfg.name + "_smap").c_str(), cfg.sma_period);
  prefs.putInt((cfg.name + "_emap").c_str(), cfg.ema_period);
  prefs.end();
}

void loadSensorConfig(SensorConfig &cfg) {
  cfg.enabled = prefs.getBool((cfg.name + "_enabled").c_str(), true);
  cfg.offset = prefs.getFloat((cfg.name + "_offset").c_str(), 0);
  cfg.gain = prefs.getFloat((cfg.name + "_gain").c_str(), 1.0);
  cfg.alpha = prefs.getFloat((cfg.name + "_alpha").c_str(), 0.0);
  cfg.p = prefs.getFloat((cfg.name + "_p").c_str(), 1000.0);
  // Only CO2 defaults to 3000 ms. A previously saved setting takes priority.
  const unsigned long defaultSampleMs = (cfg.name == "CO2") ? 3000UL : 1000UL;
  cfg.sample_interval = prefs.getULong((cfg.name + "_sample").c_str(), defaultSampleMs);
  cfg.output_interval = prefs.getULong((cfg.name + "_output").c_str(), 1000);
  cfg.output_mode = (OutputMode)prefs.getUInt((cfg.name + "_mode").c_str(), 0);
  cfg.sma_period = prefs.getInt((cfg.name + "_smap").c_str(), 5);
  cfg.ema_period = prefs.getInt((cfg.name + "_emap").c_str(), 10);
  cfg.ema_alpha = 2.0 / (cfg.ema_period + 1);

  memset(cfg.sma_buffer, 0, sizeof(cfg.sma_buffer));
  cfg.buffer_index = 0;
  cfg.buffer_count = 0;
}

void processCommand(String cmd) {
  // (unchanged – full command parsing as in original)
  cmd.trim();
  int space1 = cmd.indexOf(' ');
  int space2 = cmd.lastIndexOf(' ');
  if (space1 == -1 || space2 == -1 || space1 == space2) {
    Serial.println("Invalid command format!");
    printHelp();
    return;
  }
  String action = cmd.substring(0, space1);
  String target = cmd.substring(space1 + 1, space2);
  String param = cmd.substring(space2 + 1);
  SensorConfig *config = getSensorConfig(target);
  if (!config) {
    Serial.println("Invalid sensor name");
    return;
  }
  if (action.equalsIgnoreCase("ENABLE")) {
    config->enabled = param.equalsIgnoreCase("ON");
    saveConfig(*config);
    Serial.printf("%s Sensor has been %s\n", config->name.c_str(), config->enabled ? "Enabled" : "Disabled");
    return;
  }
  if (action.equalsIgnoreCase("SET_OFFSET")) {
    config->offset = param.toFloat();
    saveConfig(*config);
    Serial.printf("%s offset set to: %.2f\n", config->name.c_str(), config->offset);
    return;
  } else if (action.equalsIgnoreCase("SET_GAIN")) {
    config->gain = param.toFloat();
    saveConfig(*config);
    Serial.printf("%s gain set to: %.2f\n", config->name.c_str(), config->gain);
    return;
  } else if (action.equalsIgnoreCase("SET_ALPHA")) {
    config->alpha = param.toFloat();
    saveConfig(*config);
    Serial.printf("%s alpha set to: %.2f\n", config->name.c_str(), config->alpha);
    return;
  } else if (action.equalsIgnoreCase("SET_P")) {
    config->p = param.toFloat();
    saveConfig(*config);
    Serial.printf("%s p set to: %.2f\n", config->name.c_str(), config->p);
    return;
  } else if (action.equalsIgnoreCase("SET_SAMPLE")) {
    // Existing sensor sampling interval is now also the CO2 calibration interval.
    // Reject 0/negative/invalid values to avoid continuous UART polling.
    for (unsigned int i = 0; i < param.length(); ++i) {
      if (!isDigit(param[i])) {
        Serial.println("Invalid interval: enter an integer in milliseconds.");
        return;
      }
    }
    unsigned long interval = param.toInt();
    if (param.length() == 0 || interval < 100UL || interval > 3600000UL) {
      Serial.println("Invalid interval: use 100-3600000 ms.");
      return;
    }
    config->sample_interval = interval;
    saveConfig(*config);

    // Read back the value from NVS to confirm it survived the flash write.
    Preferences verifyPrefs;
    bool verifyOpened = verifyPrefs.begin("sensor_cfg", true);
    unsigned long savedInterval = 0;
    if (verifyOpened) {
      savedInterval = verifyPrefs.getULong((config->name + "_sample").c_str(), 0UL);
      verifyPrefs.end();
    }
    Serial.printf("%s sample interval set to: %lu ms\n", config->name.c_str(), config->sample_interval);
    Serial.printf("[%s NVS] Sample interval %s: %lu ms\n", config->name.c_str(),
                  (verifyOpened && savedInterval == interval) ? "saved" : "SAVE FAILED",
                  savedInterval);
    if (config == &co2 && tcal_enabled) {
      resetSampleTimer();
      startTcalCycle();
      Serial.println("[TCAL] Restarted sample batch with new CO2 sample interval.");
    }
    return;
  } else if (action.equalsIgnoreCase("SET_OUTPUT")) {
    config->output_interval = param.toInt();
    saveConfig(*config);
    Serial.printf("%s output interval set to: %dms\n", config->name.c_str(), config->output_interval);
    return;
  } else if (action.equalsIgnoreCase("SET_MODE")) {
    if (param.equalsIgnoreCase("RAW")) config->output_mode = RAW;
    else if (param.equalsIgnoreCase("SMA")) {
      config->output_mode = SMA;
      config->buffer_index = 0;
      config->buffer_count = 0;
    } else if (param.equalsIgnoreCase("EMA")) {
      config->output_mode = EMA;
      config->current_ema = 0;
    }
    saveConfig(*config);
    Serial.printf("%s output mode set to: %s\n", config->name.c_str(), param.c_str());
    return;
  } else if (action.equalsIgnoreCase("SET_SMA_PERIOD")) {
    int period = constrain(param.toInt(), 1, MAX_SMA_PERIOD);
    config->sma_period = period;
    config->buffer_index = 0;
    config->buffer_count = 0;
    saveConfig(*config);
    Serial.printf("%s SMA period set to: %d\n", config->name.c_str(), period);
    return;
  } else if (action.equalsIgnoreCase("SET_EMA_PERIOD")) {
    int period = constrain(param.toInt(), 1, 100);
    config->ema_period = period;
    config->ema_alpha = 2.0 / (period + 1);
    config->current_ema = 0;
    saveConfig(*config);
    Serial.printf("%s EMA period set to: %d (alpha:%.3f)\n", config->name.c_str(), period, config->ema_alpha);
    return;
  } else {
    Serial.println("Invalid command");
    printHelp();
    return;
  }
}

void printHelp() {
  Serial.println("\n=== Sensor Monitoring System ===");
  Serial.println("Current Mode: " + String(operation_mode == 0 ? "O2/CO2" : (operation_mode == 1 ? "VOC" : "COZIR")));
  Serial.println("Command List: ");
  if (!operation_mode) {
    Serial.println("  MODE [0/1/2] - Switch between O2/CO2 (0), VOC (1), COZIR (2)");
    Serial.println("  ENABLE [CO2/O2] [ON/OFF]");
    Serial.println("  SET_MODE [CO2/O2] [RAW/SMA/EMA]");
    Serial.println("  SET_OFFSET [CO2/O2/VOC] [value]");
    Serial.println("  SET_GAIN [O2] [value]");
    Serial.println("  SET_ALPHA [O2] [value]");
    Serial.println("  SET_P [O2] [value]");
    Serial.println("  SET_SAMPLE [CO2/O2] [ms]");
    Serial.println("  SET_OUTPUT [CO2/O2] [ms]");
    Serial.println("  SET_EMA_PERIOD [CO2/O2] [ms]");
    Serial.println("  SET_SMA_PERIOD [CO2/O2] [ms]");
  } else {
    Serial.println("  AUTO - Switch to auto reporting mode");
    Serial.println("  QUERY - Switch to query mode");
    Serial.println("  GAS - Query current concentration");
    Serial.println("  SENSOR1 - Query sensor parameters (Method 1)");
    Serial.println("  SENSOR2 - Query sensor parameters (Method 2)");
    Serial.println("  GASTH - Query Gas Concentration, temperature and humidity");
    Serial.println("  TH1 - Query temperature and humidity");
    Serial.println("  TH2 - Query temperature and humidity");
    Serial.println("  SLEEP1 - Make Sensor Sleep");
    Serial.println("  WAKE1 - Make Sensor Wake Up");
    Serial.println("  SLEEP2 - Make Sensor Sleep");
    Serial.println("  WAKE2 - Make Sensor Wake Up");
    Serial.println("  LEDON - Enable LED");
    Serial.println("  LEDOFF - Disable LED");
    Serial.println("  LED -  Query LED Status");
    Serial.println("  FACTORYRST -  Factory reset sensor callibration");
    Serial.println("  SOFT -  Query Software Version");
    Serial.println("  0xFF 0x01... - Send raw hex command");
  }
  Serial.println("\nCurrent Status:");
  if (operation_mode == 0) {
    Serial.printf("CO2: %s | Mode: %s | Sample: %dms | Output: %dms | Offset: %.2fppm | Gain: %.2f | Alpha: %.2f | p: %.2f\n",
                  co2.enabled ? "Enabled" : "Disabled", getModeString(co2.output_mode),
                  co2.sample_interval, co2.output_interval, co2.offset, co2.gain, co2.alpha, co2.p);
    Serial.printf("O2 : %s | Mode: %s | Sample: %dms | Output: %dms | Offset: %.2f%% | Gain: %.2f | Alpha: %.2f | p: %.2f\n",
                  o2.enabled ? "Enabled" : "Disabled", getModeString(o2.output_mode),
                  o2.sample_interval, o2.output_interval, o2.offset, o2.gain, o2.alpha, o2.p);
  } else if (operation_mode == 1) {
    Serial.printf("VOC: %s | Mode: %s | Sample: %dms | Output: %dms | Offset: %.2fppb\n",
                  voc.enabled ? "Enabled" : "Disabled", getModeString(voc.output_mode),
                  voc.sample_interval, voc.output_interval, voc.offset);
  } else if (operation_mode == 2) {
    Serial.printf("COZIR: %s | Mode: %s | Sample: %dms | Output: %dms | Offset: %.2fppm\n",
                  cozir.enabled ? "Enabled" : "Disabled", getModeString(cozir.output_mode),
                  cozir.sample_interval, cozir.output_interval, cozir.offset);
  }
}

void printCozirHelp() {
  Serial.println("\n=== CozIR-LP CO₂ Sensor Commands ===");
  Serial.println("  MODE0                - Switch to Command Mode");
  Serial.println("  MODE1                - Switch to Streaming Mode (default)");
  Serial.println("  MODE2                - Switch to Polling Mode");
  Serial.println("  FILTER <value>       - Set digital filter (0..65535)");
  Serial.println("  FILTER?              - Read current filter value");
  Serial.println("  ALTITUDE <value>     - Set altitude compensation (8192 = sea level)");
  Serial.println("  ALTITUDE?            - Read altitude compensation");
  Serial.println("  AUTOZERO ON|OFF      - Enable/disable auto-zero");
  Serial.println("  AUTOZERO <init> <reg> - Set intervals in days (e.g., 1.0 8.0)");
  Serial.println("  AUTOZERO?            - Read auto-zero config");
  Serial.println("  ZERO_N2              - Zero in Nitrogen (0 ppm)");
  Serial.println("  ZERO_FRESH           - Zero in Fresh Air (400 ppm)");
  Serial.println("  ZERO_KNOWN <ppm>     - Zero in known CO₂ concentration");
  Serial.println("  ZERO_ADJUST <rep> <act> - Adjust zero using known reading");
  Serial.println("  FORCE_ZERO <value>   - Force a specific zero set point");
  Serial.println("  READ_CO2             - Read filtered CO₂");
  Serial.println("  READ_CO2_RAW         - Read unfiltered CO₂");
  Serial.println("  MASK <mask>          - Set output fields (e.g., 6 = filtered+unfiltered)");
  Serial.println("  SCALE?               - Read scaling factor (always 1)");
  Serial.println("  VERSION              - Read firmware & serial number");
  Serial.println("  HELP                 - Show this help");
  Serial.println("  <any raw ASCII>      - Send any command directly");
}

void printConfigHelp() {
  Serial.println("  MODE [0/1/2] - Switch between O2/CO2 (0), VOC (1), COZIR (2)");
  Serial.println("  ENABLE [CO2/O2/VOC/COZIR] [ON/OFF]");
  Serial.println("  SET_MODE [CO2/O2/VOC/COZIR] [RAW/SMA/EMA]");
  Serial.println("  SET_OFFSET [CO2/O2/VOC/COZIR] [value]");
  Serial.println("  SET_GAIN [O2] [value]");
  Serial.println("  SET_ALPHA [O2] [value]");
  Serial.println("  SET_P [O2] [value]");
  Serial.println("  SET_SAMPLE [CO2/O2/VOC/COZIR] [ms]");
  Serial.println("  SET_OUTPUT [CO2/O2/VOC/COZIR] [ms]");
  Serial.println("  SET_EMA_PERIOD [CO2/O2/VOC/COZIR] [ms]");
  Serial.println("  SET_SMA_PERIOD [CO2/O2/VOC/COZIR] [ms]");

  Serial.println("\nCurrent Status:");
  if (operation_mode == 0) {
    Serial.printf("CO2: %s | Mode: %s | Sample: %dms | Output: %dms | Offset: %.2fppm | Gain: %.2f | Alpha: %.2f | p: %.2f\n",
                  co2.enabled ? "Enabled" : "Disabled", getModeString(co2.output_mode),
                  co2.sample_interval, co2.output_interval, co2.offset, co2.gain, co2.alpha, co2.p);
    Serial.printf("O2 : %s | Mode: %s | Sample: %dms | Output: %dms | Offset: %.2f%% | Gain: %.2f | Alpha: %.2f | p: %.2f\n",
                  o2.enabled ? "Enabled" : "Disabled", getModeString(o2.output_mode),
                  o2.sample_interval, o2.output_interval, o2.offset, o2.gain, o2.alpha, o2.p);
  } else if (operation_mode == 1) {
    Serial.printf("VOC: %s | Mode: %s | Sample: %dms | Output: %dms | Offset: %.2fppb\n",
                  voc.enabled ? "Enabled" : "Disabled", getModeString(voc.output_mode),
                  voc.sample_interval, voc.output_interval, voc.offset);
  } else if (operation_mode == 2) {
    Serial.printf("COZIR: %s | Mode: %s | Sample: %dms | Output: %dms | Offset: %.2fppm\n",
                  cozir.enabled ? "Enabled" : "Disabled", getModeString(cozir.output_mode),
                  cozir.sample_interval, cozir.output_interval, cozir.offset);
  }
}

SensorConfig *getSensorConfig(String target) {
  if (target.equalsIgnoreCase("CO2")) return &co2;
  if (target.equalsIgnoreCase("O2")) return &o2;
  if (target.equalsIgnoreCase("VOC")) return &voc;
  if (target.equalsIgnoreCase("COZIR")) return &cozir; 
  return nullptr;
}


// ============================================
// FUNCTION: sample_timer
// ============================================

void resetSampleTimer() {
  sample_count = 0;
  sample_start_time = millis();
  last_tcal_sample = sample_start_time;
  tcal_last_countdown_print = sample_start_time;
}

void updateCalibrationCountdown(unsigned long current) {
  // Sampling is count-based. Show progress, not an inaccurate fixed deadline.
  // Individual readings are printed inside sample_timer().
  (void)current;
}

void sample_timer(float co2_percent, unsigned long current) {
  if (!tcal_enabled || !tcal_known_set || !tcal_countdown_active) return;
  if (!isfinite(co2_percent) || co2_percent < 0.0f) return;
  if ((uint32_t)(current - last_tcal_sample) < co2.sample_interval) return;
  last_tcal_sample = current;

  if (sample_count >= (int)tcal_target_samples) return;
  co2_samples[sample_count++] = co2_percent;
  Serial.printf("[TCAL] Sample %d/%u = %.5f %%\n",
                sample_count, tcal_target_samples, co2_percent);

  // At 128x64 pixels, show each sample on the bottom OLED line and
  // the live CO2 concentration in the main CO2 row.
  char label[24];
  snprintf(label, sizeof(label), "S %d/%u %.3f%%",
           sample_count, tcal_target_samples, co2_percent);
  tcal_last_sample_label = String(label);
  tcal_last_sample_notice_until = current + co2.sample_interval;
  if (display_ok) displayO2CO2Data(o2_display, co2_display);

  if (sample_count < (int)tcal_target_samples) return;

  // Compare EVERY stored reading with the KNOWN reference, not with the average.
  const float allowed = fabsf(var_Known_CO2) * var_Tolerance / 100.0f;
  unsigned int failures = 0;
  for (int i = 0; i < sample_count; ++i) {
    const float difference = fabsf(co2_samples[i] - var_Known_CO2);
    if (difference > allowed) {
      failures++;
    }
  }

  tcal_countdown_active = false;
  tcal_last_sample_label = "";
  if (failures != 0) {
    tcal_batch_passed = false;
    tcal_enabled = false;
    tcal_oled_notice = "FAIL";
    tcal_oled_notice_until = millis() + 10000UL;
    Serial.println("[TCAL] FAIL");
    if (display_ok) displayO2CO2Data(o2_display, co2_display);
    return;
  }

  Serial.printf("[TCAL] PASS: all %d samples within reference +/- %.2f%%. Sending SST X.\n",
                sample_count, var_Tolerance);
  // This performs a LIVE calibration write. Only use when verified reference
  // gas is physically present at the sensor for the entire batch.
  timed_auto_calibration();
}

void timed_auto_calibration() {
  if (!tcal_enabled || !tcal_known_set || sample_count == 0 ||
      operation_mode != 0 || !co2.enabled) return;

  const float native_reference = var_Known_CO2 * (10000.0f / CO2_PPM_MULTIPLIER);
  const long reference = lroundf(native_reference);
  if (!isfinite(native_reference) || reference <= 0 || reference > 99999 ||
      fabsf(native_reference - (float)reference) > 0.001f) {
    Serial.println("[TCAL] Invalid CO2 reference; calibration cancelled.");
    tcal_enabled = false;
    return;
  }

  // SST UG-011 Appendix A: "X <native concentration>\r\n".
  // This changes the CO2 sensor's stored zero calibration: use only with a
  // verified known gas concentration at the sensor inlet.
  // Disarm first, so no subsequent reading can initiate another write.
  tcal_enabled = false;
  while (CO2_UART.available()) CO2_UART.read();
  CO2_UART.print("X ");
  CO2_UART.print(reference);
  CO2_UART.print("\r\n");
  CO2_UART.flush();

  // SST documents the response as "X #####\r\n", where ##### is a
  // five-digit, *variable zero-point value*, NOT the reference concentration.
  // Do not mistake a command echo, Z reading or timeout for confirmation.
  bool acknowledged = false;
  String zeroPointReply;
  const unsigned long start = millis();
  while (millis() - start < SENSOR_TIMEOUT) {
    if (!CO2_UART.available()) continue;
    String reply = CO2_UART.readStringUntil('\n');
    reply.trim();  // removes leading space and trailing CR, if present
    if (reply.length() != 7 || reply[0] != 'X' || reply[1] != ' ') continue;

    bool fiveDigits = true;
    for (unsigned int i = 2; i < 7; ++i) {
      if (!isDigit(reply[i])) { fiveDigits = false; break; }
    }
    if (!fiveDigits) continue;

    zeroPointReply = reply;
    acknowledged = true;
    break;
  }

  tcal_batch_passed = acknowledged;
  if (acknowledged) {
    // Show the actual SST reply, not merely the acknowledgement label.
    // PASS here means a valid SST X response was received, regardless of
    // whether the first reading passed or later stable samples triggered X.
    // It does not independently verify CO2 concentration accuracy.
    tcal_oled_notice = "PASS " + zeroPointReply;
    tcal_oled_notice_until = millis() + 10000UL;
    Serial.println("[TCAL] CO2 Response: " + zeroPointReply);
    Serial.println("[TCAL] SST X command acknowledged; verify CO2 against reference.");
  } else {
    tcal_oled_notice = "FAIL X ERROR";
    tcal_oled_notice_until = millis() + 5000UL;
    Serial.println("[TCAL] No valid SST X ##### response; calibration unconfirmed.");
  }
  if (display_ok) displayO2CO2Data(o2_display, co2_display);
  sample_count = 0;
}

// ============================================
// MANUAL CALIBRATION COMMANDS
// ============================================

void handleTimedCalibrationCommand(String cmd) {

    cmd.trim();
    cmd.toUpperCase();

    // Retry delay after ANY out-of-tolerance reading in the sample batch.
    if (cmd.startsWith("TCAL DELAY MINUTES ")) {
        String part = cmd.substring(String("TCAL DELAY MINUTES ").length());
        part.trim();
        if (part.length() == 0 || part.length() > 4) {
            Serial.println("[TCAL] Enter delay 1-1440 minutes."); return;
        }
        for (unsigned i=0; i<part.length(); ++i)
            if (!isDigit(part[i])) { Serial.println("[TCAL] Invalid delay minutes."); return; }
        unsigned long val = part.toInt();
        if (val < 1 || val > 1440UL) {
            Serial.println("[TCAL] Enter delay 1-1440 minutes."); return;
        }
        tcal_delay_seconds = (uint32_t)val * 60UL;
        saveTcalSettings();
        Serial.printf("[TCAL] Retry delay set to %lu minute(s).\n", val);
        return;
    }
    if (cmd.startsWith("TCAL DELAY SECONDS ")) {
        String part = cmd.substring(String("TCAL DELAY SECONDS ").length());
        part.trim();
        if (part.length() == 0 || part.length() > 5) {
            Serial.println("[TCAL] Enter delay 1-86400 seconds."); return;
        }
        for (unsigned i=0; i<part.length(); ++i)
            if (!isDigit(part[i])) { Serial.println("[TCAL] Invalid delay seconds."); return; }
        unsigned long val = part.toInt();
        if (val < 1 || val > 86400UL) {
            Serial.println("[TCAL] Enter delay 1-86400 seconds."); return;
        }
        tcal_delay_seconds = (uint32_t)val;
        saveTcalSettings();
        Serial.printf("[TCAL] Retry delay set to %lu second(s).\n", val);
        return;
    }

    // Short interval for bench tests; production can use TCAL DAYS.
    if (cmd.startsWith("TCAL AUTO MINUTES ")) {
        String part = cmd.substring(18); part.trim();
        if (part.isEmpty() || part.length() > 4) {
            Serial.println("[TCAL AUTO] Enter 1-1440 minutes."); return;
        }
        for (unsigned i = 0; i < part.length(); ++i) {
            if (!isDigit(part[i])) {
                Serial.println("[TCAL AUTO] Invalid minutes."); return;
            }
        }
        unsigned long n = part.toInt();
        if (n < 1 || n > 1440UL) {
            Serial.println("[TCAL AUTO] Enter 1-1440 minutes."); return;
        }
        tcal_auto_minutes = (uint32_t)n;
        tcal_auto_use_minutes = true;
        tcal_auto_use_seconds = false;
        saveTcalSettings();
        tcal_auto_started_ms = millis();
        Serial.printf("[TCAL AUTO] Period set to %lu minute(s). Timer restarted.\n", n);
        return;
    }
    if (cmd.startsWith("TCAL AUTO SECONDS ")) {
        String part = cmd.substring(18); part.trim();
        if (part.isEmpty() || part.length() > 6) {
            Serial.println("[TCAL AUTO] Enter 1-86400 seconds."); return;
        }
        for (unsigned i=0; i<part.length(); ++i)
            if (!isDigit(part[i])) { Serial.println("[TCAL AUTO] Invalid seconds."); return; }
        unsigned long n = part.toInt();
        if (n < 1 || n > 86400UL) {
            Serial.println("[TCAL AUTO] Enter 1-86400 seconds."); return;
        }
        tcal_auto_seconds = (uint32_t)n;
        tcal_auto_use_seconds = true;
        tcal_auto_use_minutes = false;
        saveTcalSettings();
        tcal_auto_started_ms = millis();
        Serial.printf("[TCAL AUTO] Period set to %lu seconds. Timer restarted.\n", n);
        return;
    }
    if (cmd.startsWith("TCAL DAYS ")) {
        String part = cmd.substring(10); part.trim();
        if (part.isEmpty() || part.length() > 2) {
            Serial.println("[TCAL AUTO] Enter 1-30 days."); return;
        }
        for (unsigned i=0; i<part.length(); ++i)
            if (!isDigit(part[i])) { Serial.println("[TCAL AUTO] Invalid days."); return; }
        unsigned long n = part.toInt();
        if (n < 1 || n > 30UL) {
            Serial.println("[TCAL AUTO] Enter 1-30 days."); return;
        }
        tcal_auto_days = (uint16_t)n;
        tcal_auto_use_seconds = false;
        tcal_auto_use_minutes = false;
        saveTcalSettings();  // Persist DAYS selection and configured period across reboot.
        tcal_auto_started_ms = millis();
        Serial.printf("[TCAL AUTO] Period set to %lu days. Timer restarted.\n", n);
        return;
    }
    if (cmd == "TCAL AUTO ON") {
        if (tcal_auto_cycle_running) {
            Serial.println("[TCAL AUTO] Already running a calibration cycle."); return;
        }
        tcal_auto_enabled = true;
        saveTcalSettings();
        tcal_auto_start_now = false;  // Wait full configured interval; do not calibrate immediately.
        tcal_retry_waiting = false;
        tcal_auto_started_ms = millis();
        Serial.printf("[TCAL AUTO] Enabled and saved. In NORMAL mode the first batch starts in %lu %s; after PASS it repeats.\n",
            tcalAutoPeriodValue(), tcalAutoPeriodUnit());
        return;
    }
    if (cmd == "TCAL AUTO OFF") {
        tcal_auto_enabled = false;
        saveTcalSettings();
        tcal_auto_start_now = false;
        tcal_retry_waiting = false;
        if (tcal_auto_cycle_running) {
            // Stop the automatic sample batch, but do not alter manual TCAL runs.
            tcal_auto_cycle_running = false;
            tcal_enabled = false;
            tcal_countdown_active = false;
            resetSampleTimer();
        }
        Serial.println("[TCAL AUTO] Disabled. Automatic sampling and retry timer stopped.");
        return;
    }
    if (cmd == "TCAL AUTO STATUS") {
        uint32_t period = tcalAutoPeriodMs();
        uint32_t elapsed = (uint32_t)(millis() - tcal_auto_started_ms);
        uint32_t left = elapsed >= period ? 0 : (period - elapsed + 999UL) / 1000UL;
        Serial.printf("[TCAL AUTO] %s | Period %lu %s | Active cycle %s\n",
            tcal_auto_enabled ? "ON" : "OFF",
            tcalAutoPeriodValue(), tcalAutoPeriodUnit(),
            tcal_auto_cycle_running ? "YES" : "NO");
        if (tcal_retry_waiting) {
            uint32_t delayElapsed = (uint32_t)(millis() - tcal_delay_started_ms);
            left = delayElapsed >= tcal_delay_seconds * 1000UL ? 0UL :
                (tcal_delay_seconds * 1000UL - delayElapsed + 999UL) / 1000UL;
            Serial.println("[TCAL AUTO] State: retry delay after failed batch.");
        } else if (tcal_auto_start_now) {
            left = 0;
            Serial.println("[TCAL AUTO] State: first sample batch pending.");
        }
        Serial.printf("[TCAL AUTO] Next trigger in %lu days %02lu:%02lu:%02lu (timer runs only while powered).\n",
            (unsigned long)(left / 86400UL), (unsigned long)((left / 3600UL) % 24UL),
            (unsigned long)((left / 60UL) % 60UL), (unsigned long)(left % 60UL));
        return;
    }


    if (cmd == "TCAL OFF") {

        tcal_enabled = false;
        tcal_batch_passed = false;
        tcal_first_check_pending = false;
        tcal_countdown_active = false;
        resetSampleTimer();

        Serial.println("[TCAL] Disabled.");
        return;
    }

    if (cmd == "TCAL ON") {

        if (!tcal_known_set ||
            !co2.enabled ||
            operation_mode != 0) {

            Serial.println(
                "[TCAL] Cannot enable. Check CO2 reference, sensor and mode."
            );
            return;
        }

        if (co2.sample_interval == 0 || tcal_target_samples < 1 ||
            tcal_target_samples > TCAL_MAX_SAMPLES) {
            Serial.println("[TCAL] Invalid setup: CO2 sampling interval must be > 0 and samples 1-120.");
            return;
        }

        startTcalCycle();

        Serial.println("[TCAL] Enabled. First CO2 reading will be checked automatically in CMD mode.");
        return;
    }

    if (cmd.startsWith("TCAL SAMPLES ")) {
        String part = cmd.substring(String("TCAL SAMPLES ").length());
        part.trim();
        if (part.length() == 0 || part.length() > 3) {
            Serial.println("[TCAL] Enter 1-120 samples.");
            return;
        }
        for (unsigned int i = 0; i < part.length(); ++i) {
            if (!isDigit(part[i])) {
                Serial.println("[TCAL] Invalid sample count; use whole numbers.");
                return;
            }
        }
        unsigned long value = part.toInt();
        if (value < 1 || value > TCAL_MAX_SAMPLES) {
            Serial.println("[TCAL] Enter 1-120 samples.");
            return;
        }
        tcal_target_samples = (unsigned int)value;
        saveTcalSettings();
        resetSampleTimer();
        if (tcal_enabled) startTcalCycle();
        Serial.printf("[TCAL] Target set to %u samples, CO2 interval %lu ms.\n",
                      tcal_target_samples, co2.sample_interval);
        return;
    }

    if (cmd.startsWith("TCAL INTERVAL ") || cmd.startsWith("TCAL DURATION ")) {
        Serial.println("[TCAL] This command was replaced. Use CONFIG -> SET_SAMPLE CO2 <ms> -> BACK for interval, and TCAL SAMPLES <count> for sample count.");
        return;
    }

    if (cmd.startsWith("TCAL TOLERANCE ")) {

        float value = cmd.substring(15).toFloat();

        if (value <= 0 || value > 100) {
            Serial.println("[TCAL] Invalid tolerance.");
            return;
        }

        var_Tolerance = value;
        saveTcalSettings();
        resetSampleTimer();
        if (tcal_enabled) startTcalCycle();
        Serial.printf("[TCAL] Tolerance set to +/- %.2f%% of the known CO2 concentration.\n", var_Tolerance);
        if (tcal_known_set) {
            const float deviation = fabsf(var_Known_CO2) * var_Tolerance / 100.0f;
            Serial.printf("[TCAL] Accepted CO2 range: %.5f%% to %.5f%%.\n",
                          var_Known_CO2 - deviation, var_Known_CO2 + deviation);
        }
        return;
    }

    if (cmd.startsWith("TCAL KNOWN ")) {

        float value = cmd.substring(11).toFloat();

        float native = value * (10000.0f / CO2_PPM_MULTIPLIER);

        if (!isfinite(value) || value <= 0 ||
           native > 99999 ||
           fabs(native - roundf(native)) > 0.001f) {

            Serial.println("[TCAL] Invalid CO2 reference");
            return;
           }

        var_Known_CO2 = value;
        tcal_known_set = true;
        saveTcalSettings();
        resetSampleTimer();
        if (tcal_enabled) startTcalCycle();

        Serial.printf("[TCAL] Known CO2 reference set to %.5f%% (%.1f ppm).\n",
                      var_Known_CO2, var_Known_CO2 * 10000.0f);
        const float deviation = fabsf(var_Known_CO2) * var_Tolerance / 100.0f;
        Serial.printf("[TCAL] With +/- %.2f%% relative tolerance, each sample must be %.5f%% to %.5f%% CO2.\n",
                      var_Tolerance, var_Known_CO2 - deviation, var_Known_CO2 + deviation);
        return;
    }

    if (cmd == "TCAL STATUS") {
        Serial.printf("TCAL AUTO: %s (saved across reboot)\n", tcal_auto_enabled ? "ON" : "OFF");
        Serial.printf(
            "Enabled: %s\nCO2 sensor sampling interval: %lu ms\nTarget samples: %u\nTolerance: %.2f %%\nKnown CO2: %.3f %%\nReference set: %s\nRetry delay: %lu sec\nSamples collected: %d/%u\n",
            tcal_enabled ? "YES" : "NO",
            co2.sample_interval,
            tcal_target_samples,
            var_Tolerance,
            var_Known_CO2,
            tcal_known_set ? "YES" : "NO",
            (unsigned long)tcal_delay_seconds,
            sample_count, tcal_target_samples
        );
        // Timer settings: keep the original compact TCAL STATUS format.
        Serial.printf("TCAL DELAY SECONDS: %lu sec\n",
                      (unsigned long)tcal_delay_seconds);
        Serial.printf("TCAL AUTO %s: %lu %s\n",
                      tcal_auto_use_minutes ? "MINUTES" :
                      (tcal_auto_use_seconds ? "SECONDS" : "DAYS"),
                      tcalAutoPeriodValue(), tcalAutoPeriodUnit());
        if (tcal_known_set) {
            const float deviation = fabsf(var_Known_CO2) * var_Tolerance / 100.0f;
            Serial.printf("Accepted range: %.5f%% to %.5f%% CO2 (tolerance relative to reference).\n",
                          var_Known_CO2 - deviation, var_Known_CO2 + deviation);
        }
        if (tcal_countdown_active) {
          Serial.printf("Current batch: %d/%u samples collected.\n", sample_count, tcal_target_samples);
        }
        return;
    }

    Serial.println("[TCAL] Unknown command.");
}