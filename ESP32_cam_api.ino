// ESP32-CAM + Edge Impulse + MJPEG stream + mDNS (esp32cam.local)
// - DHCP (auto IP)
// - FIX: ei_camera_* functions are GLOBAL (not static) to resolve linker errors
//- If object detect call the api 


//------------------------------------------------
#define BLYNK_TEMPLATE_ID "TMPL3aFPDopeM"
#define BLYNK_TEMPLATE_NAME "Smart dustbin"
#define BLYNK_AUTH_TOKEN "wjKzlfJfcuuzvxmbqSMD9PzZ6whLxof3"

#define BLYNK_PRINT Serial
/* Includes ---------------------------------------------------------------- */
#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include <Smart_Dustbin_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <string.h>
#include <ESPmDNS.h>  // mDNS
#include <BlynkSimpleEsp32.h>
#include <HTTPClient.h>

BlynkTimer timer;

/* ---------------- Camera model ---------------- */
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

/* ---------------- WiFi ---------------- */
const char *WIFI_SSID = "Embedded";
const char *WIFI_PASS = "0987654321";

/* ---------------- EI image sizes ---------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

//------------------Variables------------------------
bool dustbinFullDetected = false;

/* ---------------- Globals ---------------- */
static bool debug_nn = false;
static bool is_initialised = false;
static uint8_t *snapshot_buf = nullptr;

/* Latest prediction text */
static char last_prediction[256] = "No prediction";

/* Camera config */
static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_JPEG,  // JPEG for streaming
  .frame_size = FRAMESIZE_QVGA,    // 320x240 (fits EI buffer)
  .jpeg_quality = 12,
  .fb_count = 2,
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* --------- Forward decls (GLOBAL, not static!) --------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
static void startCameraServer();
static void connectWiFi();

/* --------- Camera access lock --------- */
static SemaphoreHandle_t cameraLock;

/* --------- HTTP server handle --------- */
static httpd_handle_t stream_httpd = NULL;

//-----------------------------------------------------

void apiCall() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String a = dustbinFullDetected ? "1" : "0";
    String url = "http://blynk.cloud/external/api/update?token=" + String(BLYNK_AUTH_TOKEN) + "&V0=" + a;
    //String url = "https://blr1.blynk.cloud/external/api/update?token=KwR8o3R8NP9oMX3LLcINozWbw8XTyBk5&V0=1";
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode > 0) {
      Serial.printf("Blynk API Response: %d\n", httpCode);
    } else
      Serial.printf("Blynk API Call failed: %s\n", http.errorToString(httpCode).c_str());
    http.end();
  }
}

void myTimerEvent() {
  static bool ledState = false;
  ledState = !ledState;  // toggle

  if (ledState)
    Blynk.virtualWrite(V4, 255);  // LED ON in app
  else
    Blynk.virtualWrite(V4, 0);  // LED OFF in app
}

/* =================== Arduino setup =================== */
void setup() {
  Serial.begin(115200);
  Serial.println();
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  Serial.println("Connecting to Blynk....");
  Serial.println("Edge Impulse + Live Stream (ESP32-CAM) with mDNS");
  cameraLock = xSemaphoreCreateMutex();

  if (!ei_camera_init()) {
    Serial.println("Failed to initialize Camera!");
  } else {
    Serial.println("Camera initialized");
  }

  connectWiFi();
  startCameraServer();

  ei_printf("\nStarting continuous inference in 2 seconds...\n");
  ei_sleep(2000);
  timer.setInterval(500L, myTimerEvent);
}

/* =================== Arduino loop ==================== */
void loop() {
  Blynk.run();
  timer.run();
  apiCall();
  // Device is working fine -> V4 LED ON

  if (ei_sleep(5) != EI_IMPULSE_OK) return;

  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
  if (snapshot_buf == nullptr) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (!ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH,
                         (size_t)EI_CLASSIFIER_INPUT_HEIGHT,
                         snapshot_buf)) {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    snapshot_buf = nullptr;
    return;
  }

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    free(snapshot_buf);
    snapshot_buf = nullptr;
    return;
  }

  // ---- Build compact prediction string safely ----
  size_t n = 0;
  last_prediction[0] = '\0';

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  uint32_t shown = 0;
  for (uint32_t i = 0; i < result.bounding_boxes_count && shown < 3; i++) {
    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
    if (bb.value == 0) continue;
    int wrote = snprintf(last_prediction + n, sizeof(last_prediction) - n,
                         "%s(%.2f) x:%u y:%u w:%u h:%u | ",
                         bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    if (wrote < 0 || (size_t)wrote >= sizeof(last_prediction) - n) { break; }
    n += (size_t)wrote;
    shown++;
    if (strcmp(bb.label, "dustbin_Full") == 0) {
      dustbinFullDetected = true;
      Serial.println("Dustbin Full");
    } else {
      dustbinFullDetected = false;
    }
  }
  if (shown == 0) {
    snprintf(last_prediction, sizeof(last_prediction), "No objects");
    dustbinFullDetected = false;
  }
#else
  float best_val = -1.0f;
  const char *best_label = "?";
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > best_val) {
      best_val = result.classification[i].value;
      best_label = ei_classifier_inferencing_categories[i];
    }
  }
  snprintf(last_prediction, sizeof(last_prediction), "%s: %.2f", best_label, best_val);
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
  n = strlen(last_prediction);
  snprintf(last_prediction + n, sizeof(last_prediction) - n, "  Anom: %.3f", result.anomaly);
#endif

  ei_printf("Pred: %s\n", last_prediction);

  free(snapshot_buf);
  snapshot_buf = nullptr;
}

/* =================== EI camera init/deinit =================== */
bool ei_camera_init(void) {
  if (is_initialised) return true;

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  // Optional sensor tweaks
  sensor_t *s = esp_camera_sensor_get();
  if (s && s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, 0);
  }

  is_initialised = true;
  return true;
}

void ei_camera_deinit(void) {
  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    ei_printf("Camera deinit failed\n");
    return;
  }
  is_initialised = false;
}

/* =================== EI capture & get_data =================== */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = nullptr;

  // Lock camera while grabbing a frame (avoid conflict with /stream)
  xSemaphoreTake(cameraLock, portMAX_DELAY);
  fb = esp_camera_fb_get();
  if (!fb) {
    xSemaphoreGive(cameraLock);
    ei_printf("Camera capture failed\n");
    return false;
  }

  // Convert JPEG -> RGB888 into snapshot_buf
  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(fb);
  xSemaphoreGive(cameraLock);

  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }

  // Resize/crop from snapshot_buf -> out_buf if sizes differ
  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    ei::image::processing::crop_and_interpolate_rgb888(
      snapshot_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height);
  } else {
    // Same size, just copy
    memcpy(out_buf, snapshot_buf,
           EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3);
  }

  return true;
}

int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    // BGR->RGB swap (common esp32-camera quirk)
    out_ptr[out_ptr_ix] =
      (snapshot_buf[pixel_ix + 2] << 16) | (snapshot_buf[pixel_ix + 1] << 8) | (snapshot_buf[pixel_ix + 0]);
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid Edge Impulse model for camera sensor"
#endif

/* =================== WiFi + Web server =================== */
static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // mDNS start
  if (!MDNS.begin("esp32cam")) {
    Serial.println("Error starting mDNS!");
  } else {
    Serial.println("mDNS responder started");
    Serial.println("You can now access: http://esp32cam.local/");
  }
}

/* ---- Simple index page ---- */
static esp_err_t index_handler(httpd_req_t *req) {
  static const char PROGMEM resp[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'/>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>ESP32-CAM Live</title>"
    "<style>body{font-family:sans-serif;margin:0;background:#111;color:#eee;}"
    ".wrap{max-width:900px;margin:20px auto;padding:12px}"
    "img{width:100%;height:auto;border-radius:16px}"
    ".pill{display:inline-block;padding:6px 10px;border-radius:12px;background:#222}</style>"
    "</head><body><div class='wrap'><h2>ESP32-CAM Live Stream</h2>"
    "<p>Endpoints: <code>/stream</code>, <code>/label</code></p>"
    "<img id='v' src='/stream'/>"
    "<p><b>Prediction:</b> <span class='pill' id='lbl'>Loading...</span></p>"
    "<script>"
    "setInterval(async()=>{try{let r=await fetch('/label');let t=await r.text();"
    "document.getElementById('lbl').innerText=t;}catch(e){}},1000);"
    "</script></div></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, resp, sizeof(resp) - 1);
}

/* ---- MJPEG stream handler ---- */
static esp_err_t stream_handler(httpd_req_t *req) {
  static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
  static const char *_STREAM_BOUNDARY = "\r\n--frame\r\n";
  static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

  char part_buf[64];
  camera_fb_t *fb = NULL;

  while (true) {
    xSemaphoreTake(cameraLock, portMAX_DELAY);
    fb = esp_camera_fb_get();
    if (!fb) {
      xSemaphoreGive(cameraLock);
      Serial.println("Camera frame failed");
      break;
    }

    if (httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) != ESP_OK) {
      esp_camera_fb_return(fb);
      xSemaphoreGive(cameraLock);
      break;
    }
    int hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, (unsigned)fb->len);
    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
      esp_camera_fb_return(fb);
      xSemaphoreGive(cameraLock);
      break;
    }
    esp_err_t res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    xSemaphoreGive(cameraLock);
    if (res != ESP_OK) break;

    vTaskDelay(1);
  }

  return ESP_OK;
}

/* ---- Label handler ---- */
static esp_err_t label_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, last_prediction, strlen(last_prediction));
}

/* ---- Start web server ---- */
static void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_resp_headers = 16;
  config.lru_purge_enable = true;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };
  httpd_uri_t label_uri = {
    .uri = "/label",
    .method = HTTP_GET,
    .handler = label_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &label_uri);
    Serial.println("HTTP server started:");
    Serial.printf("  http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.printf("  http://%s/stream\n", WiFi.localIP().toString().c_str());
    Serial.printf("  http://%s/label\n", WiFi.localIP().toString().c_str());
    Serial.println("Or use: http://esp32cam.local/");
  } else {
    Serial.println("HTTP server start failed");
  }
}
