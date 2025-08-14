
/* Live Stream + Edge Impulse (ESP32-CAM AI-Thinker)
 * - Serves MJPEG at /stream and a simple viewer at /
 * - Runs Edge Impulse inference in loop()
 * - Turns LED (GPIO 4) ON when "cap" is detected, else OFF
 * - Uses a fixed IP address and corrects camera orientation
 *
 * Tested with Arduino-ESP32 2.0.4+
 * Final code
 */

#define BLYNK_TEMPLATE_ID "TMPL3vCDq6M9Z"
#define BLYNK_TEMPLATE_NAME "Demo"
#define BLYNK_AUTH_TOKEN "S5yOTY60DZWwHZesjwvvu7SnosPXOP9Q"

#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "Arduino.h"

#include <Toy_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

BlynkTimer timer;

#define LED 4
int data_label = 0; // cap = 0, without cap = 1
// ------------ Camera model & pins (AI Thinker) ------------
#define CAMERA_MODEL_AI_THINKER
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
#error "Select CAMERA_MODEL_AI_THINKER"
#endif

// ------------ WiFi ------------
const char *WIFI_SSID = "****";
const char *WIFI_PASS = "*****";

// Set static IP
IPAddress local_IP(192, 168, 1, 200);  // Change to your desired IP
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// ------------ EI capture constants ------------
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf = nullptr;  // RGB888 buffer for EI

// Camera config (JPEG @ QVGA)
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
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size = FRAMESIZE_QVGA,  // 320x240
  .jpeg_quality = 12,
  .fb_count = 2,
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

// Forward decls
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void startCameraServer();

// ------------- Camera access lock -------------
SemaphoreHandle_t cameraLock;

// --------------------- HTTP server ----------------------
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t index_handler(httpd_req_t *req) {
  static const char PROGMEM html[] =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'/>"
    "<title>ESP32-CAM Stream</title></head><body style='margin:0;background:#000;'>"
    "<div style='display:flex;justify-content:center;align-items:center;height:100vh;'>"
    "<img src='/stream' style='max-width:100%;height:auto;'/>"
    "</div></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}


static esp_err_t stream_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);

  while (true) {
    if (xSemaphoreTake(cameraLock, portMAX_DELAY) != pdTRUE) {
      return ESP_FAIL;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    xSemaphoreGive(cameraLock);

    if (!fb) {
      continue;
    }

    esp_err_t res;
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    char part_buf[64];
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) { break; }
  }

  return ESP_OK;
}

void myTimerEvent()
{
  if(data_label == 0)
    Blynk.virtualWrite(V0, 0);
  else
   Blynk.virtualWrite(V0, 1);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_handle_t httpd = NULL;
  if (httpd_start(&httpd, &config) == ESP_OK) {
    httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(httpd, &index_uri);

    httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(httpd, &stream_uri);
  }
}

// --------------------------------- Arduino setup ---------------------------------
void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  pinMode(LED, OUTPUT);

  // Set static IP before WiFi connect
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  // WiFi connect
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  // Camera init
  if (!ei_camera_init()) {
    Serial.println("Camera init failed! Rebooting...");
    delay(3000);
    ESP.restart();
  }

  cameraLock = xSemaphoreCreateMutex();

  startCameraServer();
  Serial.println("Open http://" + WiFi.localIP().toString() + "/ to view stream");

  ei_printf("\nStarting continuous inference in 2 seconds...\n");
  ei_sleep(2000);
  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
}

// --------------------------------- Arduino loop ----------------------------------
void loop() {
  Blynk.run();
  timer.run();
  static unsigned long last_run = 0;
  const unsigned long INTERVAL_MS = 1500;

  if (millis() - last_run >= INTERVAL_MS) {
    last_run = millis();

    snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if (!snapshot_buf) {
      ei_printf("ERR: Failed to allocate snapshot buffer!\n");
      return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (!ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH,
                           (size_t)EI_CLASSIFIER_INPUT_HEIGHT,
                           snapshot_buf)) {
      ei_printf("Failed to capture image for EI\r\n");
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

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)\n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    bool led_on = false;
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
      ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
      if (bb.value == 0) { continue; }
      ei_printf("  %s (%.2f) [x:%u y:%u w:%u h:%u]\r\n",
                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
      if (strcmp(bb.label, "cap") == 0) led_on = true;
      if(led_on)
        data_label = 0;
      else
       data_label = 1;
    }
    digitalWrite(LED, led_on ? HIGH : LOW);
#else
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      ei_printf("  %s: %.5f\r\n",
                ei_classifier_inferencing_categories[i],
                result.classification[i].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

    free(snapshot_buf);
    snapshot_buf = nullptr;
  }

  ei_sleep(5);
}

// ------------------------------ EI camera helpers -------------------------------
bool ei_camera_init(void) {
  if (is_initialised) return true;

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  // Camera orientation fix
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);    // 0 = normal, 1 = flip vertically
    s->set_hmirror(s, 1);  // 0 = normal, 1 = mirror horizontally
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

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  if (xSemaphoreTake(cameraLock, portMAX_DELAY) != pdTRUE) return false;
  camera_fb_t *fb = esp_camera_fb_get();
  xSemaphoreGive(cameraLock);

  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, snapshot_buf);

  if (xSemaphoreTake(cameraLock, portMAX_DELAY) == pdTRUE) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(cameraLock);
  }

  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }

  bool do_resize = (img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS);
  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width, img_height);
  }

  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  for (size_t i = 0; i < length; i++) {
    uint8_t r = snapshot_buf[pixel_ix + 0];
    uint8_t g = snapshot_buf[pixel_ix + 1];
    uint8_t b = snapshot_buf[pixel_ix + 2];
    out_ptr[i] = (r << 16) | (g << 8) | (b);
    pixel_ix += 3;
  }
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
