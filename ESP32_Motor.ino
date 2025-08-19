/*
 * ESP32 Rover + ESP32-CAM stream + Motor Control + Gas Sensor + Blynk + mDNS
 */

#define BLYNK_TEMPLATE_ID "TMPL3aFPDopeM"
#define BLYNK_TEMPLATE_NAME "Smart dustbin"
#define BLYNK_AUTH_TOKEN "wjKzlfJfcuuzvxmbqSMD9PzZ6whLxof3"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
#include <ESPmDNS.h>   // <-- for hostname

// ===== WiFi + Blynk =====
char auth[] = BLYNK_AUTH_TOKEN;  // replace with your Blynk token
const char* ssid = "Embedded";
const char* password = "0987654321";

// ESP32-CAM stream URL (hostname based, not IP)
const char* cam_url = "http://esp32cam.local/stream";

// ===== Motor control pins =====
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25

// ===== Gas sensor pins =====
#define GAS_SENSOR1 33
#define GAS_SENSOR2 35

// Motor speed (PWM duty) hardcoded = 200
int motorSpeed = 200; // range 0-255

WebServer server(80);

// ===== HTML Page =====
const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Rover Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: Arial; text-align: center; margin:0; background-color: #222; color:white;}
  #videoStream { width: 100%; height: 50vh; object-fit: cover; background:black; }
  .btn {
    width: 100px; height: 50px; margin: 10px; font-size: 16px;
    background-color: #4CAF50; color: white; border: none; border-radius: 8px;
  }
  .btn:active { background-color: #45a049; }
</style>
</head>
<body>
<h2>ESP32 Rover Control</h2>
<img id="videoStream" src="%CAM_URL%" onerror="reloadStream()">
<div>
  <button class="btn" onclick="fetch('/forward')">Forward</button><br>
  <button class="btn" onclick="fetch('/left')">Left</button>
  <button class="btn" onclick="fetch('/stop')">Stop</button>
  <button class="btn" onclick="fetch('/right')">Right</button><br>
  <button class="btn" onclick="fetch('/backward')">Backward</button>
</div>
<script>
function reloadStream(){
  const img = document.getElementById('videoStream');
  img.src = '%CAM_URL%' + '?t=' + new Date().getTime();
}
setInterval(reloadStream, 5000);
</script>
</body>
</html>
)rawliteral";

// ===== Motor control functions =====
void forward() {
  Serial.println("Forward");
  analogWrite(IN1, motorSpeed); digitalWrite(IN2, LOW);
  analogWrite(IN3, motorSpeed); digitalWrite(IN4, LOW);
}
void backward() {
  Serial.println("Backward");
  digitalWrite(IN1, LOW); analogWrite(IN2, motorSpeed);
  digitalWrite(IN3, LOW); analogWrite(IN4, motorSpeed);
}
void left() {
  Serial.println("Left");
  digitalWrite(IN1, LOW); analogWrite(IN2, motorSpeed);
  analogWrite(IN3, motorSpeed); digitalWrite(IN4, LOW);
}
void right() {
  Serial.println("Right");
  analogWrite(IN1, motorSpeed); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); analogWrite(IN4, motorSpeed);
}
void stopMotor() {
  Serial.println("Stop");
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ===== Blynk Virtual Pin Handler =====
BLYNK_WRITE(V0) {
  int value = param.asInt();
  if (value == 1) {
    Serial.println("Blynk V0 HIGH");
  }
}

void sendGasSensor() {
  int gas1 = analogRead(GAS_SENSOR1);
  int gas2 = analogRead(GAS_SENSOR2);

  Blynk.virtualWrite(V1, gas1);
  Blynk.virtualWrite(V2, gas2);
}

void setup() {
  Serial.begin(115200);

  // Motor pin setup
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotor();

  // WiFi + Blynk
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi Connected, IP: ");
  Serial.println(WiFi.localIP());

  Blynk.begin(auth, ssid, password);

  // Start mDNS with hostname "esp32rover"
  if (!MDNS.begin("esp32rover")) {
    Serial.println("Error starting mDNS!");
  } else {
    Serial.println("mDNS responder started: http://esp32rover.local/");
  }

  // Prepare HTML page
  String page = MAIN_page;
  page.replace("%CAM_URL%", cam_url);

  // Routes
  server.on("/", [page]() { server.send(200, "text/html", page); });
  server.on("/forward", [](){ forward(); server.send(200,"text/plain","Forward"); });
  server.on("/backward", [](){ backward(); server.send(200,"text/plain","Backward"); });
  server.on("/left", [](){ left(); server.send(200,"text/plain","Left"); });
  server.on("/right", [](){ right(); server.send(200,"text/plain","Right"); });
  server.on("/stop", [](){ stopMotor(); server.send(200,"text/plain","Stop"); });

  server.begin();
}

void loop() {
  server.handleClient();
  Blynk.run();

  static unsigned long lastSend = 0;
  if (millis() - lastSend > 2000) { // send every 2 sec
    sendGasSensor();
    lastSend = millis();
  }
}
