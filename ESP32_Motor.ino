/*
   Install ESP-Mail-Client library by Mobizt
   ESP32 Rover + ESP32-CAM stream + Motor Control + Gas Sensor + Blynk + mDNS
*/

#define BLYNK_TEMPLATE_ID "TMPL3aFPDopeM"
#define BLYNK_TEMPLATE_NAME "Smart dustbin"
#define BLYNK_AUTH_TOKEN "wjKzlfJfcuuzvxmbqSMD9PzZ6whLxof3"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
#include <ESPmDNS.h>   // <-- for hostname
#include <ESP_Mail_Client.h>

// ===== WiFi + Blynk =====
char auth[] = BLYNK_AUTH_TOKEN;  
const char* ssid = "Embedded";
const char* password = "0987654321";

// ESP32-CAM stream URL
const char* cam_url = "http://esp32cam.local/stream";

// Gmail SMTP server
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

#define AUTHOR_EMAIL "xxxxx@gmail.com"
#define AUTHOR_PASSWORD "xxxxxxx"   // Gmail App Password
#define RECIPIENT_EMAIL "xxxxxxx@gmail.com"

// Define the SMTP Session object
SMTPSession smtp;
void smtpCallback(SMTP_Status status);

LiquidCrystal_I2C lcd(0x27, 16, 2);

//===========GPS=================
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

HardwareSerial gpsSerial(2);

// ===== Motor control pins =====
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33

// ===== Gas sensor pins (safe ADC pins) =====
#define GAS_SENSOR1 32
#define GAS_SENSOR2 34

#define moisture 14
#define relay 18

// Motor speed (PWM duty 0-255)
int motorSpeed = 200; 

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

// ===== Motor control functions (PWM with ledc) =====
void forward() {
  //Serial.println("Forward");
  ledcWrite(0, motorSpeed); ledcWrite(1, 0);
  ledcWrite(2, motorSpeed); ledcWrite(3, 0);
  Serial.println("Forward");
}
void backward() {
  ledcWrite(0, 0); ledcWrite(1, motorSpeed);
  ledcWrite(2, 0); ledcWrite(3, motorSpeed);
  Serial.println("Backward");
}
void left() {
  ledcWrite(0, 0); ledcWrite(1, motorSpeed);
  ledcWrite(2, motorSpeed); ledcWrite(3, 0);
  Serial.println("Left");
}
void right() {
  ledcWrite(0, motorSpeed); ledcWrite(1, 0);
  ledcWrite(2, 0); ledcWrite(3, motorSpeed);
  Serial.println("Right");
}
void stopMotor() {
  ledcWrite(0, 0); ledcWrite(1, 0);
  ledcWrite(2, 0); ledcWrite(3, 0);
  Serial.println("Stop");
}

// ===== Blynk Virtual Pin Handler =====
BLYNK_WRITE(V0) {
  int value = param.asInt();
  if (value == 1) {
    Serial.println("Blynk V0 HIGH - Sending GPS email");
    String gpsData = "";
    unsigned long startTime = millis();

    //Try to read GPS for up to 3 seconds
    while(millis() - startTime < 3000)
    {
      while(gpsSerial.available())
      {
        char c = gpsSerial.read();
        gpsData += c;
        if(c == '\n')
        {
          break;
        }
      }
      if(gpsData.indexOf("\n") > 0)break;
    }
    if(gpsData.length() > 5)
    {
      Serial.println("GPS Data Captured: "+ gpsData);
      sendMail(gpsData);
    }
    else
      Serial.println("No GPS data received.");
  }
}

void sendGasSensor() {
  int gas1 = analogRead(GAS_SENSOR1);
  int gas2 = analogRead(GAS_SENSOR2);

  Blynk.virtualWrite(V1, gas1);
  Blynk.virtualWrite(V2, gas2);

  lcd.setCursor(0, 0);
  lcd.print("Gas1: ");
  lcd.print(gas1);
  lcd.print("    ");

  lcd.setCursor(0, 1);
  lcd.print("Gas2: ");
  lcd.print(gas2);
  lcd.print("    ");
}

// Callback function to show status
void smtpCallback(SMTP_Status status) {
  Serial.println(status.info());
  if (status.success()) {
    Serial.printf("Message sent success: %d\n", status.completedCount());
    Serial.printf("Message sent failed: %d\n", status.failedCount());
  }
}

void sendMail(String gpsText){
  if (gpsText.length() < 5) {
    Serial.println("No GPS data yet.");
    return;
  }

  Session_Config config;
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = "";

  SMTP_Message message;
  message.sender.name = F("ESP32 Rover");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = F("GPS Alert");
  message.addRecipient(F("Receiver"), RECIPIENT_EMAIL);

  String textMsg = "Alert from Rover! \n\nGPS Data:\n" + gpsText;
  message.text.content = textMsg.c_str();

  if (!smtp.connect(&config)) {
    Serial.println("Failed to connect to mail server");
    return;
  }
  if (!MailClient.sendMail(&smtp, &message)) {
    Serial.print("Error sending Email, ");
    Serial.println(smtp.errorReason());
  } else {
    Serial.println("Email sent Successfully!");
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("GPS Serial started");

  // Motor PWM setup
  ledcAttachPin(IN1, 0); ledcAttachPin(IN2, 1);
  ledcAttachPin(IN3, 2); ledcAttachPin(IN4, 3);
  ledcSetup(0, 5000, 8); ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8); ledcSetup(3, 5000, 8);
  stopMotor();

  pinMode(moisture, INPUT); 
  pinMode(relay, OUTPUT);

  lcd.init();
  lcd.backlight();

  Blynk.begin(auth, ssid, password);

  if (!MDNS.begin("esp32rover")) {
    Serial.println("Error starting mDNS!");
  } else {
    Serial.println("mDNS responder started: http://esp32rover.local/");
  }

  MailClient.networkReconnect(true);
  smtp.debug(1);
  smtp.callback(smtpCallback);

  String page = MAIN_page;
  page.replace("%CAM_URL%", cam_url);

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

  if(digitalRead(moisture)== 0) digitalWrite(relay,HIGH);
  else digitalWrite(relay,LOW);

  static unsigned long lastSend = 0;
  if (millis() - lastSend > 2000) {
    sendGasSensor();
    lastSend = millis();
  }

}
