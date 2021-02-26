/*
  Inspired by and based on this Instructable: $9 RTSP Video Streamer Using the ESP32-CAM Board
  (https://www.instructables.com/id/9-RTSP-Video-Streamer-Using-the-ESP32-CAM-Board/)

  Board: AI-Thinker ESP32-CAM or ESP-EYE
  Compile as:
   ESP32 Dev Module
   CPU Freq: 240
   Flash Freq: 80
   Flash mode: QIO
   Flash Size: 4Mb
   Patrition: Minimal SPIFFS
   PSRAM: Enabled
*/

// ESP32 has two cores: APPlication core and PROcess core (the one that runs ESP32 SDK stack)
#define APP_CPU 1
#define PRO_CPU 0

#include "settings.h"
#include "camera_pins.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>

#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include "soc/soc.h" // disable brownout problems
#include "soc/rtc_cntl_reg.h"  // disable brownout problems

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define MOTOR_FORWARDS 0
#define MOTOR_BACKWARDS 1

#ifdef ENABLE_CAM
TaskHandle_t tCamServer;

OV2640 cam;
WebServer camServer(80);

const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

void handle_jpg_stream(void) {
  char buf[32];
  int s;

  WiFiClient client = camServer.client();

  client.write(HEADER, hdrLen);
  client.write(BOUNDARY, bdrLen);

  while (true) {
    if (!client.connected()) break;
    cam.run();
    s = cam.getSize();
    client.write(CTNTTYPE, cntLen);
    sprintf( buf, "%d\r\n\r\n", s );
    client.write(buf, strlen(buf));
    client.write((char *)cam.getfb(), s);
    client.write(BOUNDARY, bdrLen);
  }
}

const char JHEADER[] = "HTTP/1.1 200 OK\r\n" \
                       "Content-disposition: inline; filename=capture.jpg\r\n" \
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

void handle_jpg(void) {
  WiFiClient client = camServer.client();

  if (!client.connected()) return;
  cam.run();
  client.write(JHEADER, jhdLen);
  client.write((char *)cam.getfb(), cam.getSize());
}

void handleNotFound() {
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += camServer.uri();
  message += "\nMethod: ";
  message += (camServer.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += camServer.args();
  message += "\n";
  camServer.send(200, "text / plain", message);
}

void setupCam() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  // Configure the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame parameters: pick one
  //  config.frame_size = FRAMESIZE_UXGA; // 1600x1200
  //  config.frame_size = FRAMESIZE_SVGA; // 800x600
  // config.frame_size = FRAMESIZE_VGA; // 640x480
  //  config.frame_size = FRAMESIZE_QVGA; // 320x240

  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  cam.init(config);
}

void camServerTask(void* pvParameters) {
  camServer.on("/mjpeg/1", HTTP_GET, handle_jpg_stream);
  camServer.on("/jpg", HTTP_GET, handle_jpg);
  camServer.onNotFound(handleNotFound);
  camServer.begin();

  while (true) {
    camServer.handleClient();
  }
}

void setupCamServer() {
  //  Creating task to push the stream to all connected clients
  xTaskCreatePinnedToCore(
    camServerTask,
    "camServer",
    4 * 1024,
    NULL, //(void*) handler,
    2,
    &tCamServer,
    APP_CPU);

  IPAddress ip = WiFi.localIP();
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");
}
#endif

#ifdef ENABLE_OTA
void setupOTA() {
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}
#endif

#ifdef ENABLE_CMD
WiFiServer cmdServer(1337);
TaskHandle_t tCmdServer;

void setMotor(int side, int speed) {
  int motorPin1 = (side == MOTOR_LEFT) ? MOTOR_LEFT_PIN_1 : MOTOR_RIGHT_PIN_1;
  int motorPin2 = (side == MOTOR_LEFT) ? MOTOR_LEFT_PIN_2 : MOTOR_RIGHT_PIN_2;
  int motorPwmChannel = (side == MOTOR_LEFT) ? MOTOR_LEFT_PWM_CHANNEL : MOTOR_RIGHT_PWM_CHANNEL;
  int motorDirection = speed < 0;

  // turn motor off
  if (speed == 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }

  // turn motor forwards
  else if (motorDirection == MOTOR_FORWARDS) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  // turn motor backwards
  else if (motorDirection == MOTOR_BACKWARDS) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }

  // set motor speed
  int motorSpeed = map(abs(speed), 0, 99, 0, 255);
  ledcWrite(motorPwmChannel, motorSpeed);

  Serial.printf("(side: %d; speed: %d%%) => ", side, speed);
  Serial.printf("pwm: %d; ", motorSpeed);
  Serial.printf("direction: %d; ", motorDirection);
  Serial.printf("pin1: %d; ", motorPin1);
  Serial.printf("pin2: %d; ", motorPin2);
  Serial.printf("pwmChannel: %d\n", motorPwmChannel);
}

int trimInput(char* buf, int len) {
  for (size_t i = 0; i < len; i++) {
    if (buf[i] == '\n' || buf[i] == '\r') {
      buf[i] = '\0';
      return i;
    }
  }

  return len;
}

#define SPEED_READ_ERROR -12345

int readSpeed(char* buf) {
  int speed = 0;
  char direction = buf[1];

  if (direction != '+' && direction != '-') {
    return SPEED_READ_ERROR;
  }

  int speed1 = buf[2] - '0';
  int speed2 = buf[3] - '0';

  if (speed1 < 0 || speed1 > 9 || speed2 < 0 || speed2 > 9) {
    return SPEED_READ_ERROR;
  }

  speed = speed1 * 10 + speed2;

  if (direction == '-') {
    return speed * -1;
  }

  return speed;
}

void handleInput(WiFiClient client) {
  // check if data available
  if (client.available() <= 0) {
    return;
  }

  int len = 32;
  char buf[len];

  len = client.readBytesUntil('\n', buf, len);
  len = trimInput(buf, len);

  // stop
  if (len == 1 && buf[0] == '!') {
    setMotor(MOTOR_LEFT, 0);
    setMotor(MOTOR_RIGHT, 0);
    client.println("Stopped motors!");
    return;
  }

  // left / right motor
  if (len == 4 && (buf[0] == 'l' || buf[0] == 'r')) {
    int motor = buf[0] == 'l' ? MOTOR_LEFT : MOTOR_RIGHT;
    int speed = readSpeed(buf);

    if (speed == SPEED_READ_ERROR) {
      client.printf("Please use a proper speed value! exp: %c+36 for +36%% speed\n", buf[0]);
      return;
    }

    setMotor(motor, speed);
    client.printf("%s motor: %d%%\n", motor == MOTOR_LEFT ? "left" : "right", speed);
    return;
  }

  // both motors
  if (len == 4 && buf[0] == 'b') {
    int speed = readSpeed(buf);

    if (speed == SPEED_READ_ERROR) {
      client.printf("Please use a proper speed value! exp: %c+36 for +36%% speed\n", buf[0]);
      return;
    }

    setMotor(MOTOR_LEFT, speed);
    setMotor(MOTOR_RIGHT, speed);
    client.printf("both motors: %d%%\n", speed);
    return;
  }

  // disconnect
  if (len == 1 && buf[0] == 'q') {
    client.println("Bye bye!");
    client.stop(); // disconnect client
    return;
  }

  client.println("Syntax error!");
  client.stop(); // disconnect client
}

void cmdServerTask(void* pvParameters) {
  cmdServer.begin();

  while(true) {
    WiFiClient client = cmdServer.available();
    if (client) {
      Serial.println("New client connected.");
      client.println("Hello :wave:");

      while (client.connected()) {
        handleInput(client);
        vTaskDelay(10);
      }

      // close the connection:
      client.stop();
      Serial.println("Client disconnected.");
    }
  }
}

void setupCmdServer() {
  xTaskCreatePinnedToCore(
    cmdServerTask,
    "cmdServer",
    4 * 1024,
    NULL, //(void*) handler,
    2,
    &tCmdServer,
    APP_CPU);
}
#endif

// ==== SETUP method ==================================================================
void setup() {
  // sets the pins
  pinMode(MOTOR_LEFT_PIN_1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_2, OUTPUT);
  ledcSetup(MOTOR_LEFT_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PIN_PWM, MOTOR_LEFT_PWM_CHANNEL);

  pinMode(MOTOR_RIGHT_PIN_1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_2, OUTPUT);
  ledcSetup(MOTOR_RIGHT_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_RIGHT_PIN_PWM, MOTOR_RIGHT_PWM_CHANNEL);

  // Setup Serial connection:
  Serial.begin(115200);
  delay(1000); // wait for a second to let Serial connect
  Serial.println("Starting sauger v0.1");

  #ifdef ENABLE_CAM
    setupCam();
  #endif

  //  Configure and connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println("");
  Serial.println(F("WiFi connected"));
  Serial.println("");

  #ifdef ENABLE_CAM
    setupCamServer();
  #endif

  #ifdef ENABLE_OTA
    setupOTA();
  #endif

  #ifdef ENABLE_CMD
    setupCmdServer();
  #endif
}

void loop() {
  #ifdef ENABLE_OTA
    ArduinoOTA.handle();
  #endif

  delay(1);
}

