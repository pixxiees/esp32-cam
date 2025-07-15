/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-cam-photo-microsd-card-timestamp
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <WiFi.h>
#include "time.h"
#include <vector>

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "Adhyasta";
const char* password = "juarasatu";

// REPLACE WITH YOUR TIMEZONE STRING: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
String myTimezone ="<-07>7";

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define FLASH_GPIO_NUM    4
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Stores the camera configuration parameters
camera_config_t config;

// Struct untuk menyimpan gambar sementara
struct PhotoData {
  String filename;
  std::vector<uint8_t> data;
};

std::vector<PhotoData> photoBuffer;

#define TARGET_FPS 10
#define FRAME_INTERVAL_MS (1000 / TARGET_FPS)
unsigned long lastCaptureTime = 0;
int frameCount = 0;
const int MAX_BUFFERED_PHOTOS = 5;

// Initializes the camera
void configInitCamera(){
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA; // Gunakan SVGA untuk keseimbangan ukuran
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

// Connect to wifi
void  initWiFi(){
  WiFi.begin(ssid, password);
  Serial.println("Connecting Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
}

// Function to set timezone
void setTimezone(String timezone){
  Serial.printf("  Setting Timezone to %s\n",timezone.c_str());
  setenv("TZ",timezone.c_str(),1);
  tzset();
}

// Connect to NTP server and adjust timezone
void initTime(String timezone){
  struct tm timeinfo;
  Serial.println("Setting up time");
  configTime(0, 0, "pool.ntp.org");
  if(!getLocalTime(&timeinfo)){
    Serial.println(" Failed to obtain time");
    return;
  }
  Serial.println("Got the time from NTP");
  setTimezone(timezone);
}

// Get the picture filename based on the current time
String getPictureFilename(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return "";
  }
  char timeString[30];
  int millisPart = millis() % 1000;
  strftime(timeString, sizeof(timeString), "%Y-%m-%d_%H-%M-%S", &timeinfo);
  String filename = "/pic_" + String(timeString) + "-" + String(millisPart) + ".jpg";
  return filename;
}

//disable flash
void disableFlash() {
  rtc_gpio_init(GPIO_NUM_4);
  rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(GPIO_NUM_4, 0);
  rtc_gpio_hold_en(GPIO_NUM_4);
}


// Ambil dan simpan ke buffer
void takeBufferedPhoto(){
  camera_fb_t * fb = esp_camera_fb_get();
  if(!fb){
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }

  if (photoBuffer.size() >= MAX_BUFFERED_PHOTOS) {
    Serial.println("Buffer full — flushing via serial...");
    flushBufferToSerial();
  }

  String path = getPictureFilename();
  Serial.printf("Buffered: %s (%d bytes)\n", path.c_str(), fb->len);

  PhotoData photo;
  photo.filename = path;
  photo.data.assign(fb->buf, fb->buf + fb->len);
  photoBuffer.push_back(photo);

  esp_camera_fb_return(fb);
}

// Kirim isi buffer ke serial
void flushBufferToSerial(){
  for (auto &photo : photoBuffer) {
    Serial.println("===== START PHOTO =====");
    Serial.println(photo.filename);
    Serial.write(photo.data.data(), photo.data.size());
    Serial.println("\n===== END PHOTO =====");
  }
  photoBuffer.clear();
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(2000);

  initWiFi();
  initTime(myTimezone);
  Serial.print("Initializing the camera module...");
  configInitCamera();
  disableFlash();
  Serial.println("Ok!");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastCaptureTime >= FRAME_INTERVAL_MS) {
    lastCaptureTime = currentMillis;
    takeBufferedPhoto();
    frameCount++;

    // Tulis ke SD setiap 5 frame atau jika buffer penuh
    if (frameCount % 5 == 0) {
      flushBufferToSD();
    }
  }
}