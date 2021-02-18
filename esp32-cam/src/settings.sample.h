// Select camera model
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define T_Camera_V17_VERSION
//#define T_Camera_JORNAL_VERSION

// Setting wifi properties
#define WIFI_SSID ""
#define WIFI_PWD ""

// Setting motor properties
#define MOTOR_LEFT_PIN_1 14
#define MOTOR_LEFT_PIN_2 4
#define MOTOR_LEFT_PIN_PWM 2
#define MOTOR_LEFT_PWM_CHANNEL 0

#define MOTOR_RIGHT_PIN_1 13
#define MOTOR_RIGHT_PIN_2 15
#define MOTOR_RIGHT_PIN_PWM 12
#define MOTOR_RIGHT_PWM_CHANNEL 1

#define MOTOR_PWM_FREQ 490 // Hz
#define MOTOR_PWM_RESOLUTION 8 // 8 => 256 different values

// Enable / Disable (by commenting line)
#define ENABLE_CAM
#define ENABLE_CMD
// #define ENABLE_OTA