
#define NON_PID 0

// Debugging
#define LOG_SERVER 1
// THETA
#define USE_THETA 1
#define USE_BNO055 1

#define BNO055_CALIBRATED_BIT (1 << 0)

/*-------------------------------------------*/
#define USE_FUZZY_PID 1
#define CALIBRATION_KINEMATICS 0
/*-------------------------------------------*/

// Robot Information
#define ID_ROBOT "robot1"
#define WHEEL_RADIUS 0.03   // Bán kính bánh xe (m)
#define ROBOT_RADIUS 0.1543 // Khoảng cách từ tâm robot đến bánh xe (m)

#define IDEAL 1

#if IDEAL == 1

#define L1 0.1543
#define L2 0.1543 // Khoảng cách giữa các bánh xe (m)
#define L3 0.1543 // Khoảng cách giữa các bánh xe (m)

#define r1 0.03
#define r2 0.03
#define r3 0.03

#else

#define L1 0.1555
#define L2 0.1527 // Khoảng cách giữa các bánh xe (m)
#define L3 0.1546 // Khoảng cách giữa các bánh xe (m)

#define r1 0.0298
#define r2 0.0281
#define r3 0.0290

#endif

#define WEIGHT 2.0 // Trọng lượng robot (kg)

// #define WIFI_SSID "CEEC_Tenda"
// #define WIFI_PASS "1denmuoi1"
// #define SERVER_IP "192.168.2.126"

#define WIFI_SSID "S20 FE"
#define WIFI_PASS "25102004"
#define SERVER_IP "192.168.210.85"

// #define WIFI_SSID "A10.14"
// #define WIFI_PASS "MMNT2004"
// #define SERVER_IP "192.168.1.241"

// #define WIFI_SSID "UIT_CAR_RACING_2023"
// #define WIFI_PASS "sinhvien_CEEC"
// #define SERVER_IP "192.168.7.126"

// #define WIFI_SSID "KTMT - SinhVien"
// #define WIFI_PASS "sinhvien"
// #define SERVER_IP "192.168.155.1"