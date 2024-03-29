/*
 * Drone Controller
 * github.com/estods3/Drone
 * @author: estods3
 * Description:
 * This SW is the main controller for a NodeMCU-based quadcopter.
  *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 */
#include <cmath>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

// OPERATING MODES
//----------------
#define USE_ROS                     false        // USE_ROS = true, will transmit all data using ROS Serial to a ROS network for remote viewing and logging
#define ACCESS_POINT_WIFI           true       // ACCESS_POINT_WIFI = true, will create a wireless access point called "quadcopter" and allow the user to sign onto that network for RC.

//BMS CALIBRATION
//---------------
// Connect full battery to PCB and check displayed battery level on GUI. Adjust number below until GUI reads 100%.
#define BMS_MAX_BATTERY_BITS		    950         // This number reflects the bit-width of the ADC based on the voltage divider resistors used on PCB. Variation in resistor values is possible.

// ESC CALIBRATION
//-----------------
// These values copied from the BlHeli configuration.
#define ESC_MIN_THROTTLE		        1040
#define ESC_MAX_THROTTLE		        1960
#define ESC_REVERSE_THROTTLE	      100         // NOT USED
#define ESC_ARM_SIGNAL			        1000
#define ESC_ARM_TIME			          2000
#define ESC_DEADZONE_RANGE          100         // NOT USED
#define ESC_FLUTTER_RANGE           10

// IMU CALIBRATION
//----------------
// These values copied from IMU Calibration script.
//-0.23, -0.53, -2.29, 0.03,	0.02, -0.01
#define IMU_CAL_OFFSET1             -0.23
#define IMU_CAL_OFFSET2             -0.53
#define IMU_CAL_OFFSET3             -2.29
#define IMU_CAL_OFFSET4             0.03
#define IMU_CAL_OFFSET5             0.02
#define IMU_CAL_OFFSET6             -0.01

// PINOUT
//-------
// These values are set based on the PCB used.
int gpio_battery_level = A0;
int gpio_esc_pwm_FL = D5;
int gpio_esc_pwm_FR = D6;
int gpio_esc_pwm_RL = D8;
int gpio_esc_pwm_RR = D0; //GPIO16
//NOTE: D0 is the same as the BUILT IN LED pin. The built in LED will pulse opposite D0 PWM Signal.
//NOTE: D0 outputs a HIGH signal at Boot which will turn on the Motor for RR. Need to make sure VCC for motors is disconnected when booting on MCU. 
//Potential Solution: Add RST button circuitry to PCB which triggers reset of NodeMCU + temporarily turns off motor drivers.
int i2c_scl = D1; //USED FOR I2C, comes from hardware variant definition for NodeMCU
int i2c_sda = D2; //USED FOR I2C, comes from hardware variant definition for NodeMCU
int gpio_led_heartbeat = D3;
int gpio_led_wifi_rx = D4;
int gpio_led_wifi_conn = D7; // GPIO13
//NOTE: D4 is the same as the BUILT IN LED pin. The built in LED will pulse opposite D4 HIGH/LOW Status.

// WiFI CALIBRATION
//-----------------
const char* ssid     = "";
const char* password = "";
int wifi_rx_flash_duration_ms = 4;

// Index locations for data and instructions
# define YAW         0
# define PITCH       1
# define ROLL        2
# define THROTTLE    3

# define X           0     // X axis
# define Y           1     // Y axis
# define Z           2     // Z axis

// Status of Drone
# define STOPPED     0
# define ESC_TESTING 1
# define STARTED     2

// ---------------- Control variables ---------------------------------------
float instruction[4] = {0,0,0,0};         // Received flight instructions in order: [Yaw (cardinal direction), Pitch (x), Roll (y), Throttle]

/*
 * Real YPR measurements calculated from combination of accelerometer and gyro data in order: [yaw, pitch, roll]
 *  - Left side up implies positive roll
 *  - Nose up implies positive pitch
 *  - Nose right implies a positive yaw
 */
float measures[3] = {0,0,0};              // Real YPR measurements calculated from combination of accelerometer and gyro data in order: [yaw, pitch, roll]
bool started = false;                     // Flag to signify if the system is started

// ---------------- PID Variables ---------------------------------------
float errors[3];                          // Measured errors compared to target positions in order: [yaw, pitch, roll]
float error_sum[3] = {0,0,0};             // Error sums used for integral component of PID in order: [yaw, pitch, roll]
float previous_error[3] = {0,0,0};        // Previous errors used for derivative component of PID in order: [yaw, pitch, roll]

float Kp[3]        = {0, 0, 0};           // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3]        = {0.00, 0.00, 0.00};  // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3]        = {0, 0, 0};           // D coefficients in that order : Yaw, Pitch, Roll

int status = STOPPED;
// ---------------------------------------------------------------------------

// for calculating running frequency of code
float i {0};
float start_seconds {0};

// GUI Server
//-----------
//WiFiServer gui_server(80);
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
String batterylife = "100";
String sliderValue1 = "0";
String sliderValue2 = "0";
String sliderValue3 = "0";
String sliderValue4 = String(ESC_MIN_THROTTLE);
String sliderValue5 = String(ESC_MIN_THROTTLE);
String sliderValue6 = String(ESC_MIN_THROTTLE);
String sliderValue7 = String(ESC_MIN_THROTTLE);
int armvalue = 0;
int stopvalue = 0;

const char* PARAM_INPUT = "value";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drone RC Controller</title>
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 1.5rem;}
    p {font-size: 1rem;}
    body {margin:0px auto; padding-bottom: 25px;}
    .slider { -webkit-appearance: none; margin: 1px; width: 200px; height: 25px; background: #32a852;
      outline: none; -webkit-transition: .2s; transition: opacity .2s;}
    .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background: #000000; cursor: pointer;}
    .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; }
    .button { width: 100px; height: 50px; background: #32a852; cursor: pointer; }
    .grid-container {
      display: grid;
      grid-template-columns: auto auto;
      padding: 1px;
    }
    .grid-item {
      padding: 10px;
      font-size: 20px;
      text-align: center;
    }
    .PID_grid-container {
      display: grid;
      grid-template-columns: auto auto auto auto;
      background-color: #32a852;
      padding: 10px;
    }
    .PID_grid-item {
      background-color: rgba(255, 255, 255, 0.8);
      border: 1px solid rgba(0, 0, 0, 0.8);
      padding: 5px;
      font-size: 20px;
      text-align: center;
    }
    .collapsible {
      background-color: #32a852;
    }
  </style>
</head>
<body>
  <h2>Drone RC Controller</h2>
  <p>Battery Life:<span id="batterylife">%BATTERYLIFE%</span></p>
  <div class="grid-container">
  <div class="grid-item"><p>Lift: <span id="textSliderValue1">%SLIDER1VALUE%</span></p>
  <p><input type="range" oninput="updateSlider1(this)" id="elevationSlider" min="0" max="%ESC_MAX_THROTTLE%" value="%SLIDER1VALUE%" step="1" class="slider"></p></div>
  <div class="grid-item"><p>Forward/Backward: <span id="textSliderValue3">%SLIDER3VALUE%</span></p>
  <p><input type="range" oninput="updateSlider3(this)" id="forwardSlider" min="-5" max="5" value="%SLIDER3VALUE%" step="1" class="slider"></p></div>
  <div class="grid-item"><p>Yaw: <span id="textSliderValue2">%SLIDER2VALUE%</span></p>
  <p><input type="range" oninput="updateSlider2(this)" id="yawSlider" min="-180" max="180" value="%SLIDER2VALUE%" step="1" class="slider"></p></div>
  <div class="grid-item"><input type="button" id="stop_button" onclick="updateStopButton(this)" value="STOP" class="button"><input type="button"  id="arm_button" onclick="updateArmButton(this)" value="ARM" class="button"></div>  
  </div>
  <button type="button" class="collapsible">Test ESCs</button>
  <div class="content">
    <div class="grid-container">
    <div class="grid-item"><p>ESC 1: <span id="textSliderValue4">%SLIDER4VALUE%</span></p>
    <p><input type="range" oninput="updateSlider4(this)" id="esc1Slider" min="%ESC_MIN_THROTTLE%" max="%ESC_MAX_THROTTLE%" value="%SLIDER4VALUE%" step="1" class="slider"></p></div>
    <div class="grid-item"><p>ESC 2: <span id="textSliderValue5">%SLIDER5VALUE%</span></p>
    <p><input type="range" oninput="updateSlider5(this)" id="esc2Slider" min="%ESC_MIN_THROTTLE%" max="%ESC_MAX_THROTTLE%" value="%SLIDER5VALUE%" step="1" class="slider"></p></div>
    <div class="grid-item"><p>ESC 3: <span id="textSliderValue6">%SLIDER6VALUE%</span></p>
    <p><input type="range" oninput="updateSlider6(this)" id="esc3Slider" min="%ESC_MIN_THROTTLE%" max="%ESC_MAX_THROTTLE%" value="%SLIDER6VALUE%" step="1" class="slider"></p></div>
    <div class="grid-item"><p>ESC 4: <span id="textSliderValue7">%SLIDER7VALUE%</span></p>
    <p><input type="range" oninput="updateSlider7(this)" id="esc4Slider" min="%ESC_MIN_THROTTLE%" max="%ESC_MAX_THROTTLE%" value="%SLIDER7VALUE%" step="1" class="slider"></p></div>
    </div>
  </div>
  <button type="button" class="collapsible">Calibrate PIDs</button>  
  <div class="content">
    <iframe name="dummyframe" id="dummyframe" style="display: none;"></iframe>
    <form action="/PID" target="dummyframe">
      <div class="PID_grid-container">
        <div class="PID_grid-item"></div>
        <div class="PID_grid-item">Yaw</div>
        <div class="PID_grid-item">Pitch</div>
        <div class="PID_grid-item">Roll</div>
        <div class="PID_grid-item">Kp</div>
        <div class="PID_grid-item"><input type="text" id="Kpy" name="Kpy" value="%KPY_VALUE%"><br></div>
        <div class="PID_grid-item"><input type="text" id="Kpp" name="Kpp" value="%KPP_VALUE%"><br></div>
        <div class="PID_grid-item"><input type="text" id="Kpr" name="Kpr" value="%KPR_VALUE%"><br></div>
        <div class="PID_grid-item">Ki</div>
        <div class="PID_grid-item"><input type="text" id="Kiy" name="Kiy" value="%KIY_VALUE%"><br></div>
        <div class="PID_grid-item"><input type="text" id="Kip" name="Kip" value="%KIP_VALUE%"><br></div>
        <div class="PID_grid-item"><input type="text" id="Kir" name="Kir" value="%KIR_VALUE%"><br></div>
        <div class="PID_grid-item">Kd</div>
        <div class="PID_grid-item"><input type="text" id="Kdy" name="Kdy" value="%KDY_VALUE%"><br></div>
        <div class="PID_grid-item"><input type="text" id="Kdp" name="Kdp" value="%KDP_VALUE%"><br></div>
        <div class="PID_grid-item"><input type="text" id="Kdr" name="Kdr" value="%KDR_VALUE%"><br></div>
      </div>
      <input type="submit" value="Submit">
    </form>
  </div>
<script>
var coll = document.getElementsByClassName("collapsible");
var i;

for (i = 0; i < coll.length; i++) {
  coll[i].addEventListener("click", function() {
    this.classList.toggle("active");
    var content = this.nextElementSibling;
    if (content.style.display === "block") {
      content.style.display = "none";
    } else {
      content.style.display = "block";
    }
  });
}
function updateStopButton(element) {
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/stop?value="+1, true);
  xhr1.send();
}
function updateArmButton(element) {
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/arm?value="+1, true);
  xhr1.send();
}
function updateSlider1(element) {
  var sliderValue = document.getElementById("elevationSlider").value;
  document.getElementById("textSliderValue1").innerHTML = sliderValue;
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/slider1?value="+sliderValue, true);
  xhr1.send();
}
function updateSlider2(element) {
  var sliderValue = document.getElementById("yawSlider").value;
  document.getElementById("textSliderValue2").innerHTML = sliderValue;
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/slider2?value="+sliderValue, true);
  xhr1.send();
}
function updateSlider3(element) {
  var sliderValue = document.getElementById("forwardSlider").value;
  document.getElementById("textSliderValue3").innerHTML = sliderValue;
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/slider3?value="+sliderValue, true);
  xhr1.send();
}
function updateSlider4(element) {
  var sliderValue = document.getElementById("esc1Slider").value;
  document.getElementById("textSliderValue4").innerHTML = sliderValue;
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/slider4?value="+sliderValue, true);
  xhr1.send();
}
function updateSlider5(element) {
  var sliderValue = document.getElementById("esc2Slider").value;
  document.getElementById("textSliderValue5").innerHTML = sliderValue;
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/slider5?value="+sliderValue, true);
  xhr1.send();
}
function updateSlider6(element) {
  var sliderValue = document.getElementById("esc3Slider").value;
  document.getElementById("textSliderValue6").innerHTML = sliderValue;
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/slider6?value="+sliderValue, true);
  xhr1.send();
}
function updateSlider7(element) {
  var sliderValue = document.getElementById("esc4Slider").value;
  document.getElementById("textSliderValue7").innerHTML = sliderValue;
  var xhr1 = new XMLHttpRequest();
  xhr1.open("GET", "/slider7?value="+sliderValue, true);
  xhr1.send();
}
</script>
</body>
</html>
)rawliteral";

// FUNCTION: Processor
// Replaces placeholder with button section in your web page
String processor(const String& var){
  //Serial.println(var);
  if (var == "BATTERYLIFE"){
    return batterylife;
  }
  if (var == "ESC_MIN_THROTTLE"){
    return String(ESC_MIN_THROTTLE);
  }
  if (var == "ESC_MAX_THROTTLE"){
    return String(ESC_MAX_THROTTLE);
  }
  if (var == "SLIDER1VALUE"){
    return sliderValue1;
  }
  if (var == "SLIDER2VALUE"){
    return sliderValue2;
  }
  if (var == "SLIDER3VALUE"){
    return sliderValue3;
  }
  if (var == "SLIDER4VALUE"){
    return sliderValue4;
  }
  if (var == "SLIDER5VALUE"){
    return sliderValue5;
  }
  if (var == "SLIDER6VALUE"){
    return sliderValue6;
  }
  if (var == "SLIDER7VALUE"){
    return sliderValue7;
  }
  if (var == "KPY_VALUE"){
    return String(Kp[YAW]);
  }
  if (var == "KPP_VALUE"){
    return String(Kp[PITCH]); 
  }
  if (var == "KPR_VALUE"){
    return String(Kp[ROLL]); 
  }
  if (var == "KIY_VALUE"){
    return String(Ki[YAW]);
  }
  if (var == "KIP_VALUE"){
    return String(Ki[PITCH]); 
  }
  if (var == "KIR_VALUE"){
    return String(Ki[ROLL]);
  }
  if (var == "KDY_VALUE"){
    return String(Kd[YAW]);
  }
  if (var == "KDP_VALUE"){
    return String(Kd[PITCH]);
  }
  if (var == "KDR_VALUE"){
    return String(Kd[ROLL]);
  }
  return String();
}

// ROS Interface
//--------------
#ifdef USE_ROS
// Set the rosserial socket server IP address (Same as roscore)
IPAddress ros_socket_server(192, 168, 1, 100);
// Set the rosserial socket server port (default is 11411)
const uint16_t ros_socket_serverPort = 11411;

ros::NodeHandle drone_rosnode_handle;
std_msgs::Bool bool_msg;
ros::Publisher ros_pub_mcu_alive("mcu/alive", &bool_msg);
ros::Publisher ros_pub_mcu_wifi("mcu/wifi", &bool_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher ros_pub_imu("imu/data_raw", &imu_msg);

std_msgs::Int16 int_msg;
ros::Publisher ros_pub_batt_life("mcu/batterylife", &int_msg);

ros::Publisher ros_pub_throttle_esc_FL("throttle/esc_FL", &int_msg);
ros::Publisher ros_pub_throttle_esc_FR("throttle/esc_FR", &int_msg);
ros::Publisher ros_pub_throttle_esc_RL("throttle/esc_RL", &int_msg);
ros::Publisher ros_pub_throttle_esc_RR("throttle/esc_RR", &int_msg);

ros::Publisher ros_pub_command_elev("controller/elevation", &int_msg);
ros::Publisher ros_pub_command_yaw("controller/yaw", &int_msg);
ros::Publisher ros_pub_command_forward("controller/forward", &int_msg);

#endif

// IMU
//----
Adafruit_MPU6050 mpu;

// ESCs
//-----
double rcScaleValue = 1;
Servo esc_FL, esc_FR, esc_RL, esc_RR;
int16_t ESC_FL_CURRENT_INPUT_VALUE, ESC_FR_CURRENT_INPUT_VALUE, ESC_RL_CURRENT_INPUT_VALUE, ESC_RR_CURRENT_INPUT_VALUE = ESC_MIN_THROTTLE;

/*___________________________________________________________________________________________ 
* FUNCTIONS Library
* void connect_to_wifi()
* bool check_wifi_status()
* void initialize_gui_server()
* char get_commands_from_gui_client()
* void initialize_imu()
* struct imu_data read_imu_data(bool log)
* void initialize_ros_serial_socket_server_connection(IPAddress server, uint16_t port)
* 
* ___________________________________________________________________________________________
*/

// CLASS: Heartbeat LED
// Description: Class to create a heartbeat LED that will not block CPU in order to update.
class heartbeatLED {
  int ledPin;    // the number of the LED pin
  long OnTime;   // milliseconds of on-time
  long OffTime;  // milliseconds of off-time

  int ledState;                  // ledState used to set the LED
  unsigned long previousMillis;  // will store last time LED was updated

  public:
    heartbeatLED(int pin, long on, long off) {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);

      OnTime = on;
      OffTime = off;

      ledState = LOW;
      previousMillis = 0;
    }

    bool Update() {
      // check to see if it's time to change the state of the LED
      // update LED, return state of LED
      unsigned long currentMillis = millis();

      if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime)) {
        ledState = LOW;                  // Turn it off
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(ledPin, ledState);  // Update the actual LED
      } else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime)) {
        ledState = HIGH;                 // turn it on
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(ledPin, ledState);  // Update the actual LED
      }

      return ledState == HIGH;    
    }
};

// Instantiate Global Variables from Above.
heartbeatLED heartbeat(gpio_led_heartbeat, 100, 400);

// FUNCTION: Connect to Wifi
// Description: initialize connection to specified wifi network
void connect_to_wifi() {
  Serial.print("Connecting to WiFi: ");
  if(ACCESS_POINT_WIFI){
    WiFi.softAP("quadcopter", "qcoptlogin");
  } else{
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  if(ACCESS_POINT_WIFI){
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(WiFi.localIP());
  }
  check_wifi_status();
}

// FUNCTION: Check Wifi Status
// Description: Check Wifi status, update Wifi LED accordingly.
bool check_wifi_status() {
  bool wifi_status_connected = false;
  if(ACCESS_POINT_WIFI) {
    wifi_status_connected = WiFi.softAPgetStationNum() >= 1;
  } else {
    wifi_status_connected = WiFi.status() == WL_CONNECTED;
  }
  digitalWrite(gpio_led_wifi_conn, wifi_status_connected);
  return wifi_status_connected;
}

// FUNCTION: Initialize GUI Server
// Description: Create a GUI server with an http webpage for User to enter commands
void initialize_gui_server() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider1", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue1 = inputMessage;
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println("/slider1 = " + inputMessage);
    request->send(200, "text/plain", "OK");
  });
    // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider2", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue2 = inputMessage;
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println("/slider2 = " + inputMessage);
    request->send(200, "text/plain", "OK");
  });
    // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider3", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue3 = inputMessage;
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println("/slider3 = " + inputMessage);
    request->send(200, "text/plain", "OK");
  });
    // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider4", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue4 = inputMessage;
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println("/slider4 = " + inputMessage);
    request->send(200, "text/plain", "OK");
  });
    // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider5", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue5 = inputMessage;
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println("/slider5 = " + inputMessage);
    request->send(200, "text/plain", "OK");
  });
    // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider6", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue6 = inputMessage;
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println("/slider6 = " + inputMessage);
    request->send(200, "text/plain", "OK");
  });
    // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider7", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue7 = inputMessage;
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println("/slider7 = " + inputMessage);
    request->send(200, "text/plain", "OK");
  });
  // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/stop", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      stopvalue = 1;
      Serial.println("/stop = " + String(stopvalue));
      digitalWrite(gpio_led_wifi_rx, HIGH);
      delay(wifi_rx_flash_duration_ms);
      digitalWrite(gpio_led_wifi_rx, LOW);
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });
  // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/arm", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      armvalue = 1;
      Serial.println("/arm = " + String(armvalue));
      digitalWrite(gpio_led_wifi_rx, HIGH);
      delay(wifi_rx_flash_duration_ms);
      digitalWrite(gpio_led_wifi_rx, LOW);
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });
  server.on("/PID", HTTP_GET, [] (AsyncWebServerRequest *request) {  
    if (request->hasParam("Kpy")) {
      Kp[YAW] = request->getParam("Kpy")->value().toFloat();
    }
    if (request->hasParam("Kpp")) {
      Kp[PITCH] = request->getParam("Kpp")->value().toFloat();
    }
    if (request->hasParam("Kpr")) {
      Kp[ROLL] = request->getParam("Kpr")->value().toFloat();
    }
    if (request->hasParam("Kiy")) {
      Ki[YAW] = request->getParam("Kiy")->value().toFloat();
    }
    if (request->hasParam("Kip")) {
      Ki[PITCH] = request->getParam("Kip")->value().toFloat();
    }
    if (request->hasParam("Kir")) {
      Ki[ROLL] = request->getParam("Kir")->value().toFloat();
    }
    if (request->hasParam("Kdy")) {
      Kd[YAW] = request->getParam("Kdy")->value().toFloat();
    }
    if (request->hasParam("Kdp")) {
      Kd[PITCH] = request->getParam("Kdp")->value().toFloat();
    }
    if (request->hasParam("Kdr")) {
      Kd[ROLL] = request->getParam("Kdr")->value().toFloat();
    }
    Serial.println("/PID = " + String(Kp[YAW]) + "," + String(Kp[PITCH]) + "," + String(Kp[ROLL]));
    Serial.println("/PID = " + String(Ki[YAW]) + "," + String(Ki[PITCH]) + "," + String(Ki[ROLL]));
    Serial.println("/PID = " + String(Kd[YAW]) + "," + String(Kd[PITCH]) + "," + String(Kd[ROLL]));
    request->send(200, "text/plain", "OK");
    digitalWrite(gpio_led_wifi_rx, HIGH);
    delay(wifi_rx_flash_duration_ms);
    digitalWrite(gpio_led_wifi_rx, LOW);
  });
    
  server.begin();
  Serial.println("GUI Server started");
  Serial.print("Use this URL to connect to GUI: ");
  Serial.print("http://");
  if(ACCESS_POINT_WIFI){
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(WiFi.localIP());
  }
  Serial.println("/");
}

// FUNCTION Check Battery Life
// Description: check battery life on pin: gpio_battery_level
int check_battery_life(){
  int v = analogRead(gpio_battery_level);
  int battery_life = map(v, 0, BMS_MAX_BATTERY_BITS, 0, 100);
  //TODO - use an algorithm based on the sum of the 4 ESC throttles to scale the battery life based on bias from the motors being on and inducing more current.
  // ie. adapt the battery life for when drone is at standstill verse in the air.
  return battery_life;
}

// FUNCTION: Initialize IMU
// Description: initialize connection to IMU over I2C
void initialize_imu(){
    // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  mpu.setInterruptPinLatch(true);
  delay(100);
}

// DATA TYPES
struct imu_data {
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t temp;  
};
struct angle {
  float x, y, z = 0;
};
struct Quaterniond {
  float w, x, y, z = 0;
};
struct angle position;
unsigned long lastSampleMicros = 0;

// FUNCTION: Read IMU data
// Description: Read IMU data. Log to serial for serial plotter if log= true
struct imu_data read_imu_data(bool log, float o1, float o2, float o3, float o4, float o5, float o6){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if(log){
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");      
  }

  struct imu_data dataset;
  dataset.a = a;
  dataset.g = g;
  dataset.temp = temp;

  // Apply Calibration
  dataset.a.acceleration.x = dataset.a.acceleration.x + o1;
  dataset.a.acceleration.y = dataset.a.acceleration.y + o2;
  dataset.a.acceleration.z = dataset.a.acceleration.z + o3;
  dataset.g.gyro.x = dataset.g.gyro.x + o4;
  dataset.g.gyro.y = dataset.g.gyro.y + o5;
  dataset.g.gyro.z = dataset.g.gyro.z + o6;

  return dataset;
}

// FUNCTION: Initialize ROS Serial Socket Server Connection
// Description: Initialize a connection to a ROS Socket Server running on a different machine.
void initialize_ros_serial_socket_server_connection(IPAddress server, uint16_t port) {
  drone_rosnode_handle.getHardware()->setConnection(server, port);
  Serial.println("Connected to ROS socket server");
  drone_rosnode_handle.initNode();
  //broadcaster.init(drone_rosnode_handle);
  Serial.print("IP = ");
  Serial.println(drone_rosnode_handle.getHardware()->getLocalIP());
}

double DetermineRCScale() {
    double range = ESC_MAX_THROTTLE - ESC_MIN_THROTTLE;
    Serial.println(range / 1000);
    return (range / 1000); //1000 is the normal range for PWM signals (1000us to 2000us)
}

int ScaleRCInput(int rcValue) {
    return ((rcValue - 1000) * rcScaleValue) + ESC_MIN_THROTTLE;
}

// FUNCTION Init ESC
// Description: Initialize using the arming sequence.
void InitESCs() {
    Serial.println("Arming ESCs...");
    esc_FL.writeMicroseconds(ESC_ARM_SIGNAL);
    esc_FR.writeMicroseconds(ESC_ARM_SIGNAL);
    esc_RL.writeMicroseconds(ESC_ARM_SIGNAL);
    esc_RR.writeMicroseconds(ESC_ARM_SIGNAL);
    unsigned long now = millis();
    while (millis() < now + ESC_ARM_TIME)
    { 
      heartbeat.Update(); //Always update heartbeat LED inside while loops
    }
    Serial.println("Arming Done");
}

bool isDeadzone(int speed) {
    if (speed >= (ESC_REVERSE_THROTTLE - ESC_DEADZONE_RANGE) && speed <= (ESC_REVERSE_THROTTLE + ESC_DEADZONE_RANGE))
    {
        return true;
    }
    return false;
}

void WriteSpeed(Servo &esc_obj, int speed) {
  if (isDeadzone(speed)) speed == ESC_REVERSE_THROTTLE;
  int curSpeed = esc_obj.readMicroseconds();
  if (curSpeed >= (speed - ESC_FLUTTER_RANGE) && curSpeed <= (speed + ESC_FLUTTER_RANGE)) {
      return;
  }
  esc_obj.writeMicroseconds(speed);
}

// FUNCTION: compute_fused_euler_angles_with_complementaryfilter
// Description: compute a fused euler angle for yaw, pitch, and roll in degrees and store in 'measures' global variable.
// Source: https://www.hibit.dev/posts/92/complementary-filter-and-relative-orientation-with-mpu6050
void compute_fused_euler_angles_with_complementaryfilter(struct angle gyroscope, struct angle accelerometer) {
  measures[YAW] = measures[YAW] - degrees(gyroscope.z);   //NOTE: Yaw is being computed only with Gyro. this is prone to drift over time.
  measures[PITCH] = 0.98 * (measures[PITCH] + degrees(gyroscope.x)) + 0.02 * degrees(accelerometer.x);
  measures[ROLL] = 0.98 * (measures[ROLL] + degrees(gyroscope.y)) + 0.02 * degrees(accelerometer.y);
}

// FUNCTION: calculateErrors
// Description: Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the measure and the command.
void calculateErrors() {
    errors[YAW]   = instruction[YAW]   - measures[YAW];
    errors[PITCH] = instruction[PITCH] - measures[PITCH];
    errors[ROLL]  = instruction[ROLL]  - measures[ROLL];
}

// FUNCTION: readController
// Description: Read the remote controller inputs for throttle, forward/backward, and yaw and set 'instructions' global variable accordingly.
void readController(){
    if(sliderValue1.toInt() > 0){
      status = STARTED;
      instruction[THROTTLE] =sliderValue1.toInt();
    }
    if(sliderValue1.toInt() == 0){
      instruction[THROTTLE] = 0;          
      status = STOPPED;
    }
    instruction[YAW] = sliderValue2.toInt();
    if(sliderValue3.toInt() < 0){
      instruction[PITCH] = -2;
    } else if(sliderValue3.toInt() > 0) {
      instruction[PITCH] = 2;
    } else {
      instruction[PITCH] = 0;
    }
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 * @return void
 */
void pidController() {
    float delta_err[3] = {0, 0, 0};          // Error deltas in that order   : Yaw, Pitch, Roll
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;

    // Take no action if there is no throttle to the motors
    if (instruction[THROTTLE] > 0){
      // Calculate sum of errors : Integral coefficients
      error_sum[YAW]   += errors[YAW];
      error_sum[PITCH] += errors[PITCH];
      error_sum[ROLL]  += errors[ROLL];

      // Calculate error delta : Derivative coefficients
      delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
      delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
      delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

      // Save current error as previous_error for next time
      previous_error[YAW]   = errors[YAW];
      previous_error[PITCH] = errors[PITCH];
      previous_error[ROLL]  = errors[ROLL];

      // PID = e.Kp + ∫e.Ki + Δe.Kd
      yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
      pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
      roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

      // Cauculate new target throttle for each motor
      // NOTE: These depend on setup of drone. Verify setup is propper and
      //       consider changing the plus and minuses here if issues happen.
      //       If drone is in propper setup these make sense.
      ESC_FL_CURRENT_INPUT_VALUE = instruction[THROTTLE] + roll_pid + pitch_pid - yaw_pid;
      ESC_FR_CURRENT_INPUT_VALUE = instruction[THROTTLE] - roll_pid + pitch_pid + yaw_pid;
      // back motors are way more powerful
      ESC_RL_CURRENT_INPUT_VALUE = instruction[THROTTLE] + roll_pid - pitch_pid + yaw_pid;
      ESC_RR_CURRENT_INPUT_VALUE = instruction[THROTTLE] - roll_pid - pitch_pid - yaw_pid;
    }
}

void applyMotorSpeeds(){
  int valueSpeed = ScaleRCInput(ESC_FL_CURRENT_INPUT_VALUE);
  WriteSpeed(esc_FL, valueSpeed);
  valueSpeed = ScaleRCInput(ESC_FR_CURRENT_INPUT_VALUE);
  WriteSpeed(esc_FR, valueSpeed);
  valueSpeed = ScaleRCInput(ESC_RL_CURRENT_INPUT_VALUE);
  WriteSpeed(esc_RL, valueSpeed);
  valueSpeed = ScaleRCInput(ESC_RR_CURRENT_INPUT_VALUE);
  WriteSpeed(esc_RR, valueSpeed);
}

void stopMotors(){
    WriteSpeed(esc_FL, 0);
    WriteSpeed(esc_FR, 0);
    WriteSpeed(esc_RL, 0);
    WriteSpeed(esc_RR, 0);
}

struct angle calculateAccelerometerEulerAngles(struct imu_data dataset) {
  angle accelerometer;

  accelerometer.x = atan(dataset.a.acceleration.y / sqrt(sq(dataset.a.acceleration.x) + sq(dataset.a.acceleration.z)));
  accelerometer.y = atan(-1 * dataset.a.acceleration.x / sqrt(sq(dataset.a.acceleration.y) + sq(dataset.a.acceleration.z)));
  accelerometer.z = atan2(accelerometer.y, accelerometer.x);

  return accelerometer;
}

struct angle calculateGyroscopeEulerAngles(struct imu_data dataset, unsigned long sampleMicros) {
  angle gyroscope;

  gyroscope.x = dataset.g.gyro.x * sampleMicros / 1000000;
  gyroscope.y = dataset.g.gyro.y * sampleMicros / 1000000;
  gyroscope.z = dataset.g.gyro.z * sampleMicros / 1000000;

  return gyroscope;
}

//FUNCTION: toQuarternion
//desc: convert yaw, pitch, roll euler angles (in degrees) into a quarternion representation
//Source: https://stackoverflow.com/questions/56576403/from-euler-angles-to-quaternions
struct Quaterniond toQuaternion(double yaw, double pitch, double roll) {
    //Degree to radius:
    yaw = yaw * M_PI / 180;
    pitch = pitch * M_PI / 180;
    roll = roll * M_PI / 180;

    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaterniond q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

/*___________________________________________________________________________________________ 
* Setup
* 1. Initialize serial debug interface
* 2. Setup GPIO Pins
* 3. Initialize ESCs
* 4. Connect to Wifi
* 5. Create GUI Server
* 6. Initialize IMU
* 7. Advertise ROS Messages (if ROS is used)
* ___________________________________________________________________________________________
*/
void setup() {
  // Initialize serial debug interface @115200 Baud
  Serial.begin(115200);
  Serial.println();

  // Initializing LED Pins + Heartbeat LED
  pinMode(gpio_led_wifi_conn, OUTPUT);
  pinMode(gpio_led_wifi_rx, OUTPUT);
  pinMode(gpio_battery_level, INPUT);
  digitalWrite(gpio_led_wifi_conn, LOW);
  digitalWrite(gpio_led_wifi_rx, LOW);
  heartbeat.Update();

  // Initialize ESCs
  esc_FL.attach(gpio_esc_pwm_FL, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
  esc_FR.attach(gpio_esc_pwm_FR, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
  esc_RL.attach(gpio_esc_pwm_RL, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
  esc_RR.attach(gpio_esc_pwm_RR, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
  rcScaleValue = DetermineRCScale();
  InitESCs();

  // Connect the ESP8266 the the wifi AP
  connect_to_wifi();

  // Initialize GUI Server
  initialize_gui_server();

  // Initialize IMU
  initialize_imu();

  // Initialize ROS Serial Socket Server Connection
  // Set the connection to rosserial socket server and start advertising messages
  if(USE_ROS){  
    initialize_ros_serial_socket_server_connection(ros_socket_server, ros_socket_serverPort);
    drone_rosnode_handle.advertise(ros_pub_mcu_alive);
    drone_rosnode_handle.advertise(ros_pub_mcu_wifi);
    drone_rosnode_handle.advertise(ros_pub_batt_life);
    drone_rosnode_handle.advertise(ros_pub_command_elev);
    drone_rosnode_handle.advertise(ros_pub_command_yaw);
    drone_rosnode_handle.advertise(ros_pub_command_forward);
    drone_rosnode_handle.advertise(ros_pub_imu);
    drone_rosnode_handle.advertise(ros_pub_throttle_esc_FL);
    drone_rosnode_handle.advertise(ros_pub_throttle_esc_FR);
    drone_rosnode_handle.advertise(ros_pub_throttle_esc_RL);
    drone_rosnode_handle.advertise(ros_pub_throttle_esc_RR);
  }
}

/*___________________________________________________________________________________________ 
* Main Loop
* 1. Update Status Variables: Heartbeat Signal, Wifi Status, Battery Life, IMU, User Command.
* 2. Flight Controller: Control drone ESCs/Motors using information from Status above.
* 3. Update ROS Network: If ROS is Used, publish messages with data.
* ___________________________________________________________________________________________
*/
void loop() {
  
  /* Update Variables:
  *  -----------------
  *  Collect Data from various sources such as: heartbeat, wifi status, IMU, GUI
  */
  bool heartbeat_signal = heartbeat.Update();
  bool wifi_connected = check_wifi_status();
  int battery_life = check_battery_life();
  struct imu_data imu_data_set = read_imu_data(false, IMU_CAL_OFFSET1, IMU_CAL_OFFSET2, IMU_CAL_OFFSET3, IMU_CAL_OFFSET4,	IMU_CAL_OFFSET5, IMU_CAL_OFFSET6);
  //char gui_command = get_commands_from_gui_client();
  
  unsigned long sampleMicros = (lastSampleMicros > 0) ? micros() - lastSampleMicros : 0;
  lastSampleMicros = micros();
  angle accelerometer = calculateAccelerometerEulerAngles(imu_data_set);
  angle gyroscope = calculateGyroscopeEulerAngles(imu_data_set, sampleMicros);
  
  /* Flight Controller:
  *  ------------------
  */
  i++;
  readController();
  compute_fused_euler_angles_with_complementaryfilter(gyroscope, accelerometer);
  calculateErrors();
  bool ESC_TESTING_MODE_ACTIVATED = sliderValue4.toInt() > ESC_MIN_THROTTLE || sliderValue5.toInt() > ESC_MIN_THROTTLE || sliderValue6.toInt() > ESC_MIN_THROTTLE || sliderValue7.toInt() > ESC_MIN_THROTTLE;
  if(ESC_TESTING_MODE_ACTIVATED){
    status = ESC_TESTING; 
  }
  Serial.println(String(i/(millis()/1000-start_seconds)) + "  Status: " + String(status) + " YPR: " + String(measures[YAW]) + ", " + String(measures[PITCH]) + ", " + String(measures[ROLL]));

  if(armvalue == 1){
    InitESCs();
    armvalue = 0;
  }
  if(status == ESC_TESTING){
    ESC_FL_CURRENT_INPUT_VALUE = sliderValue4.toInt();
    ESC_FR_CURRENT_INPUT_VALUE = sliderValue5.toInt();
    ESC_RL_CURRENT_INPUT_VALUE = sliderValue7.toInt();
    ESC_RR_CURRENT_INPUT_VALUE = sliderValue6.toInt();
    applyMotorSpeeds();
  }
  if(stopvalue == 1 || status == STOPPED){
    ESC_FL_CURRENT_INPUT_VALUE = 1000;
    ESC_FR_CURRENT_INPUT_VALUE = 1000;
    ESC_RL_CURRENT_INPUT_VALUE = 1000;
    ESC_RR_CURRENT_INPUT_VALUE = 1000;
    stopvalue = 0;
    status = STOPPED;
    sliderValue1 = 0; //TODO - when "stop" clicked, start to gradually lower lift slider until it reaches 0., OR IF wifi connection is lost
    sliderValue2 = 0;
    sliderValue3 = 0;
    sliderValue4 = String(ESC_MIN_THROTTLE);
    sliderValue5 = String(ESC_MIN_THROTTLE);
    sliderValue6 = String(ESC_MIN_THROTTLE);
    sliderValue7 = String(ESC_MIN_THROTTLE);
    stopMotors();
  }
  if (status == STARTED) {
    pidController();
    applyMotorSpeeds();
  }
  delay(1); //Control loop delay

  /* Update ROS Network:
  *  -------------------
  *  Publish Data from various sources such as: heartbeat, wifi status, battery life, IMU, GUI
  *  to ROS Network
  */  
  if(USE_ROS) {
    if(drone_rosnode_handle.connected()) {

      // Publish Heartbeat, Wifi, Battery Life, and Remote Controller
      bool_msg.data = heartbeat_signal;
      ros_pub_mcu_alive.publish(&bool_msg);

      bool_msg.data = wifi_connected;
      ros_pub_mcu_wifi.publish(&bool_msg);

      int_msg.data = battery_life;
      ros_pub_batt_life.publish(&int_msg);

      int_msg.data = int16_t(sliderValue1.toInt());
      ros_pub_command_elev.publish(&int_msg);
      int_msg.data = int16_t(sliderValue2.toInt());
      ros_pub_command_yaw.publish(&int_msg);
      int_msg.data = int16_t(sliderValue3.toInt());
      ros_pub_command_forward.publish(&int_msg);

      // Read each ESC Throttle Value and Publish to ROS Network
      int_msg.data = esc_FL.readMicroseconds();
      ros_pub_throttle_esc_FL.publish(&int_msg);
      int_msg.data = esc_FR.readMicroseconds();
      ros_pub_throttle_esc_FR.publish(&int_msg);
      int_msg.data = esc_RL.readMicroseconds();
      ros_pub_throttle_esc_RL.publish(&int_msg);
      int_msg.data = esc_RR.readMicroseconds();
      ros_pub_throttle_esc_RR.publish(&int_msg);

      double time_in_seconds = drone_rosnode_handle.now().toSec();
      imu_msg.header.frame_id = "/imu_link";
      imu_msg.header.stamp = drone_rosnode_handle.now();
      struct Quaterniond q = toQuaternion(measures[YAW], measures[PITCH], measures[ROLL]);
      imu_msg.orientation.x = q.x;
      imu_msg.orientation.y = q.y;
      imu_msg.orientation.z = q.z;
      imu_msg.orientation.w = q.w;
      imu_msg.angular_velocity.x = imu_data_set.g.gyro.x;
      imu_msg.angular_velocity.y = imu_data_set.g.gyro.y;
      imu_msg.angular_velocity.z = imu_data_set.g.gyro.z;
      imu_msg.linear_acceleration.x = imu_data_set.a.acceleration.x;
      imu_msg.linear_acceleration.y = imu_data_set.a.acceleration.y;
      imu_msg.linear_acceleration.z = imu_data_set.a.acceleration.z;
      ros_pub_imu.publish(&imu_msg);          
    }
    drone_rosnode_handle.spinOnce();
  }
}
