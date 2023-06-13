/*
 * Drone Controller
 * github.com/estods3/Drone
 * @author: estods3
 * Description:
 * This SW is the main controller for a NodeMCU-based quadcopter.
 *
 */
#include <cmath>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <tf/transform_broadcaster.h>
// Accelerometer Libraries if MPU6050 is used
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

// ESC CALIBRATION
//-----------------
//these values copied from the BlHeli configuration
#define ESC_MIN_THROTTLE		        1040
#define ESC_MAX_THROTTLE		        1960
#define ESC_REVERSE_THROTTLE	        100 // NOT USED
#define ESC_ARM_SIGNAL			        1000
#define ESC_ARM_TIME			        2000
#define ESC_DEADZONE_RANGE              100
#define ESC_FLUTTER_RANGE               10

// PINOUT
//-------
int gpio_battery_level = A0; //TODO - before using this pin, check voltage divider for voltage in range 0-1v and check current at both GND and at Vout of voltage divider. check that it is below current level for NodeMCU.
int pwm_motor_FL = D5;
int pwm_motor_FR = D6;
int pwm_motor_RL = D8;
int pwm_motor_RR = D0; //GPIO16
int ESC_FL_CURRENT_INPUT_VALUE = ESC_MIN_THROTTLE;
//NOTE: D0 is the same as the BUILT IN LED pin. The built in LED will pulse opposite D0 PWM Signal.
//NOTE: D0 outputs a HIGH signal at Boot which will turn on the Motor for RR. Need to make sure VCC for motors is disconnected when booting on MCU. 
//Potential Solution: Add RST button circuitry to PCB which triggers reset of NodeMCU + temporarily turns off motor drivers.
int i2c_scl = D1; //USED FOR I2C, comes from hardware variant definition for NodeMCU
int i2c_sda = D2; //USED FOR I2C, comes from hardware variant definition for NodeMCU
int gpio_led_heartbeat = D3;
int gpio_led_wifi_rx = D4;
int gpio_led_wifi_conn = D7; // GPIO13
//NOTE: D4 is the same as the BUILT IN LED pin. The built in LED will pulse opposite D4 HIGH/LOW Status.

// Calibration
//------------
// WIFI Connection
const char* ssid     = "";
const char* password = "";
#define TESTING_MODE  true
#define ros_is_used true
int wifi_rx_flash_duration_ms = 10;
double rcScaleValue = 1;
Servo esc;

// GUI Server
//------------
WiFiServer gui_server(80);

// ROS Interface
//--------------
#ifdef ros_is_used
// Set the rosserial socket server IP address (Same as roscore)
IPAddress ros_socket_server(192, 168, 1, 200);
// Set the rosserial socket server port (default is 11411)
const uint16_t ros_socket_serverPort = 11411;

ros::NodeHandle drone_rosnode_handle;
std_msgs::Bool bool_msg;
ros::Publisher mcu_alive("MCUAlive", &bool_msg);
ros::Publisher mcu_wifi("MCUWifi", &bool_msg);

// Make a command publisher
std_msgs::Char char_msg;
ros::Publisher command("Command", &char_msg);
String gui_command = "Hold";  //default command 

sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu/data_raw", &imu_msg);

//TODO - need ROS messages and topics for commanded action (from GUI) and commanded speed to each ESC from Flight Controller (ints)
//TODO - need ROS message for battery life (float)

//sensor_msgs::Range range_msg;
//ros::Publisher pub_ran( "/range/test", &range_msg);

//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;
//char base_link[] = "/base_link";
//char odom[] = "/odom";
#endif

// IMU
//----------
Adafruit_MPU6050 mpu;

// FUNCTION: Connect to Wifi
// Description: initialize connection to specified wifi network
void connect_to_wifi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  check_wifi_status();
}

// FUNCTION: Check Wifi Status
// Description: Check Wifi status, update Wifi LED accordingly.
bool check_wifi_status() {
  bool wifi_status_connected = WiFi.status() == WL_CONNECTED;
  if (wifi_status_connected) {
    digitalWrite(gpio_led_wifi_conn, HIGH);
  } else {
    digitalWrite(gpio_led_wifi_conn, LOW);
  }
  return wifi_status_connected;
}

// FUNCTION: Initialize GUI Server
// Description: Create a GUI server with an http webpage for User to enter commands
void initialize_gui_server() {
  gui_server.begin();
  Serial.println("GUI Server started");
  Serial.print("Use this URL to connect to GUI: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

// FUNCTION: Get Commands from GUI Client
// Description: Generate HTML Webpage. Get commands from GUI Client. 
char get_commands_from_gui_client() {
    // Check if a client has connected
  WiFiClient gui_client = gui_server.available();
  if (!gui_client) {
    return 'H';
  }

  // Wait until the client sends some data
  Serial.println("new client");
  while (!gui_client.available()) {
    delay(1);
  }

  // Read the first line of the request
  String request = gui_client.readStringUntil('\r');
  char c_request = 'H';
  Serial.println(request);
  gui_client.flush();

  // interpret Request
  if (request.indexOf("/CMD=UP") != -1) {
    c_request = 'U';
  }
  if (request.indexOf("/CMD=DOWN") != -1) {
    c_request = 'D';
  }
  if (request.indexOf("/CMD=LEFT") != -1) {
    c_request = 'L';
  }
  if (request.indexOf("/CMD=RIGHT") != -1) {
    c_request = 'R';
  }
  if (request.indexOf("/CMD=FORWARD") != -1) {
    c_request = 'F';
  }
  if (request.indexOf("/CMD=BACKWARD") != -1) {
    c_request = 'B';
  }
  if (request.indexOf("/CMD=FL_DEC") != -1) {
    c_request = 'Z';
  }
  if (request.indexOf("/CMD=FL_INC") != -1) {
    c_request = 'X';
  }
  if (request.indexOf("/CMD=FL_INIT") != -1) {
    c_request = 'I';
  }

  // if any command recieved, digital write wifi rx pin
  digitalWrite(gpio_led_wifi_rx, HIGH);
  delay(wifi_rx_flash_duration_ms);
  digitalWrite(gpio_led_wifi_rx, LOW);

  // Return the response
  gui_client.println("HTTP/1.1 200 OK");
  gui_client.println("Content-Type: text/html");
  gui_client.println("");  //  do not forget this one
  gui_client.println("<!DOCTYPE HTML>");
  gui_client.println("<html>");

  gui_client.print("GUI Command: ");
  gui_client.print(c_request);

  gui_client.println("<br><br>");
  gui_client.println("<a href=\"/CMD=FL_INIT\"\"><button style=\"height:100px;width:400px\">ESC1 INIT</button></a><br />");
  gui_client.println("<a href=\"/CMD=UP\"\"><button style=\"height:100px;width:400px\">UP</button></a>");
  gui_client.println("<a href=\"/CMD=DOWN\"\"><button style=\"height:100px;width:400px\">DOWN</button></a><br />");
  gui_client.println("<a href=\"/CMD=LEFT\"\"><button style=\"height:100px;width:400px\">LEFT</button></a>");
  gui_client.println("<a href=\"/CMD=RIGHT\"\"><button style=\"height:100px;width:400px\">RIGHT</button></a><br />");
  gui_client.println("<a href=\"/CMD=FORWARD\"\"><button style=\"height:100px;width:400px\">FORWARD</button></a>");
  gui_client.println("<a href=\"/CMD=BACKWARD\"\"><button style=\"height:100px;width:400px\">BACKWARD</button></a><br />");
  if(TESTING_MODE){
    gui_client.println("<a href=\"/CMD=FL_DEC\"\"><button style=\"height:100px;width:400px\">TEST MOTOR THROTTLE DOWN</button></a><br />");
    gui_client.println("<a href=\"/CMD=FL_INC\"\"><button style=\"height:100px;width:400px\">TEST MOTOR THROTTLE UP</button></a><br />");
  }
  gui_client.println("</html>");

  delay(1);
  Serial.println("GUI Client disconnected");
  Serial.println("");

  return c_request;
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
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setInterruptPinLatch(true);
  delay(100);
}

struct imu_data{
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t temp;  
};

// FUNCTION: Read IMU data
// Description: Read IMU data. Log to serial for serial plotter if log= true
struct imu_data read_imu_data(bool log){
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

double DetermineRCScale()
{
    double range = ESC_MAX_THROTTLE - ESC_MIN_THROTTLE;
    Serial.println(range / 1000);
    return (range / 1000); //1000 is the normal range for PWM signals (1000us to 2000us)
}

int ScaleRCInput(int rcValue)
{
    return ((rcValue - 1000) * rcScaleValue) + ESC_MIN_THROTTLE;
}

void InitESC()
{
    esc.writeMicroseconds(ESC_ARM_SIGNAL);
    unsigned long now = millis();
    while (millis() < now + ESC_ARM_TIME)
    {
        esc.writeMicroseconds(ESC_MAX_THROTTLE);      
    }
    esc.writeMicroseconds(ESC_ARM_SIGNAL);
}

bool isDeadzone(int speed)
{
    if (speed >= (ESC_REVERSE_THROTTLE - ESC_DEADZONE_RANGE) && speed <= (ESC_REVERSE_THROTTLE + ESC_DEADZONE_RANGE))
    {
        return true;
    }
    return false;
}

void WriteSpeed(int speed)
{
    if (isDeadzone(speed)) speed == ESC_REVERSE_THROTTLE;

    int curSpeed = esc.readMicroseconds();

    if (curSpeed >= (speed - ESC_FLUTTER_RANGE) && curSpeed <= (speed + ESC_FLUTTER_RANGE))
    {
        return;
    }

    esc.writeMicroseconds(speed);
}


// CLASS: Heartbeat LED
// Description: Class to create a heartbeat LED that will not block CPU in order to update.
class heartbeatLED {
  // Class Member Variables
  // These are initialized at startup
  int ledPin;    // the number of the LED pin
  long OnTime;   // milliseconds of on-time
  long OffTime;  // milliseconds of off-time

  // These maintain the current state
  int ledState;                  // ledState used to set the LED
  unsigned long previousMillis;  // will store last time LED was updated

  // Constructor - creates a heartbeatLED
  // and initializes the member variables and state
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

/* SETUP
*
*
*/
void setup() {
  // Initialize serial debug interface @115200 Baud
  Serial.begin(115200);
  Serial.println();

  // Initializing LED Pins + Heartbeat LED
  pinMode(gpio_led_wifi_conn, OUTPUT);
  pinMode(gpio_led_wifi_rx, OUTPUT);
  digitalWrite(gpio_led_wifi_conn, LOW);
  digitalWrite(gpio_led_wifi_rx, LOW);
  heartbeat.Update();

  // Initialize ESCs
  esc.attach(pwm_motor_FL, ESC_MIN_THROTTLE, ESC_MAX_THROTTLE);
  rcScaleValue = DetermineRCScale();

  // Connect the ESP8266 the the wifi AP
  connect_to_wifi();

  // Initialize GUI Server
  initialize_gui_server();

  // Initialize IMU
  initialize_imu();

  // Initialize ROS Serial Socket Server Connection
  // Set the connection to rosserial socket server and start advertising messages
  if(ros_is_used){  
    initialize_ros_serial_socket_server_connection(ros_socket_server, ros_socket_serverPort);
    drone_rosnode_handle.advertise(mcu_alive);
    drone_rosnode_handle.advertise(mcu_wifi);
    drone_rosnode_handle.advertise(command);
    drone_rosnode_handle.advertise(pub_imu);
  }
}


/* Main Loop
*
*
*/
void loop() {
  
  /* Update Variables:
  *  -----------------
  *  Collect Data from various sources such as: heartbeat, wifi status, IMU, GUI
  */
  bool heartbeat_signal = heartbeat.Update();
  bool wifi_connected = check_wifi_status();
  //check_battery_life(); - analog input from battery monitoring circuitry 0-1v = 0%->100%
  struct imu_data imu_data_set = read_imu_data(false);
  char gui_command = get_commands_from_gui_client();

  /* Update ROS Network:
  *  -----------------
  *  Publish Data from various sources such as: heartbeat, wifi status, IMU, GUI
  *  to ROS Network
  */  
  if(ros_is_used) {
    if(drone_rosnode_handle.connected()) {
      bool_msg.data = heartbeat_signal;
      mcu_alive.publish(&bool_msg);

      bool_msg.data = wifi_connected;
      mcu_wifi.publish(&bool_msg);

      char_msg.data = gui_command;
      command.publish(&char_msg);

      double time_in_seconds = drone_rosnode_handle.now().toSec();
      //Serial.print(time_in_seconds);
      //if(abs(time_in_seconds - (int)time_in_seconds) < 0.01){
      Serial.print("IMU Update to ROS: ");
      Serial.print(time_in_seconds);
      Serial.println();
      imu_msg.header.frame_id = "/imu_link";
      imu_msg.header.stamp = drone_rosnode_handle.now();
      imu_msg.angular_velocity.x = imu_data_set.g.gyro.x;
      imu_msg.angular_velocity.y = imu_data_set.g.gyro.y;
      imu_msg.angular_velocity.z = imu_data_set.g.gyro.z;
      imu_msg.linear_acceleration.x = imu_data_set.a.acceleration.x;
      imu_msg.linear_acceleration.y = imu_data_set.a.acceleration.y;
      imu_msg.linear_acceleration.z = imu_data_set.a.acceleration.z;
      pub_imu.publish(&imu_msg);
      //}
    }
    drone_rosnode_handle.spinOnce();
  }
  
  /* Flight Controller:
  *  ------------------
  *  Perform Flight Control using inputs  
  *
  */
  //flight_controller();
  if(gui_command == 'I') InitESC();
  
  if(TESTING_MODE){
    if(gui_command == 'Z') ESC_FL_CURRENT_INPUT_VALUE = ESC_FL_CURRENT_INPUT_VALUE - 5;
    if(gui_command == 'X') ESC_FL_CURRENT_INPUT_VALUE = ESC_FL_CURRENT_INPUT_VALUE + 5;
    Serial.print("Recieved: " );
    Serial.println(ESC_FL_CURRENT_INPUT_VALUE);
    int valueSpeed = ScaleRCInput(ESC_FL_CURRENT_INPUT_VALUE);
    WriteSpeed(valueSpeed);
    delay(20);
  }


  // Loop
  delay(1);
}
