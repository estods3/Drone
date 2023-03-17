/*
 * Drone Controller
 * github.com/estods3/Drone
 * @author: estods3
 * Description:
 * This SW is the main controller for a NodeMCU-based quadcopter.
 *
 */
#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
// Accelerometer Libraries if MPU6050 is used and native I2C Is Used
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
//#include <Wire.h>


// PINOUT
//-------
//int pwm_motor_FL =
//int pwm_motor_FR =
//intpwm_motor_RL =
//int pwm_motor_RR =
//int i2c_sda =
//int i2c_scl =
//int gpio_led_heartbeat =
int gpio_led_wifi_connected = 13;  // GPIO13
//int gpio_led_wifi_datarx =
//int gpio_led_accel_status =

// WIFI Connection
//----------------
const char* ssid     = "";
const char* password = "";

// GUI Server
//------------
WiFiServer gui_server(80);

// ROS Interface
//--------------
// Set the rosserial socket server IP address (Same as roscore)
IPAddress ros_socket_server(192,168,1,200);
// Set the rosserial socket server port (default is 11411)
const uint16_t ros_socket_serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("Command", &str_msg);
char hold[5] = "Hold";
char gui_command[5] = "Hold";

void connect_to_wifi(){
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  Serial.println(password);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(gpio_led_wifi_connected, HIGH);
}
void update_heartbeat_led(){
  
}
void initialize_gui_server(){
  gui_server.begin();
  Serial.println("GUI Server started");
  Serial.print("Use this URL to connect to GUI: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

void get_commands_from_gui_client(){

}
void read_imu_data(){
  
}

// Initialize ROS Serial Socket Server Connection
// Description: Initialize a connection to a ROS Socket Server running on a different machine.
void initialize_ros_serial_socket_server_connection(){
  nh.getHardware()->setConnection(ros_socket_server, ros_socket_serverPort);
  Serial.println("Connected to ROS socket server");
  nh.initNode();
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
}
void transmit_ros_topic(){
  
}
void setup()
{
  // Initialize serial debug interface
  Serial.begin(115200);
  Serial.println();

  // Connect the ESP8266 the the wifi AP
  connect_to_wifi();

  // Initialize GUI Server
  initialize_gui_server();

  // Initialize ROS Serial Socket Server Connection
  // Set the connection to rosserial socket server
  initialize_ros_serial_socket_server_connection();

  // Start to be polite
  nh.advertise(chatter);
}

void loop()
{

  if (nh.connected()) {
    //Serial.println("Connected");
    // Say hello
    str_msg.data = gui_command;
    chatter.publish( &str_msg );
  }
  //} else {
  //  Serial.println("Not Connected");
  //}
  nh.spinOnce();

  // Check if a client has connected
  WiFiClient gui_client = gui_server.available();
  if (!gui_client) {
    return;
  }

  // Wait until the client sends some data
  Serial.println("new client");
  while(!gui_client.available()){
    delay(1);
  }

  // Read the first line of the request
  String request = gui_client.readStringUntil('\r');
  Serial.println(request);
  gui_client.flush();

  // interpret Request
  if (request.indexOf("/CMD=UP") != -1)  {
    char gui_command[3] = "UP";
  }
  if (request.indexOf("/CMD=DOWN") != -1)  {
    char gui_command[5] = "DOWN";
  }
  if (request.indexOf("/CMD=LEFT") != -1)  {
    char gui_command[5] = "LEFT";
  }
  if (request.indexOf("/CMD=RIGHT") != -1)  {
    char gui_command[6] = "RIGHT";
  }
  if (request.indexOf("/CMD=FORWARD") != -1)  {
    char gui_command[8] = "FORWARD";
  }
  if (request.indexOf("/CMD=BACKWARD") != -1)  {
    char gui_command[9] = "BACKWARD";
  }
  //TODO - if any command recieved, digital write wifi rx pin

  // Return the response
  gui_client.println("HTTP/1.1 200 OK");
  gui_client.println("Content-Type: text/html");
  gui_client.println(""); //  do not forget this one
  gui_client.println("<!DOCTYPE HTML>");
  gui_client.println("<html>");

  gui_client.print("GUI Command: ");
  gui_client.print(gui_command);
  
  gui_client.println("<br><br>");
  gui_client.println("<a href=\"/CMD=UP\"\"><button>UP</button></a>");
  gui_client.println("<a href=\"/CMD=DOWN\"\"><button>DOWN</button></a><br />");
  gui_client.println("<a href=\"/CMD=LEFT\"\"><button>LEFT</button></a>");
  gui_client.println("<a href=\"/CMD=RIGHT\"\"><button>RIGHT</button></a><br />");
  gui_client.println("<a href=\"/CMD=FORWARD\"\"><button>FORWARD</button></a>");
  gui_client.println("<a href=\"/CMD=BACKWARD\"\"><button>BACKWARD</button></a><br />");
  gui_client.println("</html>");

  delay(1);
  Serial.println("GUI Client disconnected");
  Serial.println("");

  // Loop exproximativly at 1Hz
  delay(1000);

  
}
