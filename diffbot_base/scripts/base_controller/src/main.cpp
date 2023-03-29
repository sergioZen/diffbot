// ESP32 keep rebooting when trying to use rosserial:
//
// Change in: ~/Arduino/libraries/ros_lib/ros.h
// #if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
//   #include "ArduinoTcpHardware.h"
// #else
//   #include "ArduinoHardware.h"
// #endif
//
// Into:
// #if defined(ROSSERIAL_ARDUINO_TCP)
//   #include "ArduinoTcpHardware.h"
// #else
//   #include "ArduinoHardware.h"
// #endif

//#define ROSSERIAL_ARDUINO_TCP
#define ESP32
 
//#include <WiFi.h>

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>


#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#define IR_pin 16
#include "diffbot_base_config.h"
#include "base_controller.h"
#include "L298N_driver.h"
#include "L298N_MotorShield.h"


/* Include definition of serial commands */
#include "commands.h"
 
ros::NodeHandle nh;

using namespace diffbot;

L298NMotorController motor_controller_right = L298NMotorController(MOTOR_RIGHT);
L298NMotorController motor_controller_left = L298NMotorController(MOTOR_LEFT);

BaseController<L298NMotorController, L298N_MotorShield> base_controller(nh, &motor_controller_left, &motor_controller_right);

IPAddress server(192, 168, 3, 151);  // Ubuntu ROSCORE node: 192.168.3.151:11311
const uint16_t serverPort = 11311;

// Variable initialization:

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int indexCmd = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// Clear the current command parameters
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  indexCmd = 0;
}


/*
// Run a command.  Commands are defined in commands.h
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    //_Reset the auto stop timer:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    //SBR:Serial.print("Left speed: ");
    Serial.print(arg1);
    //SBR:Serial.print(" Right speed: ");
    Serial.print(arg2);
    Serial.print(" ");
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    // Reset the auto stop timer:
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}
*/
   
   /*SBR: working and sending messages to rostopic echo chatter*/
   /*
   std_msgs::String str_msg;
   ros::Publisher chatter("chatter", &str_msg);
   
   char hello[13] = "hello world!";
   
    void setup()
    {
        //Serial.begin(57600);
        //Serial.println();
        //Serial.print("Connecting to ");
        nh.initNode();
        nh.advertise(chatter);

        delay(1000);
    }
   
   void loop()
   {
        str_msg.data = hello;
        chatter.publish( &str_msg );
        nh.spinOnce();
        delay(1000);

        if (nh.connected()) {  
            // Call all the callbacks waiting to be called
            nh.spinOnce();
            delay(1000); // 20Hz
            //Serial.print("\nOK Loop TCP-IP!!!");
        }
        else{
            nh.initNode();
            delay(500);
            nh.logerror("Initialize DiffBot Motor Controllers");
            //Serial.print(".");       
        }
    }
    */

std_msgs::String str_msg;
   ros::Publisher chatter("chatter", &str_msg);
   
   char hello[13] = "hello world!";
   
    void setup()
    {
        //Serial.begin(57600);
        //Serial.println();
        //Serial.print("Connecting to ");
        nh.initNode();
        nh.advertise(chatter);

        delay(1000);

        base_controller.setup();
        base_controller.init();

        nh.logerror("Initialize DiffBot Motor Controllers");
        motor_controller_left.begin();
        motor_controller_right.begin();
        nh.logdebug("Setup finished");        
    }
   
   void loop()
   {
        str_msg.data = hello;
        chatter.publish( &str_msg );
        nh.spinOnce();
        delay(1000);

        if (nh.connected()) {  
            // Call all the callbacks waiting to be called
            nh.spinOnce();
            delay(1000); // 20Hz
            //Serial.print("\nOK Loop TCP-IP!!!");
        }
        else{
            nh.initNode();
            delay(500);
            nh.logerror("Initialize DiffBot Motor Controllers");
            //Serial.print(".");       
        }
    }

/*
void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println("white");
    WiFi.begin("white", "whitewhite!");
    
    while (WiFi.status() != WL_CONNECTED) // -> wifi delay -> a must -> other wise it will restart continuously 
    {
        delay(500);
        Serial.print(".");
    }
    delay(2000);

    Serial.println("WiFi connected - IP address:  ");
    Serial.println(WiFi.localIP());
    
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
 
    // Definindo a conexão com o servidor do soquete (rosserial)
    nh.getHardware() -> setConnection(server, serverPort);  
    nh.initNode();
    while (nh.getHardware() ->connected()!=1) {
     
        nh.getHardware() -> setConnection(server, serverPort);  
        nh.initNode();
        delay(50);
        Serial.print(".");   
    }

    Serial.print("\nOK TCP-IP = ");
    Serial.println(nh.getHardware() -> getLocalIP());
    base_controller.setup();
    Serial.println("Después setup");
    base_controller.init();
    Serial.println("Después setup e init");

    nh.loginfo("Initialize DiffBot Motor Controllers");
    motor_controller_left.begin();
    motor_controller_right.begin();
    nh.loginfo("Setup finished");
}


void loop()
{
    static bool imu_is_initialized;

    if (nh.getHardware() ->connected()) {  
        // The main control loop for the base_conroller.
        // This block drives the robot based on a defined control rate
        ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
        if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec())
        {
            base_controller.read();
            base_controller.write();
            base_controller.lastUpdateTime().control = nh.now();
        }

        // This block stops the motor when no wheel command is received
        // from the high level hardware_interface::RobotHW
        command_dt = nh.now() - base_controller.lastUpdateTime().command_received;
        if (command_dt.toSec() >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
        {
            nh.logwarn("Emergency STOP");
            base_controller.eStop();
        }

        // This block publishes the IMU data based on a defined imu rate
        ros::Duration imu_dt = nh.now() - base_controller.lastUpdateTime().imu;
        if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
        {
            // Sanity check if the IMU is connected
            if (!imu_is_initialized)
            {
                //imu_is_initialized = initIMU();
                if(imu_is_initialized)
                    nh.loginfo("IMU Initialized");
                else
                    nh.logfatal("IMU failed to initialize. Check your IMU connection.");
            }
            else
            {
                //publishIMU();
            }
            base_controller.lastUpdateTime().imu = nh.now();
        }

        // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
        if(base_controller.debug())
        {
            ros::Duration debug_dt = nh.now() - base_controller.lastUpdateTime().debug;
            if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
            {
                base_controller.printDebug();
                base_controller.lastUpdateTime().debug = nh.now();
            }
        }
        
        // Call all the callbacks waiting to be called
        nh.spinOnce();
        delay(1000); // 20Hz
        Serial.print("\nOK Loop TCP-IP!!!");
    }
    else{
        Serial.println("disconnected\n");
        while (nh.getHardware() ->connected()!=1) {     
            nh.getHardware() -> setConnection(server, serverPort);  
            nh.initNode();
            delay(50);
            Serial.print(".");       
        }
    }
}
*/

