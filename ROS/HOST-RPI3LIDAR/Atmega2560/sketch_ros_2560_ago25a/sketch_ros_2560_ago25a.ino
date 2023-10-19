/*
Date: 25.08.2023
Project: VRI 4WD Robot
Design: Carlos Eduardo Magrin/Eduardo Todt

Especifications:
- 4WD Platform
- Power Module DF Robot
- Board DF Robot Atmega2560 + I/O Expansion Shield V7.1 DF Robot1
- (3) Ultrasound sensor DF Robot URM37 V3.2
- (3) IR Sharp GP2Y0A02YK (20-150cm)
- Logic level converter (UART1)
- GPS NEO-M8N (UART2 - RX)

 The circuit: 
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:
*/

#include "ros.h" 
#include <std_msgs/String.h>

#include<Wire.h>
#include <TinyGPS.h>
//#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/NavSatFix.h>

//Ultrasound sensors
#define US1_ECHO 3 //PWM output 0-50000us, every 50us represent 10mm
#define US1_TRIG 2 //Trigger pin
//#define US2_ECHO 5
//#define US2_TRIG 4
#define US3_ECHO 7
#define US3_TRIG 6
//#define US4_ECHO 9
//#define US4_TRIG 8
#define US5_ECHO 12
#define US5_TRIG 11

ros::NodeHandle  nh;

sensor_msgs::Range range_ir1_msg;
sensor_msgs::Range range_ir2_msg;
sensor_msgs::Range range_ir3_msg;
sensor_msgs::Range range_us1_msg;
//sensor_msgs::Range range_us2_msg;
sensor_msgs::Range range_us3_msg;
//sensor_msgs::Range range_us4_msg;
sensor_msgs::Range range_us5_msg;
//std_msgs::String imu_msg;
//sensor_msgs::Imu imu_msg;
//sensor_msgs::Temperature temperature_msg;
sensor_msgs::NavSatFix gps_msg;

ros::Publisher pub_ir1_range( "/ir1_sharp", &range_ir1_msg);
ros::Publisher pub_ir2_range( "/ir2_sharp", &range_ir2_msg);
ros::Publisher pub_ir3_range( "/ir3_sharp", &range_ir3_msg);
ros::Publisher pub_us1_range( "/us1_urm", &range_us1_msg);
//ros::Publisher pub_us2_range( "/us2_urm", &range_us2_msg);
ros::Publisher pub_us3_range( "/us3_urm", &range_us3_msg);
//ros::Publisher pub_us4_range( "/us4_urm", &range_us4_msg);
ros::Publisher pub_us5_range( "/us5_urm", &range_us5_msg);
//ros::Publisher imu("imu", &imu_msg);
//ros::Publisher pub_imu("/imu_mpu6050", &imu_msg); 
//ros::Publisher pub_temperature("/temp_mpu6050", &temperature_msg); 
ros::Publisher pub_gps("/gps_neo8m", &gps_msg); 

//Infrared sharp
const int analog_ir1 = 0, analog_ir2 = 1, analog_ir3 = 2;
unsigned long range_timer;

//GPS NEO8M
float lat,lon; // create variable for latitude and longitude object
TinyGPS gps; // create gps object

//Detection Range: 20-150cm  ou 8" - 60"
//Example code based on Sharp GP2Y0A02YK
//https://wiki.dfrobot.com/Sharp_GP2Y0A02YK_IR_ranger_sensor__150cm___SKU_SEN0013_
//   IR2
//IR1    IR3
float ir1_sharp(int analog_ir1){ 
  float ir1_volts = analogRead(analog_ir1)*0.0048828125; // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float ir1_distance = 65*pow(ir1_volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)
  delay(10);                                     // arbitary wait time.  
  
  return(ir1_distance);
}
float ir2_sharp(int analog_ir2){
  float ir2_volts = analogRead(analog_ir2)*0.0048828125; // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float ir2_distance = 65*pow(ir2_volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)
  delay(10);                                     // arbitary wait time.  
  
  return(ir2_distance);
}
float ir3_sharp(int analog_ir3){
  float ir3_volts = analogRead(analog_ir3)*0.0048828125; // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float ir3_distance = 65*pow(ir3_volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)
  delay(10);                                     // arbitary wait time.  
  
  return(ir3_distance);
}

//Code based on DF Robot PWM trigger mode example 
//https://wiki.dfrobot.com/URM37_V5.0_Ultrasonic_Sensor_SKU_SEN0001_
//   US3
//US2    US4
//US1    US5
float us1_urm(){
  digitalWrite(US1_TRIG, LOW);
  delay(10);
  digitalWrite(US1_TRIG, HIGH);
  
  unsigned long us1_lowLevelTime = pulseIn(US1_ECHO, LOW);  
  float us1_distanceMeasured = us1_lowLevelTime / 50;  // every 50us low level stands for 1cm
  
  return(us1_distanceMeasured); 
}
/*
float us2_urm(){
  digitalWrite(US2_TRIG, LOW);
  delay(10);
  digitalWrite(US2_TRIG, HIGH);
  
  unsigned long us2_lowLevelTime = pulseIn(US2_ECHO, LOW);  
  float us2_distanceMeasured = us2_lowLevelTime / 50;  // every 50us low level stands for 1cm
  
  return(us2_distanceMeasured); 
}
*/
float us3_urm(){
  digitalWrite(US3_TRIG, LOW);
  delay(10);
  digitalWrite(US3_TRIG, HIGH);
  
  unsigned long us3_lowLevelTime = pulseIn(US3_ECHO, LOW);  
  float us3_distanceMeasured = us3_lowLevelTime / 50;  // every 50us low level stands for 1cm
  
  return(us3_distanceMeasured); 
}
/*
float us4_urm(){
  digitalWrite(US4_TRIG, LOW);
  delay(10);
  digitalWrite(US4_TRIG, HIGH);
  
  unsigned long us4_lowLevelTime = pulseIn(US4_ECHO, LOW);  
  float us4_distanceMeasured = us4_lowLevelTime / 50;  // every 50us low level stands for 1cm
  
  return(us4_distanceMeasured); 
}
*/
float us5_urm(){
  digitalWrite(US5_TRIG, LOW);
  delay(10);
  digitalWrite(US5_TRIG, HIGH);
  
  unsigned long us5_lowLevelTime = pulseIn(US5_ECHO, LOW);  
  float us5_distanceMeasured = us5_lowLevelTime / 50;  // every 50us low level stands for 1cm
  
  return(us5_distanceMeasured); 
}

long publisher_timer;

void gps_neo8m(){
  while(Serial2.available()){ // check for gps data
    if(gps.encode(Serial2.read())){// encode gps data
      gps.f_get_position(&lat,&lon); // get latitude and longitude
    }
  
  gps_msg.latitude = lat;
  gps_msg.longitude = lon;
    
  }
}

void setup(){
  pinMode(US1_TRIG, OUTPUT);                   // A low pull on pin COMP/TRIG
  digitalWrite(US1_TRIG, HIGH);                // Set to HIGH
  pinMode(US1_ECHO, INPUT);                    // Sending Enable PWM mode command
  
  pinMode(US3_TRIG, OUTPUT);                   // A low pull on pin COMP/TRIG
  digitalWrite(US3_TRIG, HIGH);                // Set to HIGH
  pinMode(US3_ECHO, INPUT);                    // Sending Enable PWM mode command
  
  pinMode(US5_TRIG, OUTPUT);                   // A low pull on pin COMP/TRIG
  digitalWrite(US5_TRIG, HIGH);                // Set to HIGH
  pinMode(US5_ECHO, INPUT);                    // Sending Enable PWM mode command
  
  //pinMode(US2_TRIG, OUTPUT);                   // A low pull on pin COMP/TRIG
  //digitalWrite(US2_TRIG, HIGH);                // Set to HIGH
  //pinMode(US2_ECHO, INPUT);                    // Sending Enable PWM mode command
  
  //pinMode(US4_TRIG, OUTPUT);                   // A low pull on pin COMP/TRIG
  //digitalWrite(US4_TRIG, HIGH);                // Set to HIGH
  //pinMode(US4_ECHO, INPUT);                    // Sending Enable PWM mode command
  
  
  //GPS
  Serial2.begin(9600); // connect gps sensor
  
  nh.initNode();
  nh.advertise(pub_ir1_range);
  nh.advertise(pub_ir2_range);
  nh.advertise(pub_ir3_range);
  nh.advertise(pub_us1_range);
  //nh.advertise(pub_us2_range);
  nh.advertise(pub_us3_range);
  //nh.advertise(pub_us4_range);
  nh.advertise(pub_us5_range);
  nh.advertise(pub_gps);
  
  range_ir1_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_ir1_msg.header.frame_id =  "/ir1_range";
  range_ir1_msg.field_of_view = 0.0254;
  range_ir1_msg.min_range = 20.00;
  range_ir1_msg.max_range = 150.00;
  
  range_ir2_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_ir2_msg.header.frame_id =  "/ir2_range";
  range_ir2_msg.field_of_view = 0.0254;
  range_ir2_msg.min_range = 20.00;
  range_ir2_msg.max_range = 150.00;
  
  range_ir3_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_ir3_msg.header.frame_id =  "/ir3_range";
  range_ir3_msg.field_of_view = 0.0254;
  range_ir3_msg.min_range = 20.00;
  range_ir3_msg.max_range = 150.00;
  
  range_us1_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_us1_msg.header.frame_id =  "/us1_urm";
  range_us1_msg.field_of_view = 0.1;  // fake
  range_us1_msg.min_range = 2.00;
  range_us1_msg.max_range = 800.00;
  
  /*
  range_us2_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_us2_msg.header.frame_id =  "/us2_urm";
  range_us2_msg.field_of_view = 0.1;  // fake
  range_us2_msg.min_range = 2.00;
  range_us2_msg.max_range = 800.00;
  */
  
  range_us3_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_us3_msg.header.frame_id =  "/us3_urm";
  range_us3_msg.field_of_view = 0.1;  // fake
  range_us3_msg.min_range = 2.00;
  range_us3_msg.max_range = 800.00;
  
  /*
  range_us4_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_us4_msg.header.frame_id =  "/us4_urm";
  range_us4_msg.field_of_view = 0.1;  // fake
  range_us4_msg.min_range = 2.00;
  range_us4_msg.max_range = 800.00;
  */
  
  range_us5_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_us5_msg.header.frame_id =  "/us5_urm";
  range_us5_msg.field_of_view = 0.1;  // fake
  range_us5_msg.min_range = 2.00;
  range_us5_msg.max_range = 800.00;
  
  gps_msg.header.frame_id =  "/gps_neo8m";
}

void loop(){
  
  //imu_mpu6050();
  delay(100);
  gps_neo8m();
    
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 50){
    range_ir1_msg.range = ir1_sharp(analog_ir1);
    range_ir2_msg.range = ir2_sharp(analog_ir2);
    range_ir3_msg.range = ir3_sharp(analog_ir3);
    range_us1_msg.range = us1_urm();
    //range_us2_msg.range = us2_urm();
    range_us3_msg.range = us3_urm();
    //range_us4_msg.range = us4_urm();
    range_us5_msg.range = us5_urm();
    
    range_ir1_msg.header.stamp = nh.now();
    range_ir2_msg.header.stamp = nh.now();
    range_ir3_msg.header.stamp = nh.now();
    range_us1_msg.header.stamp = nh.now();
    //range_us2_msg.header.stamp = nh.now();
    range_us3_msg.header.stamp = nh.now();
    //range_us4_msg.header.stamp = nh.now();
    range_us5_msg.header.stamp = nh.now();
    //imu_msg.header.stamp = nh.now();
    //temperature_msg.header.stamp = nh.now();
    gps_msg.header.stamp = nh.now();
    
    pub_ir1_range.publish(&range_ir1_msg);
    pub_ir2_range.publish(&range_ir2_msg);
    pub_ir3_range.publish(&range_ir3_msg);
    pub_us1_range.publish(&range_us1_msg);
    //pub_us2_range.publish(&range_us2_msg);
    pub_us3_range.publish(&range_us3_msg);
    //pub_us4_range.publish(&range_us4_msg);
    pub_us5_range.publish(&range_us5_msg);
    
    pub_gps.publish(&gps_msg);
        
    range_timer =  millis() + 50;
  }  
  
  nh.spinOnce();
}
