// https://atadiat.com/en/e-ros-imu-and-arduino-how-to-send-to-ros/ : use string !
// rosrun rosserial_python serial_node.py tcp
// TCP/IP- https://github.com/ros-drivers/rosserial/blob/dd76994c67c5e4997ef64837c07afb4eb0d4df27/rosserial_arduino/src/ros_lib/examples/TcpHelloWorld/TcpHelloWorld.ino#L15
// TIMER- http://www.hardcopyworld.com/gnuboard5/bbs/board.php?bo_table=lecture_pract&wr_id=12
// ROS cam and IMU- http://grauonline.de/wordpress/?page_id=1951
// for mkr zero, ethernet library must be update to new version! (later than 2.0.0)
// can bus library : 
#include <SPI.h>
#include <Ethernet.h>

#include "Wire.h"


const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

uint16_t u_ax, u_ay, u_az;
uint16_t u_gx, u_gy, u_gz;
uint16_t u_temp;


// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/String.h>
#include <improved_topic_logger/imu_serial.h> // custom message

#define PIN_TRIGGER 7

byte mac[] = {  }; //physical mac address
IPAddress ip(192, 168, 1, 3);
IPAddress server(192,168,1,1);
const uint16_t serverPort = 11411; // rosserial socket server port, 11411


// node handler
ros::NodeHandle nh;
improved_topic_logger::imu_serial imu_msg;
ros::Publisher pub_imu("/mcu/mpu6050", &imu_msg);

volatile unsigned long trigger_time = 0;
volatile unsigned long imu_time = 0;
unsigned long time_sec = 0;
unsigned long time_nsec = 0;
volatile unsigned long triggerCounter = 0;

void setup() {
  pinMode(PIN_TRIGGER, OUTPUT);

  // Connect the Ethernet
  Ethernet.begin(mac, ip);
   
  // Let some time for the Ethernet Shield to be initialized
  delay(1000);

  Serial.println("");
  Serial.println("Ethernet connection info...");
  Serial.println("IP address: ");
  Serial.println(Ethernet.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(pub_imu);

  
  // Another way to get IP
  Serial.print("MY IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x00);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  // 0: +-2g (16384 LSB/g), 1: 4g 8192, 2: 8g 4096, 3: 16g 2048
  Wire.endTransmission(true);
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  // 0: 250 deg/s (131), 1: 500 deg/s (65.5), 2: 1000 deg/s (32.8), 3: 2000 deg/s (16.4)
  Wire.endTransmission(true);
  
  // Configure interrupt pin
  Wire.beginTransmission(MPU_ADDR);
//  Wire.write(0x37); //Talk to the INT_PIN_CFG
 // Wire.write(0b00000000);
  //Wire.write(0x38); //
  //Wire.write(0b00000001);
  Wire.endTransmission(true);
}

uint8_t cnt = 1;
uint32_t cnt_imu = 0;
uint32_t cnt_trigger = 0;
long publisher_timer;

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x      = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y      = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z      = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  u_ax = (uint16_t) 32768 + (uint16_t) accelerometer_x;
  u_ay = (uint16_t) 32768 + (uint16_t) accelerometer_y;
  u_az = (uint16_t) 32768 + (uint16_t) accelerometer_z;

  u_temp = (uint16_t) 32768 + (uint16_t) temperature;
  u_gx = (uint16_t) 32768 + (uint16_t) gyro_x;
  u_gy = (uint16_t) 32768 + (uint16_t) gyro_y;
  u_gz = (uint16_t) 32768 + (uint16_t) gyro_z;


  // print out data
  
  /*Serial.print("aX = "); Serial.print(acc[0]);
  Serial.print(" | aY = "); Serial.print(acc[1]);
  Serial.print(" | aZ = "); Serial.print(acc[2]);
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(gyro[0]);
  Serial.print(" | gY = "); Serial.print(gyro[1]);
  Serial.print(" | gZ = "); Serial.print(gyro[2]);
  Serial.println();*/
 
  String data = "$" + String(u_ax) + "," + String(u_ay) + "," + String(u_az) 
  + "," + String(u_gx) + "," + String(u_gy) + "," + String(u_gz) + "*" ;
  
  // Serial.println(data);
  int length = data.indexOf("*") + 2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);

  if(micros() > publisher_timer){
    // step 1: request reading from sensor
    imu_time = micros(); // microseconds
    time_sec  = imu_time/1000000;
    time_nsec = imu_time - time_sec*1000000;
    
    publisher_timer = imu_time + 3000; // 150 Hz
    imu_msg.stamp.sec = time_sec;
    imu_msg.stamp.nsec = time_nsec;
    imu_msg.seq  = cnt_imu;
    imu_msg.data = data_final;
    ++cnt;
    ++cnt_imu;

    // count (160 Hz IMU. ~ 20 Hz image)
    if(cnt > 7){
      cnt = 1;
      digitalWrite(PIN_TRIGGER, HIGH);
      digitalWrite(PIN_TRIGGER, LOW);
      imu_msg.flag_trigger = 1;
    }
    else{ // non-triggered signal
      imu_msg.flag_trigger = 0;      
    }
    pub_imu.publish(&imu_msg);
    nh.spinOnce();
  }
}
