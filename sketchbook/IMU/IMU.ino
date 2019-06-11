#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <Wire.h>

void readSRF();
void readMPU();
void readCMPS();
long microsecondsToCentimeters(long);

#define CMPS_ADDRESS 0x60 // i2c address of compass
#define IMU_ADDRESS 0x68	// i2c address of IMU

int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

// these constants won't change. they're the input and output pin numbers
const int trigPin = 9;
const int echoPin = 10;

ros::NodeHandle nh;

// Use this to cut down memory:
// default is 25 subscribers, 25 publishers, input buffer 512 bytes, output buffer 512 bytes
// we want 3 subscribers, 3 publishers, 200 bytes input, 150 bytes output
//ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;

////// Published topics

std_msgs::Int32 distance_msg;
std_msgs::Float32 temp_msg;
std_msgs::Float32MultiArray rpy_msg;

ros::Publisher distance_topic("distance", &distance_msg);
ros::Publisher rpy_topic("rpy", &rpy_msg);
ros::Publisher temp_topic("imu_temp", &temp_msg);

float rpy_buf[3];

////// No subscribed topics

void setup()
{
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);	

	Wire.begin(); //connects I2C
	Wire.beginTransmission(IMU_ADDRESS);
	Wire.write(0x6B);			// PWR_MGMT_1 register
	Wire.write(0);				// set to zero (wakes up the MPU-6050 which starts asleep)
	Wire.endTransmission(true);

	nh.initNode();

	rpy_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension)*2);
	rpy_msg.layout.dim[0].label = "rpy";
	rpy_msg.layout.dim[0].size = 3;
	rpy_msg.layout.dim[0].stride = 1;
	rpy_msg.layout.data_offset = 0;
	rpy_msg.data = rpy_buf;
	rpy_msg.data_length = 3;

	// topics we publish to
	nh.advertise(distance_topic);
	nh.advertise(rpy_topic);
	nh.advertise(temp_topic);

	// no subscribed topics
}

void loop()
{
	readSRF();			// read the distance to the nearest obstacle
	readMPU();			// read the MPU for pitch and roll, and temperature
	readCMPS();			// read the compass for yaw

	distance_topic.publish(&distance_msg);
	rpy_topic.publish(&rpy_msg);
	temp_topic.publish(&temp_msg);
		
	nh.spinOnce();
	delay(100);
}

void readSRF()
{
	// establish variables for duration of the trig, and the distance result in centimeters:
	long duration;
	uint32_t cm;

	// The trig))) is triggered by a HIGH pulse of 2 or more microseconds.
	// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(5);
	digitalWrite(trigPin, LOW);

	// The echo pin is used to read the signal from the trig))): a HIGH pulse
	// whose duration is the time (in microseconds) from the sending of the trig
	// to the reception of its echo off of an object.
	duration = pulseIn(echoPin, HIGH);

	// convert the time into a distance
	cm = microsecondsToCentimeters(duration);

	distance_msg.data = cm;
}

void readCMPS()
{
	// read the bearing from the CMPS01
	
	byte highByte;
	byte lowByte;
	
	Wire.beginTransmission(CMPS_ADDRESS);			// starts communication with cmps03
	Wire.write(2);						// Sends the register we wish to read
	Wire.endTransmission();
 
	Wire.requestFrom(CMPS_ADDRESS, 2);			// requests high byte
	while (Wire.available() < 2);				// while there is a byte to receive
	highByte = Wire.read();					// reads the byte as an integer
	lowByte = Wire.read();
	
	rpy_msg.data[2] = ((highByte<<8)+lowByte)/10; 
}

void readMPU()
{
	int minVal=265; int maxVal=402;

	Wire.beginTransmission(IMU_ADDRESS);
	Wire.write(0x3B);					// starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	
	Wire.requestFrom(IMU_ADDRESS,14,true);			// request a total of 14 registers

/*	MPU6050 calibration offsets

	const uint8_t acelX_offset = -585;
	const uint8_t acelY_offset = 2927;
	const uint8_t acelZ_offset = 1491;
	const uint8_t gyroX_offset = -84;
	const uint8_t gyroY_offset = 340;
	const uint8_t gyroZ_offset = 197;
*/	

	AcX=(Wire.read()<<8|Wire.read()) - 585;			// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)		
	AcY=(Wire.read()<<8|Wire.read()) + 2927;		// 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ=(Wire.read()<<8|Wire.read()) + 1491;		// 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	int16_t temp=Wire.read()<<8|Wire.read();		// 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX=(Wire.read()<<8|Wire.read()) - 84;			// 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY=(Wire.read()<<8|Wire.read()) + 340;			// 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ=(Wire.read()<<8|Wire.read()) + 197;			// 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

	int xAng = map(AcX,minVal,maxVal,-90,90); 
	int yAng = map(AcY,minVal,maxVal,-90,90); 
	int zAng = map(AcZ,minVal,maxVal,-90,90);

	// degrees:
	rpy_msg.data[0] = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); 
	// AngY = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); // use compass instead
	rpy_msg.data[1] = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

	// temperature (deg C):
	temp_msg.data = ((float) temp) / 340. + 36.53;	
}

long microsecondsToCentimeters(long microseconds) {
	// The speed of sound is 340 m/s or 29 microseconds per centimeter.
	// The trig travels out and back, so to find the distance of the object we
	// take half of the distance travelled.
	return microseconds / 29 / 2;
}
