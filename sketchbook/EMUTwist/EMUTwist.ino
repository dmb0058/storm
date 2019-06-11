// ROS Engine Contro Module
//
// Read throttle, steering and aux from either an R/C controller or from a set of ROS topics
//
// R/C controller overrides any current programmed commands (from the topics)
//
// RC code from rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts - 
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//


// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

// include the pinchangeint library to handle multiple interrupts from the R/C controller
#include <PinChangeInt.h>

#include <Servo.h>

void motionCmd(const geometry_msgs::Twist &);
void auxCmd(const std_msgs::Int32 &);
void lightLED(void);
void flashLED(void);
void calcThrottle(void);
void calcSteering(void);
void calcAux(void);
int round10 (int a);
uint16_t convertSteeringFromROSToRC(double);
uint16_t convertThrottleFromROSToRC(double);
uint16_t convertSteeringFromRCToST(uint16_t);
uint16_t convertThrottleFromRCToST(uint16_t);
double convertSteeringFromRCToROS(uint16_t);
double convertThrottleFromRCToROS(uint16_t);
double convertSteeringFromSTToROS(uint16_t);
double convertThrottleFromSTToROS(uint16_t);

// Assign channel in pins - the RX connects to these
#define STEERING_IN_PIN		8		// Channel 1
#define THROTTLE_IN_PIN		9		// Channel 2
#define AUX_IN_PIN			10		// Channel 3

// Assign channel out pins - the commands received are mirrored out on these, apart from AUX which is used for internal purposes (e.g. save a waypoint)
// For how to configure the Sabertooth, see the DIP Switch Wizard for
//   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
// Be sure to select RC Microcontroller Mode for use with this sample.
//
// Connections to make:
//   Arduino Pin 5  ->  Sabertooth S1
//   Arduino Pin 6  ->  Sabertooth S2
//   Arduino GND    ->  Sabertooth 0V
//   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
//
#define STEERING_OUT_PIN	5
#define THROTTLE_OUT_PIN	6
#define AUX_OUT_PIN			7

#define RED_LED_PIN			12
#define GREEN_LED_PIN		13

// These are the min and max values expected from the R/C
#define MIN_STEERING		950
#define MAX_STEERING		2010
#define MIN_THROTTLE		1010
#define MAX_THROTTLE		1990
#define MIN_AUX				960
#define MAX_AUX				2020

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing

Servo sigSteering;
Servo sigThrottle;
Servo sigAux;

// These bit flags are set in bUpdateFlagsShared to indicate which channels have new signals
#define STEERING_FLAG	1
#define THROTTLE_FLAG	2
#define AUX_FLAG		4

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediately take local copies so that the ISR can keep ownership of the 
// shared ones. To access these in loop we first turn interrupts off with noInterrupts
// we take a copy to use in loop 
// and then turn interrupts back on as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t RCSteeringInShared;
volatile uint16_t RCThrottleInShared;
volatile uint16_t RCAuxInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulSteeringStart;
uint32_t ulThrottleStart;
uint32_t ulAuxStart;

#define RC_MODE		1
#define AUTO_MODE	2
#define CMD_MODE	3

uint32_t mode = RC_MODE;			// default is RC control

int last_led_time;				// last time we changed the red led (for flashing)

// ROS objects
//ros::NodeHandle nh;
ros::NodeHandle_<ArduinoHardware, 5, 5, 128, 128> nh;

// ROS messages that can be published
std_msgs::Int32 aux_msg;
geometry_msgs::Twist motion_msg;

// topics we publish changed values to
ros::Publisher set_aux_topic("aux_set", &aux_msg);
ros::Publisher set_motion_topic("motion_set", &motion_msg);

void motionCmd(const geometry_msgs::Twist &motion_msg){
	if (mode == RC_MODE)
 		return;
			
	// unpack the Twist parameters - speed in m/s, rotation in degrees/s and convert to R/C controls

	RCSteeringInShared = convertSteeringFromROSToRC(motion_msg.angular.z);
	RCThrottleInShared = convertThrottleFromROSToRC(motion_msg.linear.x);
		
	char msgbuf[64];

	sprintf(msgbuf, "New Twist: RCSteering: %d, RCThrottle: %d", RCSteeringInShared, RCThrottleInShared);
	nh.loginfo(msgbuf);

	// set the steering and throttle flags to indicate that a new steering signal has been received
	// (later we'll check if the new values are actually different to the previous ones)
	bUpdateFlagsShared |= (STEERING_FLAG | THROTTLE_FLAG);
}

void auxCmd(const std_msgs::Int32 &aux_msg){	
	// set the pulse duration
	RCAuxInShared = (uint16_t)(aux_msg.data);					// aux command is 1 for RC_MODE, 2 for AUTO_MODE, 3 for CMD_MODE
	// set the steering flag to indicate that a new aux signal has been received
	bUpdateFlagsShared |= AUX_FLAG;
}

ros::Subscriber<std_msgs::Int32> cmd_aux_topic("aux_cmd", &auxCmd);
ros::Subscriber<geometry_msgs::Twist> cmd_motion_topic("motion_cmd", &motionCmd);

void setup()
{
	pinMode(GREEN_LED_PIN, OUTPUT);
	pinMode(RED_LED_PIN, OUTPUT);
	
	// Attach servo objects, these will generate the correct 
	// pulses for driving Electronic speed controllers, servos or other devices
	// designed to interface directly with RC Receivers
	//
	// The Sabertooth accepts servo pulses from 1000 us to 2000 us.
	// We need to specify the pulse widths in attach(). 
	//	0 degrees will be full reverse, 
	//	180 degrees will be full forward.
	//	90 will stop the motor. 
	// 
	// Whether the servo pulses control the motors individually or control 
	// throttle and turning depends on your mixed mode setting.
	// 
	// Notice that the second and third arguments are important.
	// With a single argument, the range is 44 to 141 degrees, with 92 being stopped.
	// With all three arguments, we can use 0 to 180 degrees, with 90 being stopped.

	sigSteering.attach(STEERING_OUT_PIN, 1000, 2000);
	sigThrottle.attach(THROTTLE_OUT_PIN, 1000, 2000);
	sigAux.attach(AUX_OUT_PIN);

	// using the PinChangeInt library, attach the interrupts
	// used to read the channels
	
	PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering, CHANGE); 
	PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE); 
	PCintPort::attachInterrupt(AUX_IN_PIN, calcAux, CHANGE); 

	nh.initNode();

	// topics we publish to
	nh.advertise(set_aux_topic);
	nh.advertise(set_motion_topic);

	// topics we subscribe to
	nh.subscribe(cmd_aux_topic);
	nh.subscribe(cmd_motion_topic);

	lightLED();
}

void loop()
{
	// create local variables to hold a local copies of the channel inputs
	// these are declared static so that their values will be retained 
	// between calls to loop.
	static int16_t RCSteeringIn;
	static int16_t RCThrottleIn;
	static int16_t STSteeringIn;
	static int16_t STThrottleIn;
	static int16_t RCAuxIn;
	
	// local copy of update flags
	static uint8_t bUpdateFlags;

	// check shared update flags to see if any channels have a new signal
	if (bUpdateFlagsShared)
	{
		noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

		// take a local copy of which channels were updated in case we need to use this in the rest of loop
		bUpdateFlags = bUpdateFlagsShared;
		
		// in the current code, the shared values are always populated
		// so we could copy them without testing the flags
		// however in the future this could change, so lets
		// only copy when the flags tell us we can.
		
		if (bUpdateFlags & STEERING_FLAG)
		{
			RCSteeringIn = RCSteeringInShared;
		}
		
		if (bUpdateFlags & THROTTLE_FLAG)
		{
			RCThrottleIn = RCThrottleInShared;
		}
		
		if (bUpdateFlags & AUX_FLAG)
		{
			RCAuxIn = RCAuxInShared;
		}

		// clear shared copy of updated flags as we have already taken the updates
		// we still have a local copy if we need to use it in bUpdateFlags
		bUpdateFlagsShared = 0;
		
		// we have local copies of the inputs, so now we can turn interrupts back on
		// as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
		// service routines own these and could update them at any time. During the update, the 
		// shared copies may contain junk. Luckily we have our local copies to work with :-)
		interrupts(); 
	}
	
	// Do any processing from here onwards
	// Only use the local values RCAuxIn, RCSteeringIn, RCThrottleIn, STSteeringIn and STThrottleIn.
	// The shared variables RCAuxInShared, RCThrottleInShared, RCSteeringInShared are always owned by 
	// the interrupt routines and should not be used in loop
	
	// the following code provides simple pass through 
	// this is a good initial test, the Arduino will pass through
	// receiver input as if the Arduino is not there.
	// This should be used to confirm the circuit and power
	// before attempting any custom processing in a project.
	
	// we are checking to see if the channel value has changed, this is indicated	
	// by the flags. For the simple pass through we don't really need this check,
	// but for a more complex project where a new signal requires significant processing
	// this allows us to only calculate new values when we have new inputs, rather than
	// on every cycle.
	
	boolean publishUpdate = false;
	static int16_t prevRCSteeringIn = sigSteering.readMicroseconds();
	static int16_t prevRCThrottleIn = sigThrottle.readMicroseconds();

	if (bUpdateFlags & STEERING_FLAG)
	{
		if (RCSteeringIn != prevRCSteeringIn)
		{
			if (RCSteeringIn < MIN_STEERING || RCSteeringIn > MAX_STEERING){
				char msgbuf[64];
				sprintf(msgbuf, "Out of range STEERING (%d)", STSteeringIn);
				nh.logwarn(msgbuf);
			}
			
			// convert from microseconds to degrees
			// The "write" method simply maps the "degrees" to microseconds and calls the "writeMicroseconds" method anyway.
			sigSteering.write(convertSteeringFromRCToST(RCSteeringIn));
			motion_msg.angular.z = convertSteeringFromRCToROS(RCSteeringIn);
		}
		else
			motion_msg.angular.z = convertSteeringFromRCToROS(prevRCSteeringIn);
	}
	
	if (bUpdateFlags & THROTTLE_FLAG)
	{
		// if the last value written is different to the width of the last pulse received
		// send the pulse to the throttle signal line
		
		if (RCThrottleIn != prevRCThrottleIn)
		{
			if (RCThrottleIn < MIN_THROTTLE || RCThrottleIn > MAX_THROTTLE){
				char msgbuf[64];
				sprintf(msgbuf, "Out of range THROTTLE (%d)", STThrottleIn);
				nh.logwarn(msgbuf);
			}
			
			sigThrottle.write(convertThrottleFromRCToST(RCThrottleIn));
			motion_msg.linear.x = convertThrottleFromRCToROS(RCThrottleIn);
			publishUpdate = true;
		}
		else
			motion_msg.linear.x = convertThrottleFromRCToROS(prevRCThrottleIn);
	}
	
	// if either the steering or throttle changed, publish the steering value in radians per second and the speed in m/s to the /set_motion topic
	if (publishUpdate)
		set_motion_topic.publish(&motion_msg);
	
	if (bUpdateFlags & AUX_FLAG)												// got a signal on the AUX channel
	{
		if (sigAux.readMicroseconds() != RCAuxIn)
		{ 
			if (RCAuxIn < MIN_AUX || RCAuxIn > MAX_AUX){
				char msgbuf[64];
				sprintf(msgbuf, "Out of range AUX (%d)", RCAuxIn);
				nh.logwarn(msgbuf);
			}
			
			sigAux.writeMicroseconds(RCAuxIn);

			// de-bounce the CMD_AUX
			uint32_t prev_mode = mode;

			if (RCAuxIn < 1970)
				mode = RC_MODE;
			else
				mode = AUTO_MODE;
				
			// publish the value to the /set_aux topic if it changed
			if (prev_mode != mode){
				lightLED();
				aux_msg.data = mode;
				set_aux_topic.publish(&aux_msg);
			}
		}
	}

	if (mode == CMD_MODE)
		flashLED();

	bUpdateFlags = 0;
	nh.spinOnce();
	delay(100);
}

// Green for RC mode, Red for ECM mode

void lightLED()
{
	switch (mode){
		case RC_MODE:								// green
			digitalWrite(GREEN_LED_PIN, HIGH);
			digitalWrite(RED_LED_PIN, LOW);
			break;
		case CMD_MODE:								// yellow
			digitalWrite(GREEN_LED_PIN, HIGH);
			digitalWrite(RED_LED_PIN, HIGH);
			break;		
		case AUTO_MODE:								// red
			digitalWrite(GREEN_LED_PIN, LOW);
			digitalWrite(RED_LED_PIN, HIGH);
			break;
	}
}

// flash LED red

void flashLED()
{
	if (micros() - last_led_time > 1000){
		digitalWrite(GREEN_LED_PIN, LOW);
		digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
		last_led_time = micros();
	}
}

// simple interrupt service routine

void calcSteering()
{
	if (mode != RC_MODE)
		return;
	
	if (digitalRead(STEERING_IN_PIN) == HIGH)
	{ 
		ulSteeringStart = micros();
	}
	else
	{
		RCSteeringInShared = (uint16_t) round10(micros() - ulSteeringStart);
		bUpdateFlagsShared |= STEERING_FLAG;
	}
}

void calcThrottle()
{
	if (mode != RC_MODE)
		return;
		
	// if the pin is high, its a rising edge of the signal pulse, so lets record its value
	if (digitalRead(THROTTLE_IN_PIN) == HIGH)
	{ 
		ulThrottleStart = micros();
	}
	else
	{
		// else it must be a falling edge, so lets get the time and subtract the time of the rising edge
		// this gives use the time between the rising and falling edges i.e. the pulse duration.
		RCThrottleInShared = (uint16_t) round10(micros() - ulThrottleStart);
		
		// set the throttle flag to indicate that a new throttle signal has been received
		bUpdateFlagsShared |= THROTTLE_FLAG;
	}
}

void calcAux()
{
	// always allow RC to toggle operating mode between EMC and RC
	
	if (digitalRead(AUX_IN_PIN) == HIGH)
	{ 
		ulAuxStart = micros();
	}
	else
	{
		RCAuxInShared = (uint16_t) round10(micros() - ulAuxStart);
		bUpdateFlagsShared |= AUX_FLAG;
	}
}

int round10(int a)
{
	return a >= 0 ? (a+5)/10*10 : (a-5)/10*10 ;
}

//////////////////////////////////////////////////////////////////////////////
// Conversion functions
//////////////////////////////////////////////////////////////////////////////

// Sabertooth accepts servo pulses from 1000 us to 2000 us.
// We need to specify the pulse widths in attach(). 0 degrees will be full reverse, 180 degrees will be
// full forward. Sending a servo command of 90 will stop the motor. Whether the servo pulses control
// the motors individually or control throttle and turning depends on your mixed mode setting.

// Convert a Twist.angular.z value (-3m/s to +3m/s) to an R/C value (MIN_STEERING to MAX_STEERING)

uint16_t convertSteeringFromROSToRC(double x)
{
	// multiply the float value by 100 to preserve two decimal places
	return map(x * 100, -36000, 36000, MIN_STEERING, MAX_STEERING);		// map +/- 360 degs/s command to R/C values
}

// Convert a Twist.linear.x value (-3m/s to +3m/s) to an R/C value (MIN_STEERING to MAX_STEERING)

uint16_t convertThrottleFromROSToRC(double x)
{
	// multiply the float value by 100 to preserve two decimal places
	return map(x * 100, -300, 300, MIN_THROTTLE, MAX_THROTTLE);			// map +/-3 m/s command to R/C values
}

uint16_t convertSteeringFromRCToST(uint16_t x)
{
	return map(x, MIN_STEERING, MAX_STEERING, 0, 180);
}

uint16_t convertThrottleFromRCToST(uint16_t x)
{
	return map(x, MIN_THROTTLE, MAX_THROTTLE, 0, 180);
}

double convertSteeringFromRCToROS(uint16_t x)
{
	return (double) round(map(x, MIN_STEERING, MAX_STEERING, 0, 18000)) / 100;
}

double convertThrottleFromRCToROS(uint16_t x)
{
	return (double) round(map(x, MIN_THROTTLE, MAX_THROTTLE, -300, 300)) / 100;
}

double convertSteeringFromSTToROS(uint16_t x)
{
	if (x < 90)
		return (double) map(x, 0, 90, -90, 0);
	else
		return (double) map(x, 90, 180, 0, 90);
}

double convertThrottleFromSTToROS(uint16_t x)
{
	if (x < 90)
		return (double) map(x, 0, 90, -3, 0);
	else
		return (double) map(x, 90, 180, 0, 3);
}

