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

// include the pinchangeint library to handle multiple interrupts from the R/C controller
#include <PinChangeInt.h>

#include <Servo.h>

void throttleCmd(const std_msgs::Int32 &);
void steeringCmd(const std_msgs::Int32 &);
void auxCmd(const std_msgs::Int32 &);
void lightLED(void);
void flashLED(void);
void calcThrottle(void);
void calcSteering(void);
void calcAux(void);
int round10 (int a);

// Assign channel in pins - the RX connects to these
#define STEERING_IN_PIN		8		// Channel 1
#define THROTTLE_IN_PIN		9		// Channel 2
#define AUX_IN_PIN			10		// Channel 3

// Assign channel out pins - the commands received are mirrored out on these, apart from AUX which is used for internal purposes (e.g. save a waypoint)
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
#define MAX_AUX				2010

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
volatile uint16_t unSteeringInShared;
volatile uint16_t unThrottleInShared;
volatile uint16_t unAuxInShared;

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
ros::NodeHandle nh;

// ROS messages that can be published
std_msgs::Int32 steering_msg;
std_msgs::Int32 throttle_msg;
std_msgs::Int32 aux_msg;

// topics we publish changed values to
ros::Publisher set_steering_topic("steering_set", &steering_msg);
ros::Publisher set_throttle_topic("throttle_set", &throttle_msg);
ros::Publisher set_aux_topic("aux_set", &aux_msg);

void steeringCmd(const std_msgs::Int32 &steering_msg){
	if (mode == RC_MODE)
		return;
			
	// set the pulse duration
	uint16_t cmdSteeringInShared = (uint16_t)(steering_msg.data);
	unSteeringInShared = map(cmdSteeringInShared, 0, 180, MIN_STEERING, MAX_STEERING);		// map 0-90-180 command to R/C values
		
	// set the steering flag to indicate that a new steering signal has been received
	bUpdateFlagsShared |= STEERING_FLAG;
}

void throttleCmd(const std_msgs::Int32 &throttle_msg){
	if (mode == RC_MODE)
		return;

	// set the pulse duration
	uint16_t cmdThrottleInShared = (uint16_t)(throttle_msg.data);
	unThrottleInShared = map(cmdThrottleInShared, 0, 180, MIN_THROTTLE, MAX_THROTTLE);		// map 0-90-180 command values to R/C values

	// set the throttle flag to indicate that a new throttle signal has been received
	bUpdateFlagsShared |= THROTTLE_FLAG;
}

void auxCmd(const std_msgs::Int32 &aux_msg){	
	// set the pulse duration
	unAuxInShared = (uint16_t)(aux_msg.data);					// aux command is 1 for RC_MODE, 2 for AUTO_MODE, 3 for CMD_MODE
	// set the steering flag to indicate that a new aux signal has been received
	bUpdateFlagsShared |= AUX_FLAG;
}

ros::Subscriber<std_msgs::Int32> cmd_steering_topic("steering_cmd", &steeringCmd);
ros::Subscriber<std_msgs::Int32> cmd_throttle_topic("throttle_cmd", &throttleCmd);
ros::Subscriber<std_msgs::Int32> cmd_aux_topic("aux_cmd", &auxCmd);

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
	nh.advertise(set_steering_topic);
	nh.advertise(set_throttle_topic);
	nh.advertise(set_aux_topic);

	// topics we subscribe to
	nh.subscribe(cmd_steering_topic);
	nh.subscribe(cmd_throttle_topic);
	nh.subscribe(cmd_aux_topic);

	lightLED();
}

void loop()
{
	// create local variables to hold a local copies of the channel inputs
	// these are declared static so that thier values will be retained 
	// between calls to loop.
	static int16_t unSteeringIn;
	static int16_t unThrottleIn;
	static int16_t unAuxIn;
	
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
			unSteeringIn = unSteeringInShared;
		}
		
		if (bUpdateFlags & THROTTLE_FLAG)
		{
			unThrottleIn = unThrottleInShared;
		}
		
		if (bUpdateFlags & AUX_FLAG)
		{
			unAuxIn = unAuxInShared;
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
	
	// do any processing from here onwards
	// only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
	// variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by 
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
	
	if (bUpdateFlags & STEERING_FLAG)
	{
		if (sigSteering.readMicroseconds() != unSteeringIn)
		{
			if (unSteeringIn < MIN_STEERING || unSteeringIn > MAX_STEERING){
				char msgbuf[64];
				sprintf(msgbuf, "Out of range STEERING (%d)", unSteeringIn);
				nh.logwarn(msgbuf);
			}
			
//			sigSteering.writeMicroseconds(unSteeringIn);
			sigSteering.write(map(unSteeringIn, MIN_STEERING, MAX_STEERING, 0, 180));
			
			// publish the value to the /set_steering topic
			steering_msg.data = unSteeringIn;
			set_steering_topic.publish(&steering_msg);
		}
	}
	
	if (bUpdateFlags & THROTTLE_FLAG)
	{
		// if the last value written is different to the width of the last pulse received
		// send the pulse to the throttle signal line
		
		if (sigThrottle.readMicroseconds() != unThrottleIn)
		{
			if (unThrottleIn < MIN_THROTTLE || unThrottleIn > MAX_THROTTLE){
				char msgbuf[64];
				sprintf(msgbuf, "Out of range THROTTLE (%d)", unThrottleIn);
				nh.logwarn(msgbuf);
			}
			
			sigThrottle.write(map(unThrottleIn, MIN_THROTTLE, MAX_THROTTLE, 0, 180));
			
			// publish the value to the /set_throttle topic
			throttle_msg.data = unThrottleIn;
			set_throttle_topic.publish(&throttle_msg);
		}
	}
	
	if (bUpdateFlags & AUX_FLAG)												// got a signal on the AUX channel
	{
		if (sigAux.readMicroseconds() != unAuxIn)
		{ 
			if (unAuxIn < MIN_AUX || unAuxIn > MAX_AUX){
				char msgbuf[64];
				sprintf(msgbuf, "Out of range AUX (%d)", unAuxIn);
				nh.logwarn(msgbuf);
			}
			
			sigAux.writeMicroseconds(unAuxIn);

			// de-bounce the CMD_AUX
			uint32_t prev_mode = mode;

			if (unAuxIn < 1970)
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
		unSteeringInShared = (uint16_t) round10(micros() - ulSteeringStart);
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
		unThrottleInShared = (uint16_t) round10(micros() - ulThrottleStart);
		
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
		unAuxInShared = (uint16_t) round10(micros() - ulAuxStart);
		bUpdateFlagsShared |= AUX_FLAG;
	}
}

int round10 (int a)
{
	return a >= 0 ? (a+5)/10*10 : (a-5)/10*10 ;
}
