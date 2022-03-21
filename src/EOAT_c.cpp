/*****************************************************
** This code runs the End of Arm Tooling for the 
**  WPI Metamorphic Manufacturing Robot, 2021-2022
**  and is compatible with either local control, or
**  control from an external ROS node
****************************************************
** {Licesense}
****************************************************
** Authors: Sean Barry, Charles Kittler,
**			Jonathan Landay, Jacob Mackenzi,
**			Aidan Melgar, Patrick Siegler
** Copyright: 2022, MMMQP
** Credits:
** License: {}
** Version: 0.3.1
** Maintainer: {}
** Email: gr-metamorphicmanufacturingmqp2021@wpi.edu
** Status: "Dev"
*****************************************************/

/**************************************************************************************************
* IMPORTANT NOTE: 
*	The EOAT node operates using distances with respect to outer sensor.
* 	The other ROS nodes operate using distances with respect to tool distance from closed.
*	
*	IF YOU PLAN ON CHANGING THE ROS MESSAGE INTERACTIONS:
* 		CONVERT LOCAL DISTANCES TO WRT INSIDE BEFORE TRANSMITTING TO WHEREVER
*		CONVERT INCOMING DISTANCES TO WRT TO OUTSIDE BEFORE SENDING MOVEMENT COMMANDS
***************************************************************************************************/

#include "tooling.cpp"
#include <iostream>
#include <sys/poll.h>
#include <sstream>
#include <signal.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"

//Runmodes: You can change these
bool autosetup = true; //if true will not prompt user to confirm setup
bool testTooling = false; //Test Mode. True will run preprogrammed tests. False will go direcly to waiting for instructions
int reportTime = 1; //Interval between EOAT reporting status in seconds.


//Program Variables. Dont change the following variables
//Error and calibration flags
bool volatile calibrating = false; //Robot calibration status. Disables safeties while calibrating
bool volatile checkCenter = false; //basically same thing as calibrating, but for center switches
bool eStopTriggered = false; //Robto ESTOP Status
bool volatile exceedLimits = false; //has the robot hit a limit switch
bool recievedInstruction = false; //If robot has recieved instructions
int volatile errorMode = 0;

int dutyCycle = 127; //0-255. 

//control stuff
int speed=0; //active desired speed in mm/s
int targetSpeed = 0; //ros input speed
float recoveryL = 0.0, recoveryR = 0.0;
float posL = 5.0, posR = 5.0;  //active setpoints in mm
float targetL = 0.0, targetR = 0.0; //ros input setpoints
float posLMax = 130.0, posRMax = 130.0; //Physical Max distance the screwblock can move towards center (no tools attached)
float offsetL = 0.0, offsetR = 0.0; //tool offsets
float toolLMax = 130.0, toolRMax = 130.0; //Physical max distance the screwblock can move towards center with current tools

//ros stuff
geometry_msgs::Point32 pos_return;
ros::Publisher chatter_pub;
ros::Subscriber sub;

//program states
enum stateMachine { Stop, initEOAT, waitForInstruction, run, recovery, state_calibrate, state_e_stop, state_invalid_command, set_tool_offset, toolingTest };
stateMachine previousState = Stop, currentState = initEOAT;

void publishCurrentPos();

//Signal handler.
void mySigintHandler(int sig){
	// Deals with ctrl+c
	if(sig == SIGINT){
		printf("exit on ctrl c\n");
		gpioWrite(EN_L,0);
		gpioWrite(EN_R,0);
		gpioTerminate();
		ros::shutdown();
		raise(SIGTERM);
	}
	//Publish position timer
	if(sig == SIGALRM){
		publishCurrentPos();
		alarm(reportTime);
	}
}


/**
 * Sets up the tooling object which contains 2 manipulator objects
 *
 * @return eoat : Tooling : object representing EOAT
 */
static Tooling setupTooling(){
    Manipulator leftManipulator(EN_L, DIR_L, STEP_L, true);
    Manipulator rightManipulator(EN_R, DIR_R, STEP_R, false);
    
    Tooling eoat(leftManipulator, rightManipulator);
    return eoat;
}

Tooling eoat = setupTooling();

/**
 * Emergency Stop. Kills program for whatever reason
 */
void emergencyStop(){
	printf("EMERGENCY STOP INITIATED\n");
	gpioWrite(EN_L,0);
	gpioWrite(EN_R,0);
	eStopTriggered = true;
	
	printf("EMERGENCY STOP EXECUTED. KILLING PROGRAM\n");
	raise(SIGTERM);
}

/**
 * Notes the precieved position when sensor tripped.
 */
void recoverySaveData(){
	eoat.stop();
	recoveryL = eoat.left.currentPosition;
	recoveryR = eoat.right.currentPosition;
	printf("Manipulators stopped at L:%f R:%f\n",recoveryL,recoveryR);
}

/**
 * ISR function for inner left sensor
 * If the left manipulator is not conducting initial calibration:
 *  Stops movements, saves data, preps for recovery
 */
void leftCloseDetected(int pin, int level, uint32_t tick){
	printf("ISR:lc\n");
	if(!checkCenter){
		usleep(100);
		if((errorMode == 0)&&gpioRead(pin)){
			printf("Left manipulator exceeded maximum bounds (too close to center)\n");
			errorMode = -1;
			exceedLimits = true;
			recoverySaveData();
		}
	}
}

/**
 * ISR function for inner right sensor
 * If the right manipulator is not conducting initial calibration:
 *  Stops movements, saves data, preps for recovery
 */
void rightCloseDetected(int pin, int level, uint32_t tick){
	printf("ISR:rc\n");
	if(!checkCenter){
		usleep(100);
		if((errorMode == 0)&&gpioRead(pin)){
			printf("Right manipulator exceeded maximum bounds (too close to center)\n");
			errorMode = 1;
			exceedLimits = true;
			recoverySaveData();
		}
	}
}

/**
 * ISR function for outer left sensor
 * If the left manipulator is not calibrating:
 *  Stops movements, saves data, preps for recovery
 */
void leftOpenDetected(int pin, int level, uint32_t tick){
	printf("ISR:lo\n");
	if(!calibrating){
		usleep(100);
		if((errorMode == 0)&&gpioRead(pin)){
			printf("Left manipulator exceeded maximum bounds (too far from center)\n");
			errorMode = -2;
			exceedLimits = true;
			recoverySaveData();
		}
	}
	
}
/**
 * ISR function for outer right sensor
 * If the right manipulator is not calibrating:
 *  Stops movements, saves data, preps for recovery
 */
void rightOpenDetected(int pin, int level, uint32_t tick){
	printf("ISR:ro\n");
	if(!calibrating){
		usleep(100);
		if((errorMode == 0)&&gpioRead(pin)){
			printf("Right manipulator exceeded maximum bounds (too far from center)\n");
			errorMode = 2;
			exceedLimits = true;
			recoverySaveData();
		}
	}
}

 
/**
 * Queries user for a y/n input on command line
 *
 * @para n/a
 * @return boolean : True if user returns y or Y, else False
 */ 
bool yesorno(){
	char c;
	scanf(" %c",&c);
	
	if((c == 'y')||(c == 'Y')){
		return true;
	}
	else{
		return false;
	}
}


/**
 * Prep for setting up GPIO
 *
 * @return boolean : True if setup completed, else False
 */
bool setupRPiPins(){
	int es_in = ES_INPUT, slc = S_L_close, slo = S_L_open, src = S_R_close, sro = S_R_open, el = EN_L, dl = DIR_L, sl = STEP_L, er = EN_R, dr = DIR_R, sr = STEP_R;
	printf("Setting up WPI MMMQP End Effector\n");
	printf("Please ensure the Raspberry Pi pins are wired as follows\n(Pin Numbers given in BCM numbering)\n");
	
	printf("Safety Sensor Input Pins:\n");	
	printf("Left Manipulator Maximum (Closest to center) on pin %d\n",slc);
	printf("Left Manipulator Minimum (Closest to outside) on pin %d\n",slo);
	printf("Right Manipulator Maximum (Closest to center) on pin %d\n",src);
	printf("Right Manipulator Minimum (Closest to outside) on pin %d\n",sro);
	
	printf("\nStepper Control Output Pins:\n");
	printf("Left Enable on pin %d\n",el);
	printf("Left Direction on pin %d\n",dl);
	printf("Left Pulse/Step on pin %d\n",sl);
	printf("Right Enable on pin %d\n",er);
	printf("Right Direction on pin %d\n",dr);
	printf("Right Pulse/Step on pin %d\n",sr);
	
	if(autosetup){
		return true;
	}
	else{
		printf("\n\n\nContinue? (y/n)\n");
		if(yesorno()){
			return true;
		}
		else{
			printf("Pin setup aborted by user\n");
			return false;
		}
	}
}

/**
 * Actually setup the GPIO
 */
bool setupPinModes(){
	if (gpioInitialise() < 0){
		fprintf(stderr, "pigpio initialisation failed\n");
		return false;
	}
	gpioSetMode(EN_L, PI_OUTPUT);
	gpioSetMode(DIR_L, PI_OUTPUT);
	gpioSetMode(STEP_L, PI_OUTPUT);
	gpioSetMode(EN_R, PI_OUTPUT);
	gpioSetMode(DIR_R, PI_OUTPUT);
	gpioSetMode(STEP_R, PI_OUTPUT);
	
	gpioSetMode(S_L_close, PI_INPUT);
	gpioSetMode(S_L_open, PI_INPUT);
	gpioSetMode(S_R_close, PI_INPUT);
	gpioSetMode(S_R_open, PI_INPUT);
	
	gpioSetPullUpDown(S_L_close,PI_PUD_DOWN);
	gpioSetPullUpDown(S_L_open,PI_PUD_DOWN);
	gpioSetPullUpDown(S_R_close,PI_PUD_DOWN);
	gpioSetPullUpDown(S_R_open,PI_PUD_DOWN);
	gpioSetPullUpDown(ES_INPUT,PI_PUD_DOWN);

	gpioSetISRFunc(S_L_close, RISING_EDGE, 0, leftCloseDetected);
	gpioSetISRFunc(S_L_open, RISING_EDGE, 0, leftOpenDetected);
	gpioSetISRFunc(S_R_close, RISING_EDGE, 0, rightCloseDetected);
	gpioSetISRFunc(S_R_open, RISING_EDGE, 0, rightOpenDetected);
	
	
	printf("PinModes set\n");
	return true;
}

/**
 * Publishes the current position and status to topic "mmm_eoat_position" as a Point32 where
 * 	x is the left manipulator position in mm
 * 	y is the right manipulator position in mm
 * 	z is a status where see readme, or read the code...
 */
void publishCurrentPos(){
	pos_return.x = toolLMax - eoat.left.currentPosition;
	pos_return.y = toolRMax - eoat.right.currentPosition;
	if(currentState == Stop){
		pos_return.z = 3;
	}
	if(currentState == initEOAT){
		pos_return.z = 4;
	}
	else if(currentState == waitForInstruction){
		pos_return.z = 0;
	}
	else if(currentState == run){
		pos_return.z = 1;
	}
	else if(currentState == recovery){
		pos_return.z = 2;
	}
	else if(currentState == state_calibrate){
		pos_return.z = 8;
	}
	else if(currentState == state_e_stop){
		pos_return.z = 911;
	}
	else if(currentState == state_invalid_command){
		pos_return.z = 7;
	}
	else if(currentState == toolingTest){
		pos_return.z = 5;
	}
	else if(currentState == set_tool_offset){
		pos_return.x = toolLMax;
		pos_return.y = toolRMax;
		pos_return.z = 6;
	}
	else{
		pos_return.z = 9;
	}
	
	chatter_pub.publish(pos_return);
	ros::spinOnce();
}

/**
 * Records ROS input on message recieved. Flags for execution if any instruction is different from previous
 */
void rosSetPosition(const geometry_msgs::Point32::ConstPtr& msg){
	if(msg->x != targetL){
		targetL = msg->x;
		recievedInstruction = true;
	}
	if(msg->y != targetR){
		targetR = msg->y;
		recievedInstruction = true;
	}
	if(msg->z != targetSpeed){
		targetSpeed = int(msg->z);
		recievedInstruction = true;
	}
	
}

//Actual code that does stuff
int main(int argc, char *argv[]){
	//ROS initialization stuffs
	ros::init(argc, argv, "mmm_eoat", ros::init_options::NoSigintHandler);
	ros::NodeHandle n;
	ros::spinOnce();

	//This line probably obsolete.
	struct pollfd mypoll = { STDIN_FILENO, POLLIN|POLLPRI };
	
	//Setup the status publishing
	chatter_pub = n.advertise<geometry_msgs::Point32>("mmm_eoat_position",500);
	sub = n.subscribe("mmm_eoat_command",10,rosSetPosition);
	ros::Rate loop_rate(10);			
	
	//State machine
	while(ros::ok()){
		switch(currentState){
			case Stop:
				printf("State = Stop");
				publishCurrentPos();
			
				break;
			
			//All the initialization stuffs
			case initEOAT:
				printf("State = Init\n");
				
				//publish that EOAT is initializing
				publishCurrentPos();
				
				//setup pins
				if(!setupRPiPins()){
					printf("Error during Pin Setup. Exiting\n");
					exit(1);
				}
				else{
					if(!setupPinModes()){
						printf("Error during Pin Setup. Exiting\n");
						exit(1);
					}	
				}
				
				//setup the signal handling to deal with program termination, and the status update timer
				signal(SIGINT, mySigintHandler);
				signal(SIGALRM, mySigintHandler);
				alarm(1);
				
				//Initial EOAT calibration: Automatic mode
				if(autosetup){
					//Calibrate outsides
					printf("error %d eL %d calib %d center %d\n",errorMode, exceedLimits, calibrating, checkCenter);
					eoat.left.currentPosition = 0.0;
					eoat.right.currentPosition = 0.0;
					exceedLimits = false;
					calibrating = true;
					errorMode = 0;
					eoat.calibrateTooling();
					printf("error %d eL %d calib %d center %d\n",errorMode, exceedLimits, calibrating, checkCenter);
					exceedLimits = false;
					checkCenter = true;
					errorMode = 0;
					
					//Calibrate insides
					printf("error %d eL %d calib %d center %d\n",errorMode, exceedLimits, calibrating, checkCenter);
					eoat.calibrateCenters();
					posLMax = eoat.left.maxPosition - 4;
					posRMax = eoat.right.maxPosition - 4;
					toolLMax = posLMax;
					toolRMax = posRMax;
					publishCurrentPos();
					printf("max positions set at %f and %f\n",posLMax,posRMax);
				}
				
				//Initial EOAT calibration: manual mode
				else{
					printf("Tooling Calibration Necessary. Manipulators will move during calibration.\n");
					printf("Continue? (y/n)\n");
					if(yesorno()){
						//Calibrate outsides
						printf("error %d eL %d calib %d center %d\n",errorMode, exceedLimits, calibrating, checkCenter);
						eoat.left.currentPosition = 0.0;
						eoat.right.currentPosition = 0.0;
						exceedLimits = false;
						calibrating = true;
						errorMode = 0;
						eoat.calibrateTooling();
						printf("error %d eL %d calib %d center %d\n",errorMode, exceedLimits, calibrating, checkCenter);
						exceedLimits = false;
						checkCenter = true;
						errorMode = 0;
					
						//Calibrate insides
						printf("error %d eL %d calib %d center %d\n",errorMode, exceedLimits, calibrating, checkCenter);
						eoat.calibrateCenters();
						posLMax = eoat.left.maxPosition - 4;
						posRMax = eoat.right.maxPosition - 4;
						toolLMax = posLMax;
						toolRMax = posRMax;
						publishCurrentPos();
						printf("max positions set at %f and %f\n",posLMax,posRMax);
						printf("tool max positions set at %f %f\n",toolLMax,toolRMax);
					}
					else{
						printf("Tooling Calibration aborted by user\n");
						exit(1);
					}
				}
				
				//if programed to run tests, go to test state, else go to main loop
				if(testTooling){
					currentState = toolingTest;
				}
				else{
					currentState = waitForInstruction;
				}
			
				eoat.stop();
				break;
			
			//Checks for errors, checks for new instructions
			case waitForInstruction:
				printf("State = wait for instruction\n");
				publishCurrentPos();
				
				//if a new instruction recieved, prep them for execution
				if(recievedInstruction){
					posL = targetL;
					posR = targetR;
					speed = targetSpeed;
				}
				else{
					sleep(1); //basically a do nothing so that program continues processing errors if no instruction recieved
				}
				
				
				//Checks that no manipulator exceeded limits somehow...
				if(exceedLimits){
					currentState = recovery;
				}
				
				//if instruction recieved check for command codes and route to appropriate execution stage
				else{
					if(recievedInstruction){
						recievedInstruction = false;
							//check for important commands
							if(speed == -911){
								currentState = state_e_stop;
							}
							else if (speed == -111){
								currentState = set_tool_offset;
							}
							else if(speed == -732){
								currentState = state_calibrate;
							}
						
							//movement command parsing
							else{
								//convert directions for dist from center to dist from outside
								printf("command %f %f tool %f %f",posL,posR,toolLMax,toolRMax);
								posR = toolRMax - posR;
								posL = toolLMax - posL;
								printf("center %f %f\n",posL,posR);
								//5 = ? - 20
								if(speed<=0){
									//printf("speed too slow\n");
									//currentState = state_invalid_command;
									speed = 10;
								}
								else if(speed>15){
									//printf("speed too fast\n");
									//currentState = state_invalid_command;
									speed = 10;
								}
								else if(posL<0||posR<0){
									printf("pos<0\n");
									currentState = state_invalid_command;
								}
								else if(posL>posLMax||posR>posRMax){
									printf("pos>max\n");
									currentState = state_invalid_command;
								}
								else{
									currentState = run;
								}
							}
						
					}
				}
				
				break;
			
			//Execute move command.
			case run:
				publishCurrentPos();
				printf("State = run\n");
			
				eoat.moveManipulators(posL,posR,speed);
				
				//checks if above command errored out on a sensor. If yes, sends to recovery. Else wait for new instruction
				if(exceedLimits){
					currentState = recovery;
				}
				else{
					currentState = waitForInstruction;
				}
				break;
			
			//Figure out what went wrong and fix it
			case recovery:
				publishCurrentPos();
				printf("State = recovery\n");
				
				eoat.left.setEnable(true);
				eoat.right.setEnable(true);
				eoat.recoverTooling(errorMode);
				exceedLimits = false;
				errorMode = 0;
				eoat.calibrateTooling();
				targetL = 0;
				targetR = 0;
				currentState = waitForInstruction;
				
				break;
			
			//Calibrates EOAT to outside again
			case state_calibrate:
				publishCurrentPos();
				eoat.calibrateTooling();
				
				currentState = waitForInstruction;
				
				break;
			
			//Things have gone very wrong. uh oh...
			case state_e_stop:
				publishCurrentPos();
				emergencyStop();
				
				break;
			
			//Tell the higher level code you sent a command the EOAT cannot execute
			case state_invalid_command:
				printf("invalid command\n");
				publishCurrentPos();
				currentState = waitForInstruction;
				
				break;
			
			//Set the distance the current tools extend past inner sensor
			case set_tool_offset:
				printf("setToolOffsets\n");
				offsetL = posL;
				offsetR = posR;
				
				toolLMax = posLMax - offsetL;
				toolRMax = posRMax - offsetR;
				
				publishCurrentPos();
				currentState = waitForInstruction;
				
				break;
			
			//test the tooling. Havent touched this in forever, so hopefully it still works
			case toolingTest:
				publishCurrentPos();
				printf("running preset tests");
			
				usleep(1000);
				printf("testing right move in\n");
				eoat.moveAManipulator(20,1,false);
				
				usleep(1000);
				printf("testing right move to same position\n");
				eoat.moveAManipulator(20,1,false);
				
				usleep(1000);
				printf("testing right move out\n");
				eoat.moveAManipulator(10,5,false);
				
				usleep(1000);
				printf("testing left move in\n");
				eoat.moveAManipulator(20,1,true);
				
				usleep(1000);
				printf("testing left move to same position\n");
				eoat.moveAManipulator(20,1,true);
				
				usleep(1000);
				printf("testing left move out\n");
				eoat.moveAManipulator(10,5,true);

				usleep(1000);
				printf("recalibrate\n");
				eoat.calibrateTooling();
				
				usleep(1000);
				printf("test move both out\n");
				eoat.moveManipulators(20,20,5);
				
				usleep(1000);
				printf("test move both in\n");
				eoat.moveManipulators(100,100,5);
				
				usleep(1000);
				printf("test move both different amount in\n");
				eoat.moveManipulators(20,15,5);
				
				usleep(1000);
				printf("back to identical\n");
				eoat.moveManipulators(15,15,10);
				
				usleep(1000);
				printf("test move different directions\n");
				eoat.moveManipulators(20,10,5);
				printf("Manipulator Tests Complete\n");
				
				eoat.stop();
				currentState = waitForInstruction;
				break;
			
		}
	}
	gpioWrite(EN_L,0);
	gpioWrite(EN_R,0); 
	std::cin.get();
}
