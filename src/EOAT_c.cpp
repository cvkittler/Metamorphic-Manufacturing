/*****************************************************
** This code runs the End of Arm Tooling for the 
**  WPI Metamorphic Manufacturing Robot, 2021-2022
**  and is controlled from an external ROS Melodic
**  node
****************************************************
** {Licesense}
****************************************************
** Authors: Sean Barry, Charles Kittler,
**			Jonathan Landay, Jacob Mackenzi,
**			Aidan Melgar, Patrick Siegler
** Copyright: 2022, MMMQP
** Credits:
** License: {}
** Version: 0.2.1
** Maintainer: {}
** Email: gr-metamorphicmanufacturingmqp2021@wpi.edu
** Status: "Dev"
*****************************************************/
#include "tooling.cpp"
#include <iostream>
#include <sys/poll.h>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"

//Runmodes: You can change these
bool autosetup = true; //if true will not prompt user to confirm setup
bool Jogging = false; //Runmode. True = manual mode with command line input, false = automatic mode with ROS input
bool testTooling = false; //Test Mode. True will run preprogrammed tests. False will go direcly to waiting for instructions

//Program Variables. Dont change these.
bool calibrating = false; //Robot calibration status. Disables safeties while calibrating
bool checkCenter = false; //basically same thing as calibrating, but for center switches
bool eStopTriggered = false; //Robto ESTOP Status
bool volatile exceedLimits = false; //has the robot hit a limit switch
bool recievedInstruction = false; //If robot has recieved instructions
int dutyCycle = 127; //0-255. 
int errorMode = 0;
int speed=0; //active desired speed in mm/s
int targetSpeed = 0; //ros input speed
float recoveryL = 0.0, recoveryR = 0.0;
float posL = 5.0, posR = 5.0;  //active setpoints in mm
float targetL = 0.0, targetR = 0.0; //ros input setpoints
float posLMax = 130.0, posRMax = 130.0;

geometry_msgs::Point32 pos_return;
ros::Publisher chatter_pub;
ros::Subscriber sub;

enum stateMachine { Stop, initEOAT, waitForInstruction, run, recovery, state_calibrate, state_e_stop, state_invalid_command, toolingTest };
stateMachine previousState = Stop, currentState = initEOAT;


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


void recoverySaveData(){
	eoat.stop();
	recoveryL = eoat.left.currentPosition;
	recoveryR = eoat.right.currentPosition;
	printf("Manipulators stopped at L:%f R:%f\n",recoveryL,recoveryR);
}

/**
 * Left manipulator exceeds operating bounds. Triggers E Stop
 */
void leftCloseDetected(int pin, int level, uint32_t tick){
	if(!checkCenter){
		if(errorMode ==0){
			printf("Left manipulator exceeded maximum bounds (too close to center)\n");
			errorMode = -1;
			exceedLimits = true;
			recoverySaveData();
		}
	}
}

/**
 * Right manipulator exceeds operating bounds. Triggers E Stop
 */
void rightCloseDetected(int pin, int level, uint32_t tick){
	if(!checkCenter){
		if(errorMode ==0){
			printf("Right manipulator exceeded maximum bounds (too close to center)\n");
			errorMode = 1;
			exceedLimits = true;
			recoverySaveData();
		}
	}
}

/**
 * Left manipulator exceeds operating bounds. Triggers E Stop if not actively calibrating
 */
void leftOpenDetected(int pin, int level, uint32_t tick){
	if(!calibrating){
		if(errorMode ==0){
			printf("Left manipulator exceeded maximum bounds (too far from center)\n");
			errorMode = -2;
			exceedLimits = true;
			recoverySaveData();
		}
	}
}

/**
 * Right manipulator exceeds operating bounds. Triggers E Stop if not actively calibrating
 */
void rightOpenDetected(int pin, int level, uint32_t tick){
	if(!calibrating){
		if(errorMode ==0){
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
 * Sets up the Raspberry Pi GPIO
 *
 * @para n/a
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
 * Actually sets the pinmodes
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
 * 		x is the left manipulator position in mm
 * 		y is the right manipulator position in mm
 * 		z is a status where see readme, or read the code...
 */
void publishCurrentPos(){
	pos_return.x = eoat.left.currentPosition;
	pos_return.y = eoat.right.currentPosition;
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
	else{
		pos_return.z = 9;
	}
	
	chatter_pub.publish(pos_return);
	ros::spinOnce();
}

/**
 * Records ROS input on message recieved. Updates command if its different from last one
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

int main(int argc, char *argv[]){
	struct pollfd mypoll = { STDIN_FILENO, POLLIN|POLLPRI };

	
	ros::init(argc, argv, "mmm_eoat");
	ros::NodeHandle n;
	chatter_pub = n.advertise<geometry_msgs::Point32>("mmm_eoat_position",500);
	sub = n.subscribe("mmm_eoat_command",10,rosSetPosition);
	ros::Rate loop_rate(10);
	
	gpioSetTimerFunc(0, 1000, publishCurrentPos);					
	
	while(true){
		switch(currentState){
			case Stop:
				printf("State = Stop");
				publishCurrentPos();
				//stopRobot
			
				break;
			
			case initEOAT:
				printf("State = Init\n");
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
				
				//calibrate tooling
				if(autosetup){
					eoat.calibrateTooling();
					eoat.calibrateCenters();
					posLMax = eoat.left.maxPosition;
					posRMax = eoat.right.maxPosition;
					publishCurrentPos();
					printf("max positions set at %f and %f\n",posLMax,posRMax);
				}
				else{
					printf("Tooling Calibration Necessary. Manipulators will move during calibration.\n");
					printf("Continue? (y/n)\n");
					if(yesorno()){
						eoat.calibrateTooling();
						eoat.calibrateCenters();
						posLMax = eoat.left.maxPosition;
						posRMax = eoat.right.maxPosition;
						publishCurrentPos();
					}
					else{
						printf("Tooling Calibration aborted by user\n");
						exit(1);
					}
				}
				
				if(testTooling){
					currentState = toolingTest;
				}
				else{
					currentState = waitForInstruction;
				}
			
				eoat.stop();
				break;
			
			case waitForInstruction:
				printf("State = wait for instruction\n");
				publishCurrentPos();
				
				//Manual Jogging Mode. Interacts with the Terminal
				if(Jogging){
					printf("left:%f right:%f\n",eoat.left.currentPosition, eoat.right.currentPosition);
					printf("Manual jogging Mode. Awaiting Command Line Input.\n");
					printf("enter left pos, right pos speed (float float int) speed -732 triggers a recalibration\n");
					while(!recievedInstruction&&!exceedLimits){
						//Checks for instruction, if no instruction in 1s exit to continue processing errors
						if(poll(&mypoll,1,1000)){
							std::cin >> posL >> posR >> speed;
							recievedInstruction = true;
						}					
					}	
				}
				
				//Automatic Mode. Interacts with ROS Commands from mmm_eoat_command topic
				else{
					if(recievedInstruction){
						posL = targetL;
						posR = targetR;
						speed = targetSpeed;
					}
					else{
						sleep(1); //basically a do nothing so that program continues processing errors if no instruction recieved
					}
				}
				
				//Checks that no manipulator exceeded limits somehow...
				if(exceedLimits){
					currentState = recovery;
				}
				
				//if instruction recieved from either command line or ROS execute it... otherwise loop again
				else{
					if(recievedInstruction){
						recievedInstruction = false;
							if(speed == 911){
								currentState = state_e_stop;
							}
							else if(speed == -732){
								currentState = state_calibrate;
							}
							else if(speed<=0||speed>15||posL<0||posR<0||posL>posLMax||posR>posRMax){
								currentState = state_invalid_command;
							}
							else{
								currentState = run;
							}
						
					}
				}
				
				break;
			
			case run:
				publishCurrentPos();
				printf("State = run\n");
			
				eoat.moveManipulators(posL,posR,speed);
				
				if(exceedLimits){
					currentState = recovery;
				}
				else{
					currentState = waitForInstruction;
				}
				break;
			
			case recovery:
				publishCurrentPos();
				printf("State = recovery\n");
				
				if(Jogging){
					printf("Exit Recovery? (y/n)\n");
						if(yesorno()){
							printf("Are you sure? (y/n)\n");
							if(yesorno()){
								eoat.left.setEnable(true);
								eoat.right.setEnable(true);
								eoat.recoverTooling(errorMode);
								exceedLimits = false;
								errorMode = 0;
								
								printf("recalibrate? (y/n)\n");
								if(yesorno()){
									eoat.calibrateTooling();
								}
								currentState = waitForInstruction;
							}
							else{
								printf("User declined to recover .Emergency Stop.\n");
								emergencyStop();
							}
						}
						else{
							printf("User declined to recover .Emergency Stop.\n");
							emergencyStop();
						}
					}
				else{
					eoat.left.setEnable(true);
					eoat.right.setEnable(true);
					eoat.recoverTooling(errorMode);
					exceedLimits = false;
					errorMode = 0;
					eoat.calibrateTooling();
					currentState = waitForInstruction;
				}
			
				break;
			
			case state_calibrate:
				publishCurrentPos();
				eoat.calibrateTooling();
				
				currentState = waitForInstruction;
				
				break;
				
			case state_e_stop:
				publishCurrentPos();
				emergencyStop();
				
				break;
				
			case state_invalid_command:
				printf("invalid command\n");
				publishCurrentPos();
				currentState = waitForInstruction;
				
				break;
			
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
}
