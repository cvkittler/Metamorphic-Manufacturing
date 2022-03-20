#include "tooling.h"

//ALL POSITIONS RELATIVE TO OUTSIDE

/**
 * Constructor for a Tooling Object
 * 
 * @param l : Manipulator : manipulator object representing the left manipulator
 * @param r : Manipulator : manipulator object representing the right manipulator
 */ 
Tooling::Tooling(Manipulator l, Manipulator r):
	left(l),
	right(r)
	{
	}

/**
 * Stops the tooling
 */ 
void Tooling::stop(){
	this->left.setEnable(false);
	this->right.setEnable(false);
}

/**
 * Calibrates the manipulators by moving them to the outer sensors. 
 */ 
void Tooling::calibrateTooling(){
	printf("Calibration in progress....\n");
	printf("Calibrating Left Manipulator\n");
	this->left.calibrateManipulator();
	printf("Left Manipulator Calibrated....\n");
	printf("Calibrating Right Manipulator\n");
	this->right.calibrateManipulator();
	printf("Right Manipulator Calibrated....\n");
	printf("Calibration Complete\n");

}


/**
 * Calculates the distance between the outer and inner sensors and sets tool movement limitations.
 */ 
void Tooling::calibrateCenters(){
	this->moveManipulators(100,100,10);
	printf("Checking left center....\n");
	this->left.calibrateCenter();
	this->moveManipulators(100,100,10);
	printf("Left Center Set at %f\n",this->left.maxPosition);
	
	printf("Checking right center\n");
	this->right.calibrateCenter();
	this->moveManipulators(100,100,10);
	printf("Right center set at %f\n",this->right.maxPosition);
	printf("Centers set\n");
	this->moveManipulators(90,90,10);

}

/**
 * Moves a manipulator
 *
 * @para position : float : absolute position to move to relative to max open
 * @para speed : int : speed of manipulator in mm/s
 * @para side : bool : Selects the manipulator. True = left, false = right
 */ 
void Tooling::moveAManipulator(float position, int speed, bool side){
		if(side){
			this->left.moveManipulator(position, speed);
		}
		else{
			this->right.moveManipulator(position,speed);
		}
}

/**
 * Moves both manipulators
 *
 * @para positionL : float : absolute position to move Left Manipulator to relative to max open
 * @para positionR : float : absolute position to move Right Manipulator to relative to max open
 * @para speed : int : speed of manipulator in mm/s
 */ 
void Tooling::moveManipulators(float positionL, float positionR, int speed){
	int distMultiplierL, distMultiplierR;

	//if desired position more closed than current position, set direction closed
	if(positionL > this->left.currentPosition){
		distMultiplierL = 1;
		this->left.setDirection(true);
	}
	else{
		distMultiplierL = -1;
		this->left.setDirection(false);
	}
	if(positionR > this->right.currentPosition){
		distMultiplierR = 1;
		this->right.setDirection(true);
	}
	else{
		distMultiplierR = -1;
		this->right.setDirection(false);
	}
	
	
	//calculate the number of steps necessary to move each manipulator from the current position to the commanded position
	float distanceL = positionL - this->left.currentPosition;
	float distanceR = positionR - this->right.currentPosition;
	float distPerStepL = this->left.distPerStep;
	float distPerStepR = this->right.distPerStep;
	
	int num_pulses_L = this->left.stepsPerMillimeter * abs(distanceL);
	int num_pulses_R = this->right.stepsPerMillimeter * abs(distanceR);
	int frequency = this->left.stepsPerMillimeter * speed;
	int microperiod = (1/frequency)*1000000; //period in microseconds
	int delayTime = microperiod/2;
	
	printf("moving from L: %f and R: %f to L: %f R: %f \n",this->left.currentPosition, this->right.currentPosition, positionL, positionR);
	

	this->left.setEnable(true);
	this->right.setEnable(true);
	
	int l = 0, r = 0;
	bool leftRunning = true, rightRunning = true;
	
	// As long as at at least one manipulator has not reached its target AND no sensor is tripped:
	//  move the manipulators the number of pulses necessary. update position periodically (every ~1/10th of a mm)
	while((leftRunning || rightRunning)&&!exceedLimits){ 
		//Check if left reached target. If not, give half a pwm pulse
		if(l<num_pulses_L){
			gpioWrite(this->left.step_pin,1);
			l++;
			if(l % 128 == 0){
				this->left.manipMoved(128*distMultiplierL*distPerStepL);
			}
		}
		else{
			leftRunning = false;
		}
		
		//check if right reached target. if not, give half a pwm pulse	
		if(r<num_pulses_R){
			gpioWrite(this->right.step_pin,1);
			r++;
			if(r % 128 == 0){
				this->right.manipMoved(128*distMultiplierR*distPerStepR);
			}
		}
		else{
			rightRunning = false;
		}
		usleep(delayTime);
		
		//give the other half of the pwm pulse
		gpioWrite(this->left.step_pin,0);
		gpioWrite(this->right.step_pin,0);
		usleep(delayTime);
	
	}
	if(exceedLimits){
		printf("error %d\n",errorMode);
	}
}

/**
 * Recovers the tooling after a sensor tripped by issuing a movement override to clear the sensor.
 * @para mode : int : sensor code for which manipulator went too far in which direction
 *  -2: Left manipulator too far from center
 *  -1: Left manipulator too close to center
 *   1: Right manipulator too close to center
 *   2: Right manipulator too far from center
 */ 
void Tooling::recoverTooling(int mode){
	switch(mode){
		case -1:
			this->left.recoverManipulator(this->left.currentPosition-10,5);
			break;
		case -2:
			this->left.recoverManipulator(this->left.currentPosition+10,5);
			break;
		case 1:
			this->right.recoverManipulator(this->right.currentPosition-10,5);
			break;
		case 2:
			this->right.recoverManipulator(this->right.currentPosition+10,5);
			break;
		case 0:
			printf("some error occured\n");
			break;
	}
}
