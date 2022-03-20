#include "manipulator.h"

//ALL POSITIONS RELATIVE TO OUTSIDE

/**
 * Constructor for a manipulator object
 * 
 * @param a : int : pin number for stepper enablepin
 * @param b : int : pin number for direction pin
 * @param c : int : pin number for the step/pulse pin
 * @param d : bool : true if left manipulator
 */ 
Manipulator::Manipulator(int a, int b, int c, bool d):
	en_pin(a),
	dir_pin(b),
	step_pin(c),
	left(d)
	{
	}
 
/**
 * Sets the direction for a manipulator
 *
 * @para close : boolean : If true, set the manipulator to move towards center
 */ 
void Manipulator::setDirection(bool close){
	if(this->left){
		if(close){
			gpioWrite(this->dir_pin, 0);
		}
		else{
			gpioWrite(this->dir_pin, 1);
		}
	}
	else{
		if(close){
			gpioWrite(this->dir_pin, 0);
		}
		else{
			gpioWrite(this->dir_pin, 1);
		}
	}

}


/**
 * Enables a manipulator
 *
 * @para run : boolean. If true enable the manipulator, else disabled
 */ 
void Manipulator::setEnable(bool run){
	if(run){
		gpioWrite(this->en_pin, 1);
	}
	else{
		gpioWrite(this->en_pin, 0);
	}
}

/**
 * Moves a manipulator
 *
 * @para position : float : absolute position to move to relative to max open
 * @para speed : int : speed of manipulator in mm/s
 */ 
void Manipulator::moveManipulator(float position, int speed){
	//printf("error %d\n",errorMode);
	int distMultiplier;
	
	//if desired position more closed than current position, set direction closed
	if(position > this->currentPosition){
		distMultiplier = 1;
		this->setDirection(true);
	}
	else{
		distMultiplier = -1;
		this->setDirection(false);
	}
	float distance = position - this->currentPosition;
	
	//calculate the pulses to move from current position to new position
	int num_pulses = stepsPerMillimeter * abs(distance);
	int frequency = stepsPerMillimeter * speed;
	int microperiod = (1/frequency)*1000000; //period in microseconds
	int delayTime = microperiod/2;
	
	printf("moving from %f to %f\n",this->currentPosition, position);
	
	//move the manipulator the number of pulses necessary. update position periodically (every ~1/10th of a mm)
	this->setEnable(true);
	for(int i = 0;i<num_pulses&&!exceedLimits;i++){
		if(i % 128 == 0){
			this->manipMoved(128*distMultiplier*distPerStep);
		}
		gpioWrite(this->step_pin,1);
		usleep(delayTime);
		gpioWrite(this->step_pin,0);
		usleep(delayTime);
	}
}


/**
 * Recovery movement command. Basically same as moveManipulator, except it ignores any sensors.
 *  Used to back the manipulators off the sensors if tripped. Can be used any other time, but shouldnt due to the fact it ignores sensors... duh...
 *
 * @para position : float : absolute position to move to relative to max open
 * @para speed : int : speed of manipulator in mm/s
 */ 
void Manipulator::recoverManipulator(float position, int speed){
	int distMultiplier;
	
	//if desired position more closed than current position, set direction closed
	if(position > this->currentPosition){
		distMultiplier = 1;
		this->setDirection(true);
	}
	else{
		distMultiplier = -1;
		this->setDirection(false);
	}
	float distance = position - this->currentPosition;
	
	//calculate the pulses to move from current position to new position
	int num_pulses = stepsPerMillimeter * abs(distance);
	int frequency = stepsPerMillimeter * speed;
	int microperiod = (1/frequency)*1000000; //period in microseconds
	int delayTime = microperiod/2;
	
	printf("RECOVERY: moving from %f to %f\n",this->currentPosition, position);
	
	//move the manipulator the number of pulses necessary. update position periodically (every ~1/10th of a mm)
	this->setEnable(true);
	for(int i = 0;i<num_pulses;i++){
		if(i % 128 == 0){
			//this->currentPosition = this->currentPosition + 128*distMultiplier*distPerStep;
			this->manipMoved(128*distMultiplier*distPerStep);
			//printf("step %d pos %f dir %d\n", i, this->currentPosition, distMultiplier);
		}
		gpioWrite(this->step_pin,1);
		usleep(delayTime);
		gpioWrite(this->step_pin,0);
		usleep(delayTime);
	}
}


/**
 * Calibrates a manipulator by moving it until the max open sensor trips
 *  Note: closes the manipulator 5 mm before trying to find max open
 */ 
void Manipulator::calibrateManipulator(){
	int calibratePin;
	if(this->left){
		calibratePin = S_L_open;
	}
	else{
		calibratePin = S_R_open;
	}
	
	//close manipulator to clear sensor if already tripped
	this->moveManipulator(5,10);
	
	int frequency = 1280;
	int microperiod = (1/frequency)*1000000; //period in microseconds
	int delayTime = microperiod/2;

	//move the manipulator outwards at 1mm/s till it hits the sensor
	printf("Moving manipulator until sensor trip (Auto E Stop Disabled)\n");
	calibrating = true; //disable the auto E Stop
	this->setDirection(false);
	this->setEnable(true);
	while(!gpioRead(calibratePin)){
		gpioWrite(this->step_pin,1);
		usleep(delayTime);
		gpioWrite(this->step_pin,0);
		usleep(delayTime);
	}
	
	printf("Sensor Tripped\n");
	
	//close manipulator 10mm from the position at which it trips the outer sensor. Set this position to "0"
	// this gives us a safety margin so the manipulator is not constantly tripping when we send it to max open
	this->recoverManipulator(10,10);
	this->currentPosition = 0;
	usleep(100);
	calibrating = false; //reinable auto estop
	printf("Auto E Stop reinabled\n");
	
	
}


/**
 * Calculates the distance the manipulator can close from "0" and sets movement limits
 *  Note: this function is privledged and can ignore sensors...
 */ 
void Manipulator::calibrateCenter(){
	int calibratePin;
	if(this->left){
		calibratePin = S_L_close;
	}
	else{
		calibratePin = S_R_close;
	}

	//move the manipulator outwards at 1mm/s till it hits the sensor
	printf("checking center\n");
	checkCenter = true; //disable the auto E Stop
	this->setDirection(true);
	
	while(!gpioRead(calibratePin)){
		this->recoverManipulator((this->currentPosition + .5),1);
		usleep(100);
	}
	
	printf("Sensor Tripped\n");
	printf("set max\n");
	this->maxPosition = this->currentPosition;
	printf("move back\n");
	this->moveManipulator(115,10);
	usleep(200);
	checkCenter = false; //reinable auto estop
	printf("Auto E Stop reinabled\n");
}

