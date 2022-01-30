#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <pigpio.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>

#define PIN_SYSTEM BCM

#define EN_L 5
#define DIR_L 6
#define STEP_L 13
#define EN_R 14
#define DIR_R 15
#define STEP_R 18

#define S_L_close 17
#define S_L_open 27
#define S_R_close 22
#define S_R_open 10

#define ES_INPUT 9

extern bool calibrating;
extern bool volatile exceedLimits;

class Manipulator {
    public:
        int en_pin;
        int dir_pin;
        int step_pin;
		bool left; //true if left manipulator else false
        
        int distPerRotation = 5; //mm
        int stepsPerRotation = 6400;
        
        float distPerStep = float(distPerRotation)/float(stepsPerRotation);
        int stepsPerMillimeter = stepsPerRotation/distPerRotation;
        
        float currentPosition = 0;    //mm
        float minPosition = 0;        //mm
        float maxPosition = 200;      //mm
		
		void setDirection(bool close);
		void setEnable(bool run);
		void moveManipulator(float position, int speed);
		void recoverManipulator(float position, int speed);
		void calibrateManipulator();
		void manipMoved(float p){
			currentPosition = currentPosition + p;
		}
        
		Manipulator(int a, int b, int c, bool d);
        
};
