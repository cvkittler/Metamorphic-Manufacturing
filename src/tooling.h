#include "manipulator.cpp"

class Tooling{
    public:
        Manipulator left;
        Manipulator right;
        float leftPos;
        float rightPos;
        bool eStop = false;
        
        Tooling(Manipulator l, Manipulator r);
        

		void calibrateTooling();
		void moveAManipulator(float position, int speed, bool side);
		void moveManipulators(float positionL, float positionR, int speed);
		void recoverTooling(int mode);
		void stop();
        

};
