#ifndef PanTilt_h
#define PanTilt_h

class PanTilt
{
    public:
        PanTilt();
        ~PanTilt();

        void init();

        //functions to set angles. return the angle so the user knows what was actually set
        float setPanAngle(float angle);
        float setTiltAngle(float angle);
        float incrementPanAngle(float delta); //new = old + delta
        float incrementTiltAngle(float delta);//new = old + delta

        void scan(); //move the arm around
        void centre(float& xPos, float& yPos); // move arm to centre face
        void resetCtrl(); //reset PI controller states

        void getAngles(float& anglePan, float& angleTilt); //use pointers- update both angles in one function

    private:
        int _pinPan;
        int _pinTilt;

        static void pwmStart();
        static void pwmStopPan();
        static void pwmStopTilt();
};

#endif