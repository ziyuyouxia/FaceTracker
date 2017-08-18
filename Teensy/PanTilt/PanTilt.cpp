#include <Arduino.h>
#include "PanTilt.h"
#include "DMAServo.h"

//pan: 0: pointing left, 180: pointing right
//tilt: 0: pointing up, 180: pointing down

const int pinPan = 2;
const int pinTilt = 3;
const float MIN_ANGLE_PAN = 45;
const float MAX_ANGLE_PAN = 135;
const float MIN_ANGLE_TILT = 60;
const float MAX_ANGLE_TILT = 80;
const float MIN_PWM_US = 500.0;
const float MAX_PWM_US = 2500.0;
const float SCALING_FACTOR_US_DEG = (MAX_PWM_US - MIN_PWM_US) / 180.0;

float anglePan = 0.5*(MIN_ANGLE_PAN+MAX_ANGLE_PAN);   //STATE: current pan angle
float angleTilt = 0.5*(MIN_ANGLE_TILT+MAX_ANGLE_TILT);   //STATE: current tilt angle
unsigned long prevTime;  //STATE: time at previous step (used to get dt)
float integralErrorPan;  //STATE: integral error for pan
float integralErrorTilt; //STATE: integral error for tilt

float usPan;  //the PWM width actually used for signal generation
float usTilt; //the PWM width actually used for signal generation

int panDirection = -1;  //STATE: 1: moving clockwise, -1 moving anticlockwise
int tiltDirection = -1; //STATE: 1: moving down, -1 moving up (confusing?)
float panErrPrev = 0, tiltErrPrev = 0; //for PID control
float xErrPrev = 0; //for control strategy

DMAServo servos;

PanTilt::PanTilt()
{
}

PanTilt::~PanTilt()
{
}

void PanTilt::init()
{
    //TODO: checks on pin numbers. E.g. need to keep i2c pins clear

    servos.init();

    setPanAngle(anglePan);
    setTiltAngle(angleTilt);

}

float PanTilt::setPanAngle(float ang)
{
    //set a new angle
    if (ang > MAX_ANGLE_PAN)
    {
        anglePan = MAX_ANGLE_PAN;
    }
    else if (ang < MIN_ANGLE_PAN)
    {
        anglePan = MIN_ANGLE_PAN;
    }
    else
    {
        anglePan = ang;
    }

    usPan = MIN_PWM_US + SCALING_FACTOR_US_DEG * anglePan;
    servos.writeMicroseconds(1,usPan);
    return anglePan;
}

float PanTilt::setTiltAngle(float ang)
{
    //set a new angle
    if (ang > MAX_ANGLE_TILT)
    {
        angleTilt = MAX_ANGLE_TILT;
    }
    else if (ang < MIN_ANGLE_TILT)
    {
        angleTilt = MIN_ANGLE_TILT;
    }
    else
    {
        angleTilt = ang;
    }

    usTilt = MIN_PWM_US + SCALING_FACTOR_US_DEG * angleTilt;
    servos.writeMicroseconds(2,usTilt);
    return angleTilt;
}

float PanTilt::incrementPanAngle(float delta)
{
    //add to the existing angle
    return setPanAngle(anglePan + delta);
}

float PanTilt::incrementTiltAngle(float delta)
{
    //add to the existing angle
    return setTiltAngle(angleTilt + delta);
}

void PanTilt::getAngles(float &pan, float &tilt)
{
    pan = anglePan;
    tilt = angleTilt;
}

void PanTilt::scan()
{
    const float deltaPanAngle = 20;
    const float deltaAngleTilt = 10;

    if ((anglePan >= MAX_ANGLE_PAN) && (panDirection == 1))
    {
        //reach end of clockwise scan: increment tilt and switch pan direction
        panDirection = -1;

        //check if the tile direction needs to change
        if ((angleTilt >= MAX_ANGLE_TILT) && (tiltDirection == 1))
        {
            tiltDirection = -1;
        }
        else if ((angleTilt <= MIN_ANGLE_TILT) && (tiltDirection == -1))
        {
            tiltDirection = 1;
        }
        incrementTiltAngle(deltaAngleTilt * (float)(tiltDirection));
    }

    else if ((anglePan <= MIN_ANGLE_PAN) && (panDirection == -1))
    {
        //reach end of anticlockwise scan: increment tilt and switch pan direction
        panDirection = 1;

        //check if the tile direction needs to change
        if (angleTilt >= MAX_ANGLE_TILT && tiltDirection == 1)
        {
            tiltDirection = -1;
        }
        else if ((angleTilt <= MIN_ANGLE_TILT) && (tiltDirection == -1))
        {
            tiltDirection = 1;
        }
        incrementTiltAngle(deltaAngleTilt * (float)(tiltDirection));
    }
    //increment the pan angle
    incrementPanAngle(deltaPanAngle * (float)(panDirection));
}

void PanTilt::centre(float &xPosition, float &yPosition)
{
    /*PID feedback law to move the arm so that the face appears in the middle
    Proportional name is a good start but will hunt.
    Integral is useful but has to be small: the delay in face detection can cause it to oscillate
    Derivative seems to work pretty well with high values
    */

    const float panPropGain = 10;
    const float panIntGain = 0.5;
    const float panDerGain = 5;
    const float tiltPropGain = 2.5;
    const float tiltIntGain = 0.2;
    const float tiltDerGain = 2;

    unsigned long newTime = micros();
    float timeDelta = (float)(newTime - prevTime) / 1000000.0;
    prevTime = newTime; //for next step

    float xErr = xPosition - 0.5;
    float yErr = yPosition - 0.5;
    float distance = sqrt(xErr * xErr + yErr * yErr); //control metric

    // //we were close, but the face moved so we have to start all over again...
    // if ((xErrPrev > -0.1 & xErrPrev < 0.1) & (xErr < -0.15 | xErrPrev > 0.15))
    // {
    //     resetCtrl();
    // }

    if(distance < 0.1) {
        //close enough
        Serial.print("Distance: ");
        Serial.println(distance);
        return;
    }
    
    float panErr = -xErr; //note sign convention
    float tiltErr = yErr;

    //adjust the angles according to the control law (remember these can return the new angles if needed)
    integralErrorPan = integralErrorPan + timeDelta * panErr;
    integralErrorTilt = integralErrorTilt + timeDelta * tiltErr;
    float derErrorPan = 0, derErrorTilt = 0;
    if(timeDelta > 0.05)
    {
        //generally the first time we enter the control loop, dt will be tiny so it blows up
        derErrorPan = panDerGain*(panErr - panErrPrev)/timeDelta;
        derErrorTilt = tiltDerGain*(tiltErr - tiltErrPrev)/timeDelta;
    }
    float panDelta = panPropGain * panErr + panIntGain * integralErrorPan + derErrorPan;
    float tiltDelta = tiltPropGain * tiltErr + tiltIntGain * integralErrorTilt + derErrorTilt;
    incrementPanAngle(panDelta);
    incrementTiltAngle(tiltDelta);
    Serial.print("dp: ");
    Serial.println(panDelta);

    panErrPrev = panErr;
}

void PanTilt::resetCtrl()
{
    prevTime = micros();
    integralErrorPan = 0.0;
    integralErrorTilt = 0.0;   
    panErrPrev = 0;
    xErrPrev = 0;
}
