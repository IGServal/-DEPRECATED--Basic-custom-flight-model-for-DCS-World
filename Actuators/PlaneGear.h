#ifndef _PLANEGEAR_H_
#define _PLANEGEAR_H_

#include "../ED_FM_Utility.h"
#include <math.h>

class PlaneGearWheel
{
private:
    double yawPositionTarget = 0;
    double movingStep = 0.003;
public:
    double GearStatus = 0;
    int Steering = 0;
    double BrakeStatusMultiPlier = 0;
    double weightOnWheel = 0;
    double currentYaw = 0;

    void updateYawPosition(double inputRudder, double airspeed)
    {
        if (GearStatus >= 0.8 && Steering == 1 && airspeed < 40)
        {   
            yawPositionTarget = (- inputRudder) * (40 - airspeed)/40;
        }
        else
        {
            yawPositionTarget = 0;
        }
        
    }

    void updateCurrentYaw()
    {
        if (Steering == 1 && weightOnWheel > 0) // activate hydrolic
        {
            if (fabs(yawPositionTarget - currentYaw) < movingStep)
            {
                currentYaw = yawPositionTarget;
            }
            else
            {
                if (yawPositionTarget < currentYaw)
                {
                    currentYaw -= movingStep;
                }
                else
                {
                    currentYaw += movingStep;
                } 
            }
        }
        else if (yawPositionTarget == 0)
        {
            /* code */
            if (fabs(yawPositionTarget - currentYaw) < movingStep)
            {
                currentYaw = yawPositionTarget;
            }
            else
            {
                if (yawPositionTarget < currentYaw)
                {
                    currentYaw -= movingStep;
                }
                else
                {
                    currentYaw += movingStep;
                } 
            }
        }
        
    }
};

class PlaneGearSystem
{
private:
    /* data */

public:
    PlaneGearWheel nose;
    PlaneGearWheel left;
    PlaneGearWheel right;
    int CarrierPos = 0;

    void initial(int birth) // 0 for ground and 1 for air
    {   
        if (birth == 0)
        {
            nose.GearStatus = 1;
            left.GearStatus = 1;
            right.GearStatus = 1;
        }
        else
        {
            nose.GearStatus = 0;
            left.GearStatus = 0;
            right.GearStatus = 0;
        }
	    CarrierPos = 0;
    }

    void setWheelBrakes(double value)
    {
        nose.BrakeStatusMultiPlier = value;
        left.BrakeStatusMultiPlier = value;
        right.BrakeStatusMultiPlier = value;
    }
};

#endif