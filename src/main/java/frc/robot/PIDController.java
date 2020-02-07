/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

// Public class to contain all the hardware elements (BotBits)
public class PIDController {

private double kp;
private double ki;
private double kd;
private double kf;

private double proportional;
private double integral;
private double derivative;

private double lastRPM           = 0;
private double lastRPMCorrection = 0;

private Timer elaspedTime;
private boolean firstTime = true;

    /**for shooter
     *  proportional    = .3
     *  integral        = .01
     *  derivative      = 0
     * forwardFeedInRPM = 6000
     **/

    // constructor
    public PIDController(double proportional, double integral, double derivative, double forwardFeedInRPM){
        kp  = proportional;
        ki  = integral;
        kd  = derivative;
        //kf is rpm at max power        
        kf  = forwardFeedInRPM;
        elaspedTime.start();
    }

    //inputs are in RPM
    //return is in power
    public double runPID(double currentRPM, double targetRMP){

        if(firstTime){
            elaspedTime.reset();
            firstTime = false;
            return targetRMP/kf;
        }

        double returnVal      = targetRMP/kf;
        double RPMCorrection  = targetRMP - currentRPM;

        proportional = RPMCorrection;
        integral     += RPMCorrection;
        derivative   = lastRPM - currentRPM;

        returnVal += (proportional * kp + integral * ki + derivative * kd)/kf;

        lastRPM           = currentRPM;
        lastRPMCorrection = RPMCorrection;
        elaspedTime.reset();

        return clip1(returnVal);
    }

    private double clip1(double input){
        if(input > 1){
            return 1;
        }else if(input < -1){
            return -1;
        }else{
            return input;
        }    
    }
}
