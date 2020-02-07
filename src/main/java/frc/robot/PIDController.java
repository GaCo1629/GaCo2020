/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Public class to contain all the hardware elements (BotBits)
public class PIDController {

private double kp;
private double ki;
private double kd;
private double kf;

private double proportional    = 0;
private double integral        = 0;
private double derivative      = 0;
private double feedForward     = 0;
private double runningIntegral = 0;

private double lastRPM           = 0;
private double lastRPMCorrection = 0;
private double returnVal         = 0;


private boolean firstTime = true;
private Timer elaspedTime;

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
        
        elaspedTime = new Timer();
        elaspedTime.start();
    }

    //inputs are in RPM
    //return is in power
    public double run(double currentRPM, double targetRMP){

        if(firstTime){
            elaspedTime.reset();
            firstTime = false;
            return targetRMP/kf;
        }

        double returnVal = 0;
        double RPMError  = targetRMP - currentRPM;

        proportional = RPMError               * kp;
        integral     = RPMError               * ki;
        derivative   = (lastRPM - currentRPM) * kd;
        feedForward  = (targetRMP/kf);

        runningIntegral += integral;
        runningIntegral = clip1(runningIntegral);

        returnVal = proportional + derivative + feedForward + runningIntegral;

        lastRPM           = currentRPM;
        lastRPMCorrection = RPMError;
        elaspedTime.reset();
        displayValues();

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

    private void displayValues(){
        SmartDashboard.putNumber("Proportional", proportional);
        SmartDashboard.putNumber("Integral", integral);
        SmartDashboard.putNumber("Derivative", derivative);
        SmartDashboard.putNumber("Feed Forward", feedForward);
        SmartDashboard.putNumber("Running Integral", runningIntegral);
        SmartDashboard.putNumber("return", returnVal);
    }
    
}
