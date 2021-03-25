/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Public class to contain all the hardware elements (BotBits)
public class PIDController {

private double kp;
private double ki;
private double kd;
private double kf;
private double integralActiveZone;
private boolean angleWrapOn = false;
private double outputLimit; 

private double proportional         = 0;
private double integral             = 0;
private double derivative           = 0;
private double feedForward          = 0;
private double runningIntegral      = 0;
private double tolerance            = 0;

private double lastVal              = 0;
private double lastError            = 0;
private double returnVal            = 0;

private String name = "";


private boolean firstTime = true;

    /**for shooter
     * (.0005,.000001,.00005,5700,500)
     **/

    /**for turret
     * (180 degrees off then full power,?,0,0,5 degrees)
     **/

    // constructor
    public PIDController(double proportional, double integral, double derivative, double forwardFeedInRPM, double integralActiveZone, double tolerance, boolean angleWrapOn, String name, double limit){
        kp  = proportional;
        ki  = integral;
        kd  = derivative;
        //kf for shooter is rpm at max power        
        kf  = forwardFeedInRPM;
        this.integralActiveZone = integralActiveZone;
        this.name = name;
        this.tolerance = tolerance;
        this.angleWrapOn = angleWrapOn;
        outputLimit = limit;
    
    }

    //inputs are in RPM
    //return is in power
    public double run(double current, double target){

        if(firstTime){
            firstTime = false;
            return target*kf;
        }

        returnVal = 0;
        double error = target - current;

        if(angleWrapOn){
            error = angleWrap180(error);
        }

        proportional = error               * (kp);
        integral     = error               * ki;
        derivative   = (lastVal - current) * kd;
        feedForward  = (target*kf);


        if(Math.abs(error) < integralActiveZone){
        runningIntegral += integral;
        runningIntegral = clip(runningIntegral);
        } else {
            runningIntegral = 0;
        }

        returnVal = proportional + derivative + feedForward + runningIntegral;

        lastVal              = current;
        lastError            = error;

        //If this is not motor speed control
        if((kf != 0) && (Math.abs(error) < tolerance)){
            returnVal = 0;
        }

        returnVal = clip(returnVal);
        displayValues();
        
        return returnVal;
    }

    private double clip(double input){
        if(input > outputLimit){
            return outputLimit;
        }else if(input < -outputLimit){
            return -outputLimit;
        }else{
            return input;
        }    
    }

    private void displayValues(){
        SmartDashboard.putNumber(name + " Proportional", proportional);
        SmartDashboard.putNumber(name + " Integral", integral);
        SmartDashboard.putNumber(name + " Derivative", derivative);
        SmartDashboard.putNumber(name + " Feed Forward", feedForward);
        SmartDashboard.putNumber(name + " Running Integral", runningIntegral);
        SmartDashboard.putNumber(name + " return", returnVal);
    }

    private double angleWrap180(double angle){
        while(angle <= -180){
            angle += 360;
        }
        while(angle >= 180){
            angle -=360;
        }
        return angle;
    }
    
}
