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
private double integralActiveZone;

private double proportional    = 0;
private double integral        = 0;
private double derivative      = 0;
private double feedForward     = 0;
private double runningIntegral = 0;

private double lastVal         = 0;
private double lastError       = 0;
private double returnVal       = 0;

private String name = "";


private boolean firstTime = true;
private Timer elaspedTime;

    /**for shooter
     * (.0005,.000001,.00005,5700,500)
     **/

    /**for turret
     * (180 degrees off then full power,?,0,0,5 degrees)
     **/

    // constructor
    public PIDController(double proportional, double integral, double derivative, double forwardFeedInRPM, double integralActiveZone, String name){
        kp  = proportional;
        ki  = integral;
        kd  = derivative;
        //kf for shooter is rpm at max power        
        kf  = forwardFeedInRPM;
        this.integralActiveZone = integralActiveZone;
        this.name = name;
    
    }

    //inputs are in RPM
    //return is in power
    public double run(double current, double target){

        if(firstTime){
            elaspedTime.reset();
            firstTime = false;
            return target/kf;
        }

        double returnVal = 0;
        double error     = target - current;

        proportional = error               * kp;
        integral     = error               * ki;
        derivative   = (lastVal - current) * kd;
        feedForward  = (target/kf);

        //if it is moving twords the target rpm set derviative to 0
        if(Math.abs(lastError) > Math.abs(error)){
            derivative = 0;
        }

        if(Math.abs(error) < integralActiveZone){
        runningIntegral += integral;
        runningIntegral = clip1(runningIntegral);
        } else {
            runningIntegral = 0;
        }

        returnVal = proportional + derivative + feedForward + runningIntegral;

        lastVal      = current;
        lastError    = error;

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
        SmartDashboard.putNumber(name + " Proportional", proportional);
        SmartDashboard.putNumber(name + " Integral", integral);
        SmartDashboard.putNumber(name + " Derivative", derivative);
        SmartDashboard.putNumber(name + " Feed Forward", feedForward);
        SmartDashboard.putNumber(name + " Running Integral", runningIntegral);
        SmartDashboard.putNumber(name + " return", returnVal);
    }
    
}
