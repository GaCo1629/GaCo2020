/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class fuelSystem {

    //constructor
    public  fuelSystem () {
    }

    //initalize fuel system 
    public void init(){
        //initalize Collector motor
        //initalize Collector Psunamatic
        //initalize Both Transfer Motors
        //initalize Both Shooter Motors
        //initalize Turret Spinner Motor
        //initalize Back Plate Motor?
        //initalize Front Limelight
        //initalize Back Limelight
    }

    //turn the turret to a given angle
    public void turnTurretTo(double angle){

    }

    //set the shooter to a given speed in RPM
    public void setShooterToSpeed(){

    }

    //use a pid loop to tune the target speed
    public void tuneShooterSpeedPID(){

    }

    //use the limelight to find the reflective tape
    public void findTarget(){

    }

    //use the target location to aim the turrett twords the target and set the shooter to the correct speed
    public void aimAtTarget(){

    }

    //use the reflective tape location and the heading of the shooter to find the robots location on the field
    public void getRobotLocation(){

    }

    //tun on the collector motors to on so that balls are sucked into the robot
    public void collectorIntake(){

    }

    //turn off the collector motors
    public void collectorOff(){

    }

    //tun on the collector motors to on so that balls are pushed out of the robot
    public void collectorOuttake(){

    }

    //use the psunamatics to put the collector down into the intaking position
    public void collectorDown(){

    }

    //use the psunamatics to put the collector up so it is inside the robot frame
    public void collectorUp(){

    }

    //use the back limelight to track balls outside the robot
    public void trackBalls(){

    }

    //run the transfer motors so that one ball is taken up into the transfer system
    public void indexFuel(){

    }

    //check to see if a ball is ready to be taken into the transfer system
    public void readyToIndex(){

    }

    //run inxed system in reverse until the last ball in the system is at the very bottom of the transfer mechanism
    public void resetIndexSystem(){

    }

    //run the transfer mechanism so that one ball is fired
    public void fireOne(){

    }

    //constatny run the transfer mechanism so that all balls are fired
    public void rapidFire(){

    }


    public void move(){
        //Driver 1 - (button/trigger) track and collect
        //Driver 1 (button) fire 1
        //Driver 1 (trigger) fire all
        //Driver 2 - (button) put down/up collector
        //Driver 2 - (button) collector on/off
        //Driver 2 (button) run storage system
    }

}
