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

public class DriveTrain {

    DriverStation driverStation;

    final double REDUCED_POWER_LEVEL  = .6;
    final double BASELINE_POWER_LEVEL = .8;
    final double EXTRA_POWER_LEVEL    =  1;

    double axialPower;
    double yawPower;

    double robotHeading;
    double targetHeading;

    //constructor
    public  DriveTrain () {
    }

    //initalize hardware for the drive train
    public void init(DriverStation driverStation){
        this.driverStation = driverStation;

        //initialize 3 right side motors
        //initialize 3 right side motors
        //initialize gyro
    }

    //check to see if the robot is turning
    public void isTurning(){

    }

    //return the currnet heading of the robot
    public void getRobotHeading(){

    }

    //use the controller values to set axial and yaw values
    public void driveRobot(){

    }

    //use the target heading and robot heading to modify the yaw value to continue driving strait
    public void autoCorrect(){

    }

    //use the curretn heading and target heading to turn the robot to the target heading
    public void turnToHeading(double tragetHeading){

    }

    //move the left and right motor position to target positions
    public void moveToPosition(double leftTarget, double rightTarget){

    }

    public void move(){
    //Driver 1 - left stick y drive forward/backward
    //Driver 1 - right stick x turn left/right
    //Driver 1 - (Button) Power mode
    //Driver 1 - (Button) Slow mode
    }

}
