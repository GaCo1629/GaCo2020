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

    /**CANids
    *leftDriveMasterCANid  = 1;
    *leftDriveFrontCANid   = 2;
    *leftDriveBackCANid    = 3;
    *rightDriveMasterCANid = 4;
    *rightDriveFrontCANid  = 5;
    *rightDriveBackCANid   = 6;
    */

    DriverStation driverStation;

    //declaring motors
    //one master and two slaves per side
    private CANSparkMax leftDriveMaster;
    private CANSparkMax leftDriveFront;
    private CANSparkMax leftDriveBack;
    private CANSparkMax rightDriveMaster;
    private CANSparkMax rightDriveFront;
    private CANSparkMax rightDriveBack;

    private final int leftDriveMasterCANid  = 1;
    private final int leftDriveFrontCANid   = 2;
    private final int leftDriveBackCANid    = 3;
    private final int rightDriveMasterCANid = 4;
    private final int rightDriveFrontCANid  = 5;
    private final int rightDriveBackCANid   = 6;
    
    private final double REDUCED_POWER_LEVEL  = .6;
    private final double BASELINE_POWER_LEVEL = .8;
    private final double EXTRA_POWER_LEVEL    =  1;

    private double axialPower;
    private double yawPower;

    private double robotHeading;
    private double targetHeading;

    //constructor
    public  DriveTrain () {
    }

    //initalize hardware for the drive train
    public void init(DriverStation driverStation){
        this.driverStation = driverStation;

        leftDriveMaster  = new CANSparkMax(leftDriveMasterCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftDriveFront   = new CANSparkMax(leftDriveFrontCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftDriveBack    = new CANSparkMax(leftDriveBackCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveMaster = new CANSparkMax(rightDriveMasterCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveFront  = new CANSparkMax(rightDriveFrontCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveBack   = new CANSparkMax(rightDriveBackCANid, CANSparkMaxLowLevel.MotorType.kBrushless);

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
