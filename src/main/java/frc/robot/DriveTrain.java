/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


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

    private DriverStation driverStation;

    //declaring motors
    //one master and two slaves per side
    private CANSparkMax leftDriveMaster;
    private CANSparkMax leftDriveFront;
    private CANSparkMax leftDriveBack;
    private CANSparkMax rightDriveMaster;
    private CANSparkMax rightDriveFront;
    private CANSparkMax rightDriveBack;

    private CANEncoder leftDriveEncoder;
    private CANEncoder rightDriveEncoder;

    //gyro
    private AHRS gyro; 

    private final int leftDriveMasterCANid  = 1;
    private final int leftDriveFrontCANid   = 2;
    private final int leftDriveBackCANid    = 3;
    private final int rightDriveMasterCANid = 4;
    private final int rightDriveFrontCANid  = 5;
    private final int rightDriveBackCANid   = 6;
    
    private final double REDUCED_POWER_LEVEL  = .5;
    private final double BASELINE_POWER_LEVEL = .7;
    private final double EXTRA_POWER_LEVEL    = .9;

    private double axial  = 0;
    private double yaw    = 0;

    private double absoluteHeading = 0;
    private double robotHeading    = 0;
    private double targetHeading   = 0;
    private double robotHeadingModifier = 0;
    private boolean turning = false;

    //constructor
    public  DriveTrain () {
    }

    //initalize hardware for the drive train
    public void init(DriverStation driverStation){
        this.driverStation = driverStation;

        leftDriveMaster  = new CANSparkMax(leftDriveMasterCANid,  CANSparkMaxLowLevel.MotorType.kBrushless);
        leftDriveFront   = new CANSparkMax(leftDriveFrontCANid,   CANSparkMaxLowLevel.MotorType.kBrushless);
        leftDriveBack    = new CANSparkMax(leftDriveBackCANid,    CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveMaster = new CANSparkMax(rightDriveMasterCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveFront  = new CANSparkMax(rightDriveFrontCANid,  CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveBack   = new CANSparkMax(rightDriveBackCANid,   CANSparkMaxLowLevel.MotorType.kBrushless);

        leftDriveMaster.restoreFactoryDefaults();
        leftDriveFront.restoreFactoryDefaults();
        leftDriveBack.restoreFactoryDefaults();
        rightDriveMaster.restoreFactoryDefaults();
        rightDriveFront.restoreFactoryDefaults();
        rightDriveBack.restoreFactoryDefaults();

        //invert right motors so that positive values move the robot forward 
        rightDriveMaster.setInverted(true);
        rightDriveFront.setInverted(true);
        rightDriveBack.setInverted(true);

        //set encoders for drivetrain
        leftDriveEncoder  = leftDriveMaster.getEncoder();
        rightDriveEncoder = rightDriveMaster.getEncoder();

        try {
            gyro = new AHRS(SPI.Port.kMXP);
          } catch (RuntimeException ex ) {
            //insert report on driver station saying an error has occured
        }
    }

    //check to see if the robot is turning
    public void isTurning(){
        if (Math.abs(axial) > 0.05) {
            turning = true;
            targetHeading = robotHeading;
          }
          else {
            // Allow the robot to stop rotating and then lock in the heading.
            if (turning) {
              if (Math.abs(gyro.getRawGyroZ()) < 75) {
                targetHeading = robotHeading;
                turning = false;
              }
            }
        }
    }

    //use the controller values to set axial and yaw values
    public void setVectorsToController(){
        axial = driverStation.getLeftStickY();
        yaw   = driverStation.getRightStickX();
    }

    //return the currnet heading of the robot
    public void getAbsoluteHeading(){
        absoluteHeading = gyro.getAngle();
    }

    //update robot heading
    public void getRobotHeading(){
        robotHeading = angleWrap180(robotHeading + robotHeadingModifier);
    }

    //resets gyro and sets headings to zero
    public void resetHeading() {
        gyro.zeroYaw();
        robotHeading         = 0;
        targetHeading        = 0;
        robotHeadingModifier = 0;
    }

    //sets heading to input value
    public void setHeading(double newHeading){
        resetHeading();
        robotHeadingModifier = newHeading;
    }

    //use the target heading and robot heading to modify the yaw value to continue driving strait
    public void holdHeading(){

    }

    //use the curretn heading and target heading to turn the robot to the target heading
    public void turnToHeading(double tragetHeading){

    }

    //move the left and right motor position to target positions
    public void moveToPosition(double leftTarget, double rightTarget){

    }

    public double angleWrap180(double angle){
            while(angle <= -180){
                angle += 360;
            }
            while(angle >= 180){
                angle -=360;
            }
            return angle;
    }

    public void move(){
    //Driver 1 - left stick y drive forward/backward
    //Driver 1 - right stick x turn left/right
    //Driver 1 - (Button) Power mode
    //Driver 1 - (Button) Slow mode
    }

}
