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

public class DriveTrain extends Subsystem{

    /**CANids
    leftDriveMasterCANid  = 13
    leftDriveFrontCANid   = 12
    leftDriveBackCANid    = 14
    rightDriveMasterCANid = 16
    rightDriveFrontCANid  = 17
    rightDriveBackCANid   = 15
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

    private final int leftDriveMasterCANid  = 13;
    private final int leftDriveFrontCANid   = 12;
    private final int leftDriveBackCANid    = 14;
    private final int rightDriveMasterCANid = 16;
    private final int rightDriveFrontCANid  = 17; 
    private final int rightDriveBackCANid   = 15;
    
    private final double AXIAL_SLOW_POWER_LEVEL    = .2;
    private final double AXIAL_REGULAR_POWER_LEVEL = .3;
    private final double AXIAL_FAST_POWER_LEVEL    = .4;

    private final double YAW_SLOW_POWER_LEVEL    = .1;
    private final double YAW_REGULAR_POWER_LEVEL = .2;
    private final double YAW_FAST_POWER_LEVEL    = .3;

    private final double GYRO_SCALE           =  1.00;
    private final double HEADING_GAIN         = .01; //tweak this value to increase or decreasu auto correct power
    private final double MAX_AUTOCORRECT_YAW  = .5;

    private double axialPowerLevel = AXIAL_REGULAR_POWER_LEVEL;
    private double yawPowerLevel   = YAW_REGULAR_POWER_LEVEL;

    private double axial  = 0;
    private double yaw    = 0;

    private double absoluteHeading           = 0;
    private double robotHeading              = 0;
    private double targetHeading             = 0;
    private boolean autoHeading              = false;
    private double requiredHeadingCorrection = 0;
    private double robotHeadingModifier      = 0;
    private boolean turning = false;              //this always needs to start as false

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

        leftDriveFront.follow(leftDriveMaster);
        leftDriveBack.follow(leftDriveMaster);

        rightDriveBack.follow(rightDriveMaster);
        rightDriveFront.follow(rightDriveMaster);


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
            autoHeading = false;
        } else {
            // Allow the robot to stop rotating and then lock in the heading.
            if (turning) {
                if (Math.abs(gyro.getRawGyroZ()) < 75) {
                    targetHeading = robotHeading;
                    turning = false;
                    autoHeading = true;
                }
            }
        }
    }

    public void adjustPowerLevel(){
        if (driverStation.leftBumper()){
            axialPowerLevel = AXIAL_SLOW_POWER_LEVEL;
            yawPowerLevel   = YAW_SLOW_POWER_LEVEL;
        } else if (driverStation.rightBumper()){
            axialPowerLevel = AXIAL_FAST_POWER_LEVEL;
            yawPowerLevel   = YAW_FAST_POWER_LEVEL;
        } else {
            axialPowerLevel = AXIAL_REGULAR_POWER_LEVEL;
            yawPowerLevel   = YAW_REGULAR_POWER_LEVEL;
        }
    }

    //use the controller values to set axial and yaw values
    public void setVectorsToController(){
        axial = driverStation.getLeftStickY();
        yaw   = driverStation.getRightStickX();
    }

    //return the currnet heading of the robot
    public void getAbsoluteHeading(){
        absoluteHeading = gyro.getAngle() * GYRO_SCALE;
    }

    //update robot heading
    public void getRobotHeading(){
        getAbsoluteHeading();
        robotHeading = angleWrap180(absoluteHeading + robotHeadingModifier);
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
        if(autoHeading){
            requiredHeadingCorrection = angleWrap180(robotHeading - targetHeading);
            yaw = requiredHeadingCorrection * HEADING_GAIN;
        }
    }

    public void moveRobot(){
        adjustPowerLevel();
        setVectorsToController();
        //getRobotHeading();
        //isTurning();
        //holdHeading();

        yaw   *= yawPowerLevel;
        axial *= axialPowerLevel;

        double left  = axial + yaw;
        double right = axial - yaw;

        double max = Math.max(Math.abs(left), Math.abs(right));

        if(max > 1){
            left  /= max;
            right /= max;
        }

        leftDriveMaster.set(left);
        rightDriveMaster.set(right);
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

    public void adjustDrivePowerToAvoidTipping(double newAxial){

        if(Math.abs(newAxial - axial) >= .05){
            
        }
    }

    
    //use the curretn heading and target heading to turn the robot to the target heading
    public void turnToHeading(double tragetHeading){

    }

    //move the left and right motor position to target positions
    public void moveToPosition(double leftTarget, double rightTarget){

    }

    /**
     * left master  - dpad up
     * left front   - dpad right
     * left back    - dpad left
     * 
     * right master - y
     * right front  - a
     * right back   - x
     */
    public void driveMotorTest(){

    /*
        if(driverStation.dpadUp()){
            leftDriveMaster.set(.1);
        }else{
            leftDriveMaster.set(0);
        }

        if(driverStation.dpadLeft()){
            leftDriveBack.set(.1);
        }else{
            leftDriveBack.set(0);
        }

        if(driverStation.dpadRight()){
            leftDriveFront.set(.1);
        }else{
            leftDriveFront.set(0);
        }

        if(driverStation.x()){
            rightDriveBack.set(.1);
        }else{
            rightDriveBack.set(0);
        }

        if(driverStation.a()){
            rightDriveFront.set(.1);
        }else{
            rightDriveFront.set(0);
        }

        if(driverStation.y()){
            rightDriveMaster.set(.1);
        }else{
            rightDriveMaster.set(0);
        }
    */

    /*
        double left    = driverStation.getLeftStickY();
        double right   = driverStation.getRightStickY();

        rightDriveMaster.set(right);

        leftDriveMaster.set(left);
    */

    }

    @Override
    public void teleopPeriodic(){

        moveRobot();

    //Driver 1 - left stick y drive forward/backward
    //Driver 1 - right stick x turn left/right
    //Driver 1 - (Button) Power mode
    //Driver 1 - (Button) Slow mode
    }


}
