/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
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

    //declare encoders for drive train
    private CANEncoder leftDriveEncoder;
    private CANEncoder rightDriveEncoder;

    private double leftEncoder;
    private double rightEncoder;

    //gyro
    private AHRS gyro; 

    //declare motor can IDs
    private final int leftDriveMasterCANid  = 13;
    private final int leftDriveFrontCANid   = 12;
    private final int leftDriveBackCANid    = 14;
    private final int rightDriveMasterCANid = 16;
    private final int rightDriveFrontCANid  = 17; 
    private final int rightDriveBackCANid   = 15;
    
    //set final power levels for modifing power levels  
    private final double AXIAL_SLOW_POWER_LEVEL    = 0.2;
    private final double AXIAL_REGULAR_POWER_LEVEL = 0.2;
    private final double AXIAL_FAST_POWER_LEVEL    = 1.0;

    private final double YAW_SLOW_POWER_LEVEL    = 0.1;
    private final double YAW_REGULAR_POWER_LEVEL = 0.2;
    private final double YAW_FAST_POWER_LEVEL    = 0.4;

    //set gyro final variables
    private final double GYRO_SCALE                           = 1.00;
    private final double HEADING_GAIN                         = 0.05; //tweak this value to increase or decreasu auto correct power
    private final double MAX_AUTOCORRECT_YAW                  = 0.5;
    private final double MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE = 0.75;
    private final double DEGREES_TOLERANCE                    = 1.5;

    //set default axial and yaw power level to the regular power level
    private double axialPowerLevel = AXIAL_REGULAR_POWER_LEVEL;
    private double yawPowerLevel   = YAW_REGULAR_POWER_LEVEL;

    //declare drive power variables
    private double axial     = 0;
    private double yaw       = 0;
    private double lastAxial = 0;

    //declare variables for gyro and heading correction
    private Timer timer;
    private double lastTime                  = 0;
    private double robotHeading              = 0;
    private double targetHeading             = 0;
    private boolean autoHeading              = false;
    private double requiredHeadingCorrection = 0;
    private double robotHeadingModifier      = 0;
    private boolean turning                  = false; 

    //declare variables for tracting robot location
    private double x;
    private double y;
    private double turretHeadingFieldCentric;

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

        //set front and back motors on each side to follow the center master motor
        leftDriveFront.follow(leftDriveMaster);
        leftDriveBack.follow(leftDriveMaster);

        rightDriveBack.follow(rightDriveMaster);
        rightDriveFront.follow(rightDriveMaster);


        //set encoders for drivetrain
        leftDriveEncoder  = leftDriveMaster.getEncoder();
        rightDriveEncoder = rightDriveMaster.getEncoder();

        //try to initalize the gyro 
        try {
            gyro = new AHRS(SPI.Port.kMXP);
            gyro.zeroYaw();
            SmartDashboard.putString("Gyro Calibration", "Sucessful");
        } catch (RuntimeException ex ) {
            SmartDashboard.putString("Gyro Calibration", "Failed");
        }

        //reset all gyro and auto heading variables
        turning                   = false;
        robotHeading              = 0;
        targetHeading             = 0;
        autoHeading               = false;
        requiredHeadingCorrection = 0;
        robotHeadingModifier      = 0;

        PIDController headingPID = new PIDController(.05, 0, 0, 0, 5, 1, true, "Gyro");
        
        //start timer thats used to adjust axial inputs
        timer = new Timer();
        timer.start();
    }

    //sets heading to input value
    public void setHeading(double newHeading){
        gyro.zeroYaw();
        robotHeading         = newHeading;
        targetHeading        = newHeading;
        robotHeadingModifier = newHeading;
    }    

    //called every cycle in robot periodic
    public double updateRobotHeading(){
        robotHeading = angleWrap180((gyro.getAngle() * GYRO_SCALE) + robotHeadingModifier);
        return robotHeading;
    }

    //use the target heading and robot heading to modify the yaw value to continue driving strait
    public void runHoldHeading(){
        if (Math.abs(yaw) > 0.05) {
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

        if(autoHeading){
            requiredHeadingCorrection = angleWrap180(targetHeading - robotHeading);
            if(Math.abs(requiredHeadingCorrection) > DEGREES_TOLERANCE){
                yaw = requiredHeadingCorrection * HEADING_GAIN;
            }else {
                yaw = 0;
            }
        }
    }
    
    //use the controller values to set axial and yaw values
    public void setVectorsToController(){
        axial = driverStation.getLeftStickY();
        yaw   = driverStation.getRightStickX();
    }
        
    public void adjustPowerLevel(){
        //free up left bumper for jaylen
        /*if (driverStation.leftBumper()){
            axialPowerLevel = AXIAL_SLOW_POWER_LEVEL;
            yawPowerLevel   = YAW_SLOW_POWER_LEVEL;
        }
        */ 
        if (driverStation.rightBumper()){
            axialPowerLevel = AXIAL_FAST_POWER_LEVEL;
            yawPowerLevel   = YAW_FAST_POWER_LEVEL;
        } else {
            axialPowerLevel = AXIAL_REGULAR_POWER_LEVEL;
            yawPowerLevel   = YAW_REGULAR_POWER_LEVEL;
        }
    }

    public void updateDriveEncoders(){
        leftEncoder  = leftDriveEncoder.getPosition();
        rightEncoder = rightDriveEncoder.getPosition();
    }

    //use the reflective tape location and the heading of the shooter to find the robots location on the field
    /**Uses
     * distance to target (in feet)
     * heading of robot   (in degrees)
     * heading of turret  (in degrees)
     */
    public void getRobotLocation(double distanceToTarget, double rHeading, double tHeading){
        distanceToTarget = 12;
        rHeading         = 0;
        tHeading         = 0;

        turretHeadingFieldCentric = angleWrap180(rHeading + tHeading);

        x = distanceToTarget * Math.cos(turretHeadingFieldCentric);
        y = distanceToTarget * Math.sin(turretHeadingFieldCentric);

    }

    public void calculateAndSetMotorPowers(){
        //reduce axial and yaw according to power level
        yaw   *= yawPowerLevel;
        axial *= axialPowerLevel;

        //adjust axial to avoid tipping
        double deltaTime  = timer.get() - lastTime;
        double deltaPower = axial - lastAxial;

        if(deltaTime != 0){
            if(Math.abs(deltaPower/deltaTime) > MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE){
                axial = lastAxial + Math.signum(deltaPower)*deltaTime*MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE;
            }
        }
    
        lastTime = timer.get();
        lastAxial = axial;

        //calculate left and right motor powers
        double left  = axial + yaw;
        double right = axial - yaw;

        //scale them down so that the maxium of left/right is equal to one
        double max = Math.max(Math.abs(left), Math.abs(right));

        if(max > 1){
            left  /= max;
            right /= max;
        }

        //set motors to calculated values
        leftDriveMaster.set(left);
        rightDriveMaster.set(right);
    }

    public void moveRobot(){
        adjustPowerLevel();
        setVectorsToController();
        runHoldHeading();
        calculateAndSetMotorPowers();
    }

    //move the left and right motor position to target positions
    public void moveToPosition(double leftTarget, double rightTarget){

    }

    @Override
    public void teleopPeriodic(){

        moveRobot();
        show();

    //Driver 1 - left stick y drive forward/backward
    //Driver 1 - right stick x turn left/right
    //Driver 1 - (Button) Power mode
    //Driver 1 - (Button) Slow mode
    }

    @Override
    public void show() {
        SmartDashboard.putNumber("Axial", axial);
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Heading", robotHeading);
        SmartDashboard.putBoolean("Truning", turning);
        SmartDashboard.putBoolean("Auto Heading On?", autoHeading);
        SmartDashboard.putNumber("Target Heading", targetHeading);
        SmartDashboard.putNumber("Left Encoder Position", leftEncoder);
        SmartDashboard.putNumber("Right Encoder Position", rightEncoder);
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
}
