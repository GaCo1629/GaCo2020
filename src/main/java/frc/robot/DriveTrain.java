/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel;

import javax.swing.text.StyledEditorKit.BoldAction;

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

    private GaCoDrive     pilot;
    private GaCoDrive     copilot;
    private GaCoDrive     minion;

    private Vision        turretVision;
    private FuelSystem    fuelSystem;

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
    private final int LEFT_DRIVE_MASTER_CAN_ID  = 13;
    private final int LEFT_DRIVE_FRONT_CAN_ID   = 12;
    private final int LEFT_DRIVE_BACK_CAN_ID    = 14;
    private final int RIGHT_DRIVE_MASTER_CAN_ID = 16;
    private final int RIGHT_DRIVE_FRONT_CAN_ID  = 17; 
    private final int RIGHT_DRIVE_BACK_CAN_ID   = 15;
    
    //set final power levels for modifing power levels  
    private final double AXIAL_SLOW_POWER_LEVEL    = 0.2;
    private final double YAW_SLOW_POWER_LEVEL      = 0.1;

    private final double AXIAL_REGULAR_POWER_LEVEL = 0.3;
    private final double YAW_REGULAR_POWER_LEVEL   = 0.2;

    private final double AXIAL_FAST_POWER_LEVEL    = 0.6;
    private final double YAW_FAST_POWER_LEVEL      = 0.4;

    //set gyro final variables
    private final double GYRO_SCALE                           = 1.00;
    private final double HEADING_GAIN                         = 0.05; //tweak this value to increase or decreasu auto correct power
    private final double MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE = 0.75;
    private final double DEGREES_TOLERANCE                    = 1.5;

    //encoder inches per tick
    private final double INCHES_PER_REV = 0.39671806;
    double lastDistanceTraveled = 0;
    boolean robotIsMoving = false;
    final double MAX_STOPPED_DISTANCE = 0.1; // Must move less inches in a cycle to be considered stopped.
    double distanceTarget = 0;
	boolean distanceCorrect = true;
	double headingTarget = 0;
	boolean headingCorrect = true;
	double drivePower = 0;
	double timeout = 0;

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
    private double targetHeading             = 0;
    private boolean autoHeading              = false;
    private double requiredHeadingCorrection = 0;
    private double robotHeadingModifier      = 0;
    private boolean turning                  = false; 

    private double lastHeading = 0;


    //declare variables for tracting robot location
    public double x             = 0;
    public double y             = 0;
    public double robotHeading  = 0;

    private double test1;

    public Point robotLocation = new Point(0,0,0);

    private double turretHeadingFieldCentric;

    //proportional, integral, derivative, forwardFeedInRPM, integralActiveZone, tolerance, angleWrapOn, name
    PIDController headingPID = new PIDController(.05, 0, 0, 0, 5, 1, true, "Gyro");

    //constructor
    public  DriveTrain () {
    }

    //initalize hardware for the drive train
    public void init(GaCoDrive pilot, GaCoDrive copilot, GaCoDrive minion, Vision turretVision, FuelSystem fuelSystem){
        this.pilot      = pilot;
        this.copilot    = copilot;
        this.minion     = minion;
        this.turretVision   = turretVision;
        this.fuelSystem     = fuelSystem;

        leftDriveMaster  = new CANSparkMax(LEFT_DRIVE_MASTER_CAN_ID,  CANSparkMaxLowLevel.MotorType.kBrushless);
        leftDriveFront   = new CANSparkMax(LEFT_DRIVE_FRONT_CAN_ID,   CANSparkMaxLowLevel.MotorType.kBrushless);
        leftDriveBack    = new CANSparkMax(LEFT_DRIVE_BACK_CAN_ID,    CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveMaster = new CANSparkMax(RIGHT_DRIVE_MASTER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveFront  = new CANSparkMax(RIGHT_DRIVE_FRONT_CAN_ID,  CANSparkMaxLowLevel.MotorType.kBrushless);
        rightDriveBack   = new CANSparkMax(RIGHT_DRIVE_BACK_CAN_ID,   CANSparkMaxLowLevel.MotorType.kBrushless);

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
            yaw = headingPID.run(robotHeading, targetHeading);
        }
    }
    

    //use the controller values to set axial and yaw values
    public void setVectorsToController( double a, double y){
        axial = a; //axial
        yaw   = y; //yaw
    }
        
    public void adjustPowerLevel(){
        if (pilot.leftBumper()){
            axialPowerLevel = AXIAL_SLOW_POWER_LEVEL;
            yawPowerLevel   = YAW_SLOW_POWER_LEVEL;
        } else if (pilot.rightBumper()){
            axialPowerLevel = AXIAL_FAST_POWER_LEVEL;
            yawPowerLevel   = YAW_FAST_POWER_LEVEL;
        } else {
            axialPowerLevel = AXIAL_REGULAR_POWER_LEVEL;
            yawPowerLevel   = YAW_REGULAR_POWER_LEVEL;
        }
    }

    public void updateVariables(){
        //update drive encoders
        leftEncoder  = leftDriveEncoder.getPosition();
        rightEncoder = rightDriveEncoder.getPosition();

        //update robot heading
        robotHeading = angleWrap180((gyro.getAngle() * GYRO_SCALE) + robotHeadingModifier);

        //update robot location if limelight is visible
        if(turretVision.targetVisible){
            turretHeadingFieldCentric = angleWrap180(robotHeading + fuelSystem.getTurretHeading());
            x = turretVision.getDistanceFromTarget() * Math.cos(turretHeadingFieldCentric);
            y = turretVision.getDistanceFromTarget() * Math.sin(turretHeadingFieldCentric);
            robotLocation.set(x, y, robotHeading);
        }
    }

    public double getHeadingChange(){
        test1 = gyro.getRate()*(180/3.14);
        return test1;
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
                axial = lastAxial + Math.signum(deltaPower) * deltaTime * MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE;
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

    public void moveRobot(double a, double y){
        adjustPowerLevel();
        setVectorsToController(a , y );
        //runHoldHeading();
        calculateAndSetMotorPowers();
    }

    @Override
    public void teleopPeriodic(){

        moveRobot(pilot.getLeftStickY(), pilot.getRightStickX());
        show();

    //Driver 1 - left stick y drive forward/backward
    //Driver 1 - right stick x turn left/right
    //Driver 1 - right bumper Power mode
    //Driver 1 - left bumper Slow mode
    }

    @Override
    public void show() {
        SmartDashboard.putNumber ("Robot X", x);
        SmartDashboard.putNumber ("Robot Y", y);
        SmartDashboard.putNumber ("Heading", robotHeading);
        SmartDashboard.putNumber ("Axial", axial);
        SmartDashboard.putNumber ("Yaw", yaw);
        SmartDashboard.putBoolean("Truning", turning);
        SmartDashboard.putBoolean("Auto Heading On?", autoHeading);
        SmartDashboard.putNumber ("Target Heading", targetHeading);
        SmartDashboard.putNumber ("Left Encoder Position", leftEncoder);
        SmartDashboard.putNumber ("Right Encoder Position", rightEncoder);
        SmartDashboard.putNumber ("Heading Change RPM", getHeadingChange());

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


        ////// using code from 2018 to make auto functions\\\\\\
    public double encoderToInches( double inches){
       return inches*INCHES_PER_REV;
    }



    // Get the average distance traveled by each wheel
	double inchesTraveled() {

		double distanceTravelled = encoderToInches((Math.abs(leftDriveEncoder.getPosition()) + Math.abs(rightDriveEncoder.getPosition()))/2);

		// Wait for a short time before really checking for moving condition.
		robotIsMoving = ((timer.get() < 0.5)
				|| (Math.abs(distanceTravelled - lastDistanceTraveled) > MAX_STOPPED_DISTANCE));

		lastDistanceTraveled = distanceTravelled; // Save last movement distance for next time around the loop

		return (distanceTravelled);
    }
    // Reset both Encoders & timer
	void resetEncoders() {
		lastDistanceTraveled = -1; // pre-load with invalid value
		leftDriveEncoder.setPosition(0);
		rightDriveEncoder.setPosition(0);
		timer.reset();
	}

    
	void driveRobot(double inches, double fwd,  double timeoutS) {
      //  if (driveFirstTime){
      //  resetEncoders();
      //  }

	    if (inchesTraveled()<inches && timer.get()<timeoutS){
            moveRobot(fwd, 0);
           // driveFirstTime = false;
        }	
		
	}
}
