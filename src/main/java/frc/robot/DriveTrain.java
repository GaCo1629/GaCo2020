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
import java.util.*;

public class DriveTrain extends Subsystem{

  /**CANids
  leftDriveMasterCANid  = 13
  leftDriveFrontCANid   = 12
  leftDriveBackCANid    = 14
  rightDriveMasterCANid = 16
  rightDriveFrontCANid  = 17
  rightDriveBackCANid   = 15
  */

  private Controller    controller;

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

  public double leftEncoder;
  public double rightEncoder;

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
  private final double YAW_SLOW_POWER_LEVEL      = 0.2;

  private final double AXIAL_REGULAR_POWER_LEVEL = 0.5;
  private final double YAW_REGULAR_POWER_LEVEL   = 0.3;

  private final double AXIAL_FAST_POWER_LEVEL    = 0.7;
  private final double YAW_FAST_POWER_LEVEL      = 0.4;


  //set gyro final variables
  private final double GYRO_SCALE                           = 1.00;
  private final double HEADING_GAIN                         = 0.05; //tweak this value to increase or decreasu auto correct power
  private final double MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE = 0.20; // was.75

  //encoder inches per tic
  private final double INCHES_PER_AXIAL_REV = 0.39671806;
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
  private double axialDrive     = 0;
  private double yawDrive       = 0;
  private double lastAxial = 0;

  //declare variables for gyro and heading correction
  private Timer timer;
  private double lastTime                  = 0;
  private double targetHeading             = 0;
  private boolean autoHeading              = false;
  private double robotHeadingModifier      = 0;
  private boolean turning                  = false; 
    
  public final double  AXIAL_SCALE          =  0.7;
  public final double  YAW_SCALE            =  0.5;

  //declare variables for tracting robot location
  public double x             = 0;
  public double y             = 0;
  public double robotHeading  = 0;

  private double test1;

  public Point robotLocation = new Point(0,0,0);
  private double turretHeadingFieldCentric;

    // Autonomous variables
  private ArrayList<Step> path  = null;
  private boolean followingPath = false;
  private int currentIndex      = 0;
  private Timer stepTime;


    //  General Variables
  public boolean targetLocked; //
  public double currentHeading;  // Current Gyro heading
  public double currentPitch;  // Current Gyro pitch.  +ve = front up
  public double headingLock;     // Current locked heading (from Gyro)
  public double axialInches = 0; // used to track motion
  
  public double vectorInches = 0;
  public double maxDriveCurrent = 0;
  public double yawDegrees = 0;

  double LF1  = 0;
  double RF1  = 0;
  
  private final double MIN_HEADING_ERROR = 3;
  private final double MAX_YAW_POWER = 0.5;  // was .5
  


  //Used in getPowerProfile()
  //private final double VRAMP = 0.03;  //  full speed in 33 inches was .024

  public final double  SAFE_POWER           =  1.0; //was .9

  //proportional, integral, derivative, forwardFeedInRPM, integralActiveZone, tolerance, angleWrapOn, name
  PIDController headingPID = new PIDController(.01, 0.0005, 0, 0, 3, 1, true, "Gyro");

  //constructor
  public  DriveTrain () {
  }

  //initalize hardware for the drive train
  public void init(Controller controller, Vision turretVision, FuelSystem fuelSystem){

    this.controller = controller;
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

    //start timer thats used to adjust axial inputs
    timer = new Timer();
    timer.start();

    //start timer thats used to monitor step progress
    stepTime = new Timer();
    stepTime.start();
  }

  // =============================================================
  //  TELEOP METHODS
  // =============================================================

  @Override
  public void teleopInit() {
    //reset all gyro and auto heading variables
    turning                   = false;
    autoHeading               = false;
    robotHeadingModifier      = 0;
    timer.reset();
  }

  @Override
  public void autonomousInit() {
    turning                   = false;
    autoHeading               = false;
    robotHeadingModifier      = 0;
    path                      = null;
    followingPath             = false;
    currentIndex              = 0;
    timer.reset();      
    super.autonomousInit();
  }

  @Override
  public void teleopPeriodic(){
    
    if(controller.resetRobotHeading){
      setHeading(0);
    }
    
    //reduce axial and yaw joystick input according to power level
    adjustPowerLevel();
    axialDrive = controller.joystickForward * axialPowerLevel;
    yawDrive   = controller.joystickTurn * yawPowerLevel;

    limitAcceleration();
    moveRobot();
  }

  public void adjustPowerLevel(){
    if (controller.powerMode){
      axialPowerLevel = AXIAL_SLOW_POWER_LEVEL;
      yawPowerLevel   = YAW_SLOW_POWER_LEVEL;
    } else if (controller.slowMode){
      axialPowerLevel = AXIAL_FAST_POWER_LEVEL;
      yawPowerLevel   = YAW_FAST_POWER_LEVEL;
    } else {
      axialPowerLevel = AXIAL_REGULAR_POWER_LEVEL;
      yawPowerLevel   = YAW_REGULAR_POWER_LEVEL;
    }
  }

 
  // =============================================================
  // Auto METHODS
  // =============================================================
  //Will work eventually
  @Override
  public void autonomousPeriodic(){
    //This is where we will follow the path
    if (followingPath){
      Step currentStep = path.get(currentIndex);
      switch (currentStep.mode){
        case STOP:
        stopRobot();
        nextStep();
        break;

        case STRAIGHT:
        if (stepTime.get() >= currentStep.timeout){
          nextStep();
        } else {
          limitAcceleration(currentStep.speed, 0);
          yawDrive = headingPID.run(robotHeading, currentStep.heading);
          moveRobot();
        }
        break;

        default:
        break;
      }
    }
  }

  //Set up path
  public void setPath(ArrayList<Step> initPath){
    path          = initPath;
    followingPath = true;
    currentIndex   = 0;
    stepTime.reset();
  }

  //Go to next step unless at end
  public void nextStep(){
    if (path.size() - 1 == currentIndex){
      followingPath = false;
      stopRobot();
    } else {
      currentIndex++;
      stepTime.reset();
    }
  }
  
  // =============================================================
  //  GENERAL DRIVE METHODS
  // =============================================================

  //sets heading to input value
  public void setHeading(double newHeading){
    robotHeadingModifier -= robotHeading;
  }    

  //use the target heading and robot heading to modify the yaw value to continue driving strait
  public void runHoldHeading(){
    if (Math.abs(yawDrive) > 0.05) {
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
      yawDrive = headingPID.run(robotHeading, targetHeading);
    }
  }
  
  public void readSensors(){
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

  public void limitAcceleration(double axial, double yaw){
    axialDrive = axial;
    yawDrive = yaw;
    limitAcceleration();
  }

  public void limitAcceleration(){
      
    double tempTime = timer.get();

    //adjust axial to avoid tipping
    double deltaTime  = tempTime - lastTime;
    double deltaPower = axialDrive - lastAxial;

    if(deltaTime != 0){
      if(Math.abs(deltaPower/deltaTime) > MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE){
        axialDrive = lastAxial + (Math.signum(deltaPower) * deltaTime * MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE);
      }
    }

    lastTime = tempTime;
    lastAxial = axialDrive;
  }

  public void moveRobot(double axial, double yaw) {
    axialDrive = axial;
    yawDrive = yaw;
    moveRobot();
  }
  
  public void moveRobot(){

   //calculate left and right motor powers
   double left  = axialDrive + yawDrive;
   double right = axialDrive - yawDrive;

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

  @Override
  public void show() {
      SmartDashboard.putNumber ("Robot X", x);
      SmartDashboard.putNumber ("Robot Y", y);
      SmartDashboard.putNumber ("Heading", robotHeading);
      SmartDashboard.putNumber ("Axial", axialDrive);
      SmartDashboard.putNumber ("Yaw", yawDrive);
      SmartDashboard.putBoolean("Truning", turning);
      SmartDashboard.putBoolean("Auto Heading On?", autoHeading);
      SmartDashboard.putNumber ("Target Heading", targetHeading);
      SmartDashboard.putNumber ("Left Encoder Position", leftEncoder);
      SmartDashboard.putNumber ("Right Encoder Position", rightEncoder);
      SmartDashboard.putNumber ("Heading Change RPM", getHeadingChange());
      SmartDashboard.putBoolean("Following Path", followingPath);
      SmartDashboard.putNumber("Current Step Index", currentIndex);

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

 
  ////// using code from 2018 to make auto functions\\\\\\
  public double encoderToInches( double inches){
    return inches*INCHES_PER_AXIAL_REV;
  }

  
  // Reset both Encoders & timer
	void resetEncoders() {
		leftDriveEncoder.setPosition(0);
		rightDriveEncoder.setPosition(0);
  }
  
  //turns to the correct angle
  //not currently used
  public void turnToHeading(double heading, double timeout) {
    double endTime = timer.get() + timeout;

    while (/*Subsystem.isEnabled() &&*/  (timer.get() <= endTime)) {

      readSensors(); 
      moveRobot(0.0, getYawPower(heading));

      // exit loop if we are close enough
      if (Math.abs(heading - getHeading()) <= MIN_HEADING_ERROR)
        break;
    }
    stopRobot();
  }
  
  public void stopRobot() {
    moveRobot(0,0);
    headingLock = getHeading();
  }

  

  //calculate yaw power based on disired heading
  private double getYawPower(double desired) {
    double output = (normalizeHeading(desired - getHeading())) * HEADING_GAIN;
    return (clip(output, MAX_YAW_POWER));
  }

  //resets gyro and sets headings to zero
  public void resetHeading() {
    gyro.zeroYaw();
    headingLock = 0;
    currentHeading = 0;
    resetMotion();
  }

  //return the corrected angle of the gyro
  public double getHeading(){
    currentHeading = gyro.getAngle() * GYRO_SCALE ;
    return  (currentHeading);
  }  //return the corrected angle of the gyro

  public double lockHeading(){
    headingLock = currentHeading;
    return  (headingLock);
  }

  // resest the measurment for any motion profile
  private void resetMotion() {
    LF1 = leftDriveEncoder.getPosition();
   RF1= rightDriveEncoder.getPosition();
    
    axialInches = 0;
    yawDegrees = 0;
  }
  // Provide an easy way to limit values to +/- limit
  public double clip(double raw, double limit) {
    if (raw > limit)
      return limit;
    else if (raw < -limit)
      return -limit;
    else
      return raw;
  }
    
	
  public double normalizeHeading(double heading) {
    while (heading > 180)
      heading -= 360;
    while (heading < -180)
      heading += 360;

      return heading;
  }


  // adjust power to provide smooth acc/decell curves
  /*private double getPowerProfile(double dTotal, double pTop, double dMotion, double vRamp) {
    double sign = Math.signum(dTotal);
    double dAcc = pTop / VRAMP;
    double udTotal = Math.abs(dTotal);
    double dConst = udTotal - dAcc;
    double udMotion = Math.abs(dMotion);
    double reqPower = 0;

    // Check to see if top speed can even be reached in time.
    if (dAcc > (udTotal / 2)) {
      dAcc = udTotal / 2;
      dConst = dAcc;
    }

    if (udMotion < dAcc)
      reqPower = 0.1 + (udMotion * vRamp);  // Add a little bit to get started
    else if (udMotion < dConst )
      reqPower = pTop;
    else if (udMotion < udTotal)
      reqPower = 0.1 + ((udTotal - udMotion) * vRamp);
    else
      reqPower = 0;

    reqPower = clip(reqPower, pTop);

    return (reqPower * sign);
  }*/
  
}
