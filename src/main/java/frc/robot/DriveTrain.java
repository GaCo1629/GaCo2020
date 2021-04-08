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

  // Declare Hardware Interfaces

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
  private AHRS gyro;

  //declare motor can IDs
  private final int LEFT_DRIVE_MASTER_CAN_ID  = 13;
  private final int LEFT_DRIVE_FRONT_CAN_ID   = 12;
  private final int LEFT_DRIVE_BACK_CAN_ID    = 14;
  private final int RIGHT_DRIVE_MASTER_CAN_ID = 16;
  private final int RIGHT_DRIVE_FRONT_CAN_ID  = 17;
  private final int RIGHT_DRIVE_BACK_CAN_ID   = 15;

  //set final power levels for modifing power levels
  private final double AXIAL_SLOW_POWER_LEVEL    = 0.3;
  private final double YAW_SLOW_POWER_LEVEL      = 0.2;

  private final double AXIAL_REGULAR_POWER_LEVEL = 0.5;
  private final double YAW_REGULAR_POWER_LEVEL   = 0.25;

  private final double AXIAL_FAST_POWER_LEVEL    = 0.7;
  private final double YAW_FAST_POWER_LEVEL      = 0.35;

  private final double GYRO_SCALE                = 1.01528;

  //set gyro final variables
  private final double MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE = 0.9; // was.75

  //driving constants
  private final double R2KSLOPE               = -0.0391; //Derived from tests
  private final double R2KOFFSET              = 0.522; //Derived from tests
  private final double FEET_PER_AXIAL_REV     = 0.205; //Derived from tests

  double  lastDistanceTraveled = 0;
  boolean robotIsMoving = false;
  final   double MAX_STOPPED_DISTANCE = 0.1; // Must move less inches in a cycle to be considered stopped.
  double  distanceTarget = 0;
  boolean distanceCorrect = true;
  double  headingTarget = 0;
  boolean headingCorrect = true;
  double  drivePower = 0;
  double  timeout = 0;

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
  private double headingLock               = 0;     // used for manual driving
  private boolean autoHeading              = false; // enable heading hold
  private boolean turning                  = false; // still slowing down after turn
  private double  robotTurnRate            = 0;

  public final double  AXIAL_SCALE          =  0.7;
  public final double  YAW_SCALE            =  0.5;

  //declare variables for tracting robot location
  public double x             = 0;
  public double y             = 0;
  public double robotHeading  = 0;

  public Point robotLocation  = new Point(0,0,0);

  // Autonomous variables
  private ArrayList<Step> path  = null;
  private boolean followingPath = false;
  private int currentIndex      = 0;
  private Timer stepTime;
  private double stepTraveled   = 0;
  private double leftEncoderStart   = 0;
  private double rightEncoderStart  = 0;
  private double headingStart = 0;

  //  General Variables
  public boolean targetLocked;   //
  public double axialInches = 0; // used to track motion

  public double vectorInches = 0;
  public double maxDriveCurrent = 0;
  public double yawDegrees = 0;

  double LF1  = 0;
  double RF1  = 0;

  //proportional, integral, derivative, forwardFeedInRPM, integralActiveZone, tolerance, angleWrapOn, name
  PIDController headingPID = new PIDController(.008, 0.0007, 0, 0, 3, 1, true, "Gyro", 0.15);

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
    timer.reset();
  }

  @Override
  public void teleopPeriodic(){

    // look for manual heading reset
    if(controller.resetRobotHeading){
      resetHeading();
    }

    //reduce axial and yaw joystick input according to power level
    adjustPowerLevel();
    axialDrive = limitAcceleration(controller.joystickForward * axialPowerLevel);
    yawDrive   = controller.joystickTurn * yawPowerLevel;

    // implement heading lock if driving straight
    runHoldHeading();

    // send power commands to wheels
    moveRobot();
  }

  public void adjustPowerLevel(){
    if (controller.slowMode){
      axialPowerLevel = AXIAL_SLOW_POWER_LEVEL;
      yawPowerLevel   = YAW_SLOW_POWER_LEVEL;
    } else if (controller.powerMode){
      axialPowerLevel = AXIAL_FAST_POWER_LEVEL;
      yawPowerLevel   = YAW_FAST_POWER_LEVEL;
    } else {
      axialPowerLevel = AXIAL_REGULAR_POWER_LEVEL;
      yawPowerLevel   = YAW_REGULAR_POWER_LEVEL;
    }
  }

  //use the target heading and robot heading to modify the yaw value to continue driving strait
  public void runHoldHeading(){
    if (Math.abs(yawDrive) > 0.05) {
      turning = true;
      headingLock = robotHeading;
      autoHeading = false;
    } else {
      // Allow the robot to stop rotating and then lock in the heading.
      if (turning) {
        if (Math.abs(robotTurnRate) < 2) {
          headingLock = robotHeading;
          turning = false;
          autoHeading = true;
        }
      }
    }

    if(autoHeading){
      yawDrive = headingPID.run(robotHeading, headingLock);
    }
  }

  // =============================================================
  // Auto METHODS
  // =============================================================

  @Override
  public void autonomousInit() {
    turning                   = false;
    autoHeading               = false;
    path                      = null;
    followingPath             = false;
    currentIndex              = 0;
    timer.reset();
    resetHeading();
    startMotion();
  }

  @Override
  public void autonomousPeriodic(){
    //This is where we will follow the path
    if (followingPath){
      Step currentStep = path.get(currentIndex);
      switch (currentStep.mode){
        case E_STOP:
          stopRobot();
          nextStep();
          break;

        case BRAKE:
          axialDrive = limitAcceleration(0);
          yawDrive = headingPID.run(robotHeading, currentStep.heading);
          moveRobot();
          if (Math.abs(axialDrive) <= 0.001){
            stopRobot();
            nextStep();
          }
          break;

        case DRIVE_STRAIGHT:
          if (stepTime.get() >= currentStep.timeout){
            nextStep();
          } else if ((currentStep.distance != 0) && (Math.abs(stepTraveled) >= currentStep.distance)) {
            nextStep();
          } else {
            axialDrive = limitAcceleration(currentStep.speed);
            yawDrive = headingPID.run(robotHeading, currentStep.heading);
            moveRobot();
          }
          break;

        case TURN_TO_HEADING:
          if (stepTime.get() >= currentStep.timeout){
            nextStep();
          } else if (Math.abs(currentStep.heading - robotHeading) < 1){
            nextStep();
          } else {
            axialDrive = limitAcceleration(0);
            yawDrive = headingPID.run(robotHeading, currentStep.heading);
            moveRobot();
          }
          break;

        case SWEEP_FOR_DISTANCE:
          if (stepTime.get() >= currentStep.timeout){
            nextStep();
          } else if ((currentStep.distance != 0) && (Math.abs(stepTraveled) >= currentStep.distance)) {
            nextStep();
          } else {
			      axialDrive = limitAcceleration(currentStep.speed);
            yawDrive = getKFactor(currentStep.radius) * axialDrive;
            moveRobot();
          }
          break;

        case SWEEP_TO_HEADING:
          double headingError = normalizeHeading(currentStep.heading - robotHeading);
          if (stepTime.get() >= currentStep.timeout){
            nextStep();
          } else if ((Math.abs(headingError) < 45) &&
                     (Math.signum(headingError) != Math.signum(currentStep.radius))
                    ){  //Stop if close to target and have crossed over the target heading
            nextStep();
          } else {
            //Calculate turn rate based on radius and current limited speed
			      axialDrive = limitAcceleration(currentStep.speed);
            yawDrive   = getKFactor(currentStep.radius) * axialDrive;
            if (Math.abs(headingError) < 10){
              yawDrive *= (Math.abs(headingError) / 15);
            }
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
    startMotion();
  }

  //Go to next step unless at end
  public void nextStep(){
    if (path.size() - 1 == currentIndex){
      followingPath = false;
      stopRobot();
    } else {
      currentIndex++;
      stepTime.reset();
      startMotion();
    }
  }

  public double getKFactor(double radius){
    return Math.signum(radius) * ((R2KSLOPE * Math.abs(radius)) + R2KOFFSET);
  }

  // =============================================================
  //  GENERAL DRIVE METHODS
  // =============================================================

  public void readSensors(){
    //update drive encoders
    leftEncoder  = leftDriveEncoder.getPosition();
    rightEncoder = rightDriveEncoder.getPosition();

    //update distance traveled
    double leftWheelTraveled = leftEncoder - leftEncoderStart;
    double rightWheelTraveled = rightEncoder - rightEncoderStart;
    double avgTraveled = (leftWheelTraveled + rightWheelTraveled)/2;
    stepTraveled = avgTraveled * FEET_PER_AXIAL_REV;

    //update robot heading
    robotHeading = getHeading();
    robotTurnRate = gyro.getRawGyroZ();
  }

  public double inchesToFeet(double inches){
    return inches/12;
  }

  public void startMotion(){
    leftEncoderStart = leftEncoder;
    rightEncoderStart = rightEncoder;
    stepTraveled = 0;
    headingStart = robotHeading;
  }

  //return the corrected angle of the gyro
  public double getHeading(){
    return  (gyro.getAngle() * GYRO_SCALE);
  }

  //resets gyro and sets headings to zero
  public void resetHeading() {
    gyro.zeroYaw();
    headingLock = 0;
    robotHeading = 0;
  }

  public void limitAcceleration(double axial, double yaw){
    axialDrive = limitAcceleration(axial);
    yawDrive = yaw;
  }

  public double limitAcceleration(double reqAxial){

    double thisTime = timer.get();

    //adjust axial to avoid tipping
    double deltaTime  = thisTime - lastTime;
    double deltaPower = reqAxial - lastAxial;
    double limitedAxial = reqAxial;

    if((deltaTime != 0) && (Math.abs(deltaPower/deltaTime) > MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE)){
        limitedAxial = lastAxial + (Math.signum(deltaPower) * deltaTime * MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE);
    }

    lastTime = thisTime;
    lastAxial = limitedAxial;

    return(limitedAxial);
  }

  public void stopRobot() {
    moveRobot(0,0);
    headingLock = getHeading();
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
    SmartDashboard.putNumber ("Target Heading", headingLock);
    SmartDashboard.putNumber ("Left Encoder Position", leftEncoder);
    SmartDashboard.putNumber ("Robot Turn Rate", robotTurnRate);
    SmartDashboard.putNumber ("Right Encoder Position", rightEncoder);
    SmartDashboard.putBoolean("Following Path", followingPath);
    SmartDashboard.putNumber("Current Step Index", currentIndex);
    SmartDashboard.putNumber("Step Traveled", stepTraveled);

    SmartDashboard.putNumber ("Left Velocity", leftDriveEncoder.getVelocity());
    SmartDashboard.putNumber ("Right Velocity", rightDriveEncoder.getVelocity());

  }

  public double normalizeHeading(double angle){
    while(angle <= -180){
        angle += 360;
    }
    while(angle >= 180){
        angle -=360;
    }
    return angle;
  }

  /*

  // Reset both Encoders & timer
	void resetEncoders() {
		leftDriveEncoder.setPosition(0);
		rightDriveEncoder.setPosition(0);
  }

  // resest the measurment for any motion profile
  private void resetMotion() {
    LF1 = leftDriveEncoder.getPosition();
    RF1= rightDriveEncoder.getPosition();

    axialInches = 0;
  }

*/

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
