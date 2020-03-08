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

    private GaCoDrive     pilot;
    private GaCoDrive     copilot;
    private GaCoDrive     minion;
    private Controller    controller;

    private Vision        turretVision;
    private FuelSystem    fuelSystem;
    private Auto          auto;

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
    private final double YAW_SLOW_POWER_LEVEL      = 0.2;

    private final double AXIAL_REGULAR_POWER_LEVEL = 0.5;
    private final double YAW_REGULAR_POWER_LEVEL   = 0.4;

    private final double AXIAL_FAST_POWER_LEVEL    = 0.7;
    private final double YAW_FAST_POWER_LEVEL      = 0.4;


    //set gyro final variables
    private final double GYRO_SCALE                           = 1.00;
    private final double HEADING_GAIN                         = 0.05; //tweak this value to increase or decreasu auto correct power
    private final double MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE = 0.75;
    private final double DEGREES_TOLERANCE                    = 1.5;

    //encoder inches per tick
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
    
  public final double  AXIAL_SCALE          =  0.7;
  public final double  YAW_SCALE            =  0.5;

    //declare variables for tracting robot location
    public double x             = 0;
    public double y             = 0;
    public double robotHeading  = 0;

    private double test1;

    public Point robotLocation = new Point(0,0,0);
    private double turretHeadingFieldCentric;

    //  General Variables
  public boolean targetLocked; //
  public double currentHeading;  // Current Gyro heading
  public double currentPitch;  // Current Gyro pitch.  +ve = front up
  public double headingLock;     // Current locked heading (from Gyro)
  public double axialInches = 0; // used to track motion
  
  public double vectorInches = 0;
  public double maxDriveCurrent = 0;
  public double yawDegrees = 0;
  //  Driving variables
  private double driveAxial;      // Power for fwd/rev driving
  
  private double driveYaw;        // Power for rotating
 
  private boolean prototypeRobot = false;
  double LF1  = 0;
  double RF1  = 0;
  
  private final double MIN_HEADING_ERROR = 3;
  private final double MAX_YAW_POWER = 0.5;  // was .5
  private final double VRAMP = 0.03;  //  full speed in 33 inches was .024

  public final double  SAFE_POWER           =  1.0; //was .9

    //proportional, integral, derivative, forwardFeedInRPM, integralActiveZone, tolerance, angleWrapOn, name
    PIDController headingPID = new PIDController(.05, 0, 0, 0, 5, 1, true, "Gyro");

    //constructor
    public  DriveTrain () {
    }

    //initalize hardware for the drive train
    public void init(GaCoDrive pilot, GaCoDrive copilot, GaCoDrive minion, Controller controller, Vision turretVision, FuelSystem fuelSystem, Auto auto){
        this.pilot      = pilot;
        this.copilot    = copilot;
        this.minion     = minion;
        this.controller = controller;
        this.turretVision   = turretVision;
        this.fuelSystem     = fuelSystem;
        this.auto = auto;
    

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

    // =============================================================
    //  TELEOP METHODS
    // =============================================================

    @Override
    public void teleopInit() {
        //reset all gyro and auto heading variables
        turning                   = false;
        autoHeading               = false;
        requiredHeadingCorrection = 0;
        robotHeadingModifier      = 0;
        timer.reset();
     
    }

    @Override
    public void teleopPeriodic(){
        adjustPowerLevel();
         //reduce axial and yaw according to power level
        axial = pilot.getLeftStickY()  * axialPowerLevel;
        yaw   = pilot.getRightStickX() * yawPowerLevel;
       
        calculateAndSetMotorPowers(axial, yaw);
        show();
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

    public void calculateAndSetMotorPowers( double anAxial, double aYaw){

        //adjust axial to avoid tipping
        double deltaTime  = timer.get() - lastTime;
        double deltaPower = anAxial - lastAxial;

        if(deltaTime != 0){
            if(Math.abs(deltaPower/deltaTime) > MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE){
                anAxial = lastAxial + Math.signum(deltaPower) * deltaTime * MAXIUM_AXIAL_POWER_PER_SECOND_CHANGE;
            }
        }
    
        lastTime = timer.get();
        lastAxial = anAxial;

        //calculate left and right motor powers
        double left  = anAxial + aYaw;
        double right = anAxial - aYaw;

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

    // =============================================================
    // Auto METHODS
    // =============================================================

    
    // =============================================================
    //  GENERAL DRIVE METHODS
    // =============================================================

    //sets heading to input value
    public void setHeading(double newHeading){
        robotHeadingModifier -= robotHeading;
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
        if(controller.resetRobotHeading){
          setHeading(0);
        }
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
  public void turnToHeading(double heading, double timeout) {
    double endTime = timer.get() + timeout;

 

    while (/*Subsystem.isEnabled() &&*/  (timer.get() <= endTime)) {

    //  periodic();
      driveRobot(0.0, getYawPower(heading));

      // exit loop if we are close enough
      if (Math.abs(heading - getHeading()) <= MIN_HEADING_ERROR)
        break;
    }
    stopRobot();
  }
  public void stopRobot() {
    driveRobot(0,0);
    headingLock = getHeading();
  }

  
  // applies the desired power to the wheels.
  public void driveRobot(double ax,  double yaw) {
    driveAxial = ax;
    driveYaw = yaw;
    driveRobot();
  }
   
  public void driveRobot() {
    driveRobot(0);
  }

  public void driveRobot(double gyroHeading) {
    //robot.driveTank(driveAxial, driveYaw, gyroHeading);
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
  // Drive a distance in a direction, while rotating robot to finalHeading.
  public  boolean driveVector(double vectorDistance, double maxSpeed, double direction, double finalHeading, double timeout){
    double endTime = timer.get() + timeout;
    double vectorPower = 0;

    //limit speed for now
    maxSpeed *= SAFE_POWER;


    resetMotion(); //remember where we started
    while (/*myRobot.isEnabled() &&*/ (timer.get() <= endTime)) {

    //  periodic();  // get latest robot data

      driveYaw = clip(getYawPower(finalHeading), 0.2);  // slow turn rate
      vectorPower = getPowerProfile(vectorDistance, maxSpeed, vectorInches, VRAMP);
      
      driveAxial = vectorPower * Math.cos(direction * (Math.PI / 180.0));

      driveRobot(-currentHeading);
      
      if(vectorPower == 0){
        break;
      }
    }

    // Stop the robot
    stopRobot();

    return (timer.get() <= endTime);  // return true if we reached the target before timing out
  }
  // adjust power to provide smooth acc/decell curves
  private double getPowerProfile(double dTotal, double pTop, double dMotion, double vRamp) {
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
  }

  public void tankDrive( double ax, double ya, double inches, double time){
    resetEncoders();
    axial= ax;
    yaw = ya;
    double encoders = (Math.abs(leftDriveEncoder.getPosition()) + Math.abs(rightDriveEncoder.getPosition()))/2;
    while(auto.timeOut.get()<time && encoders < encoderToInches(inches)){
      calculateAndSetMotorPowers();
      encoders = (Math.abs(leftDriveEncoder.getPosition()) + Math.abs(rightDriveEncoder.getPosition()))/2;
    }
    stopRobot();
  }
  
  

}
