/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
upper transfer 10
lower transfer 11
collector 12
left shooter 20
right shooter 21
turret 22
collectorleft 0,1
collector right 0,1
*/

public class FuelSystem extends Subsystem {

  private GaCoDrive pilot;
  private GaCoDrive copilot;
  private GaCoDrive minion;

  private boolean copilotYLastState = false;
  private boolean minionLastAState  = false;

  private boolean visionEnabled     = true;

  private Vision        turretVision;
  private DriveTrain    driveTrain;

  private VictorSP lowerTransfer;
  private VictorSP upperTransfer;
  private VictorSP collector;

  private CANSparkMax turret;
  private CANSparkMax leftShooter; 
  private CANSparkMax rightShooter;

  private CANEncoder shooterEncoder;
  private CANEncoder turretEncoder;

  private DoubleSolenoid lowerCollector;

  private DigitalInput lowerBallDetector;
  private DigitalInput upperBallDetector;

  private boolean lastUpperBallDetectorState = true;
  private boolean lastLowerBallDetectorState = true;

  private boolean upperBallDetected     = true;
  private boolean lowerBallDetected     = true;

  private final double COLLECTOR_SPEED = 1.0;

  private final double TURRET_REVS_PER_DEGREE   = 1.27866;
  private final double MIN_DISTANCE_TO_TARGET   = 10;
  private final double MAX_DISTANCE_TO_TARGET   = 40;
  private final double MAX_SHOOTER_RPM          = 5700;
  private final double MIN_SHOOTER_RPM          = 2000;
  private final double MAX_TURRET_ANGLE         = 100;
  private final double TURRET_PROPORTIONAL_GAIN = .002;
  private final double TURRET_INTEGRAL_GAIN     = 0;

  private boolean collectorSolinoidDown         = false;

  private final double SHOOTER_RPM_TOLERANCE    = 75;
  private final double TURRET_DEGREE_TOLERANCE  = .5;

  private static final int L_SHOOTER_ID  = 21;
  private static final int R_SHOOTER_ID  = 20;
  private static final int TURRET_ID     = 10;
  private static final int COLLECTOR_ID  = 1;
  private static final int U_TRANSFER_ID = 3;
  private static final int L_TRANSFER_ID = 2;

  private boolean shooterPIDEnabled = false;
  private double targetSpeedRPM     = 0;
  private double shooterRPM         = 0;

  private double turretHeading         = 0;
  private double targetTurretHeading   = 0;
  private boolean turretPIDEnabled     = false;
  private double turretHeadingModifier = 0;
  private double ballsInIndex          = 0;

  private boolean prepairToFireFlag = false;
  private boolean fireOneFlag       = false;

  public boolean readyToShoot = false;
  private boolean runCollector = false;
  private boolean reverseCollector = false;
  private boolean collectorIsDown = false;
  private boolean collectorIsRunning = false;

  private double tempRPM;

  private Timer timer = new Timer();  

  //proportional, integral, derivative, forwardFeedInRPM, integralActiveZone, tolerance, angleWrapOn, name
  PIDController shooterPID = new PIDController(.0003,.000001,.0005,5700,500, 0, false, "Shooter");
  PIDController turretPID  = new PIDController(.04, .001, 0, 0, 10, .05, false, "Turret");

  //constructor
  public FuelSystem () {
        
  }

  //initalize fuel system 
  public void init(GaCoDrive pilot, GaCoDrive copilot, GaCoDrive minion, Vision turretVision, DriveTrain driveTrain){
    this.pilot   = pilot;
    this.copilot = copilot;
    this.minion  = minion;
    this.turretVision   = turretVision;
    this.driveTrain     = driveTrain;

    //initialize motors
    leftShooter  = new CANSparkMax(L_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightShooter = new CANSparkMax(R_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    turret       = new CANSparkMax(TURRET_ID,    CANSparkMaxLowLevel.MotorType.kBrushless);

    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
      
    //invert left motor so that positive values shoots the balls out 
    leftShooter.setInverted(true);
      
    // Encoder object created to display position/velocity values
    shooterEncoder = leftShooter.getEncoder(); 
    turretEncoder  = turret.getEncoder();

    //Victor SP motor initialization
    upperTransfer = new VictorSP(U_TRANSFER_ID);
    lowerTransfer = new VictorSP(L_TRANSFER_ID);
    collector     = new VictorSP(COLLECTOR_ID);

    lowerBallDetector = new DigitalInput(0);
    upperBallDetector = new DigitalInput(1);

    //invert collector so that positive values move balls up twords the shooter
    lowerTransfer.setInverted(true);
    turret.setInverted(true);

    lowerCollector = new DoubleSolenoid(1,1,0);

    //reset turret heading variables
    turretHeading       = 0;
    targetTurretHeading = 0;
    turretPIDEnabled    = true;

    //reset shooter RPM variables
    shooterRPM          = 0;
    targetSpeedRPM      = 2300;
    shooterPIDEnabled   = true;

    lowerCollector.set(DoubleSolenoid.Value.kReverse);

    readyToShoot = false;

    timer.start();    
    timer.reset();
  }
  
  @Override
  public void teleopInit() {

  }

  // =========================================================
  // Turret Control
  // =========================================================
  
  //turn the turret to a given angle
  public void turnTurretTo(double targetAngle){

    if(targetAngle > MAX_TURRET_ANGLE){
      targetAngle = MAX_TURRET_ANGLE;
    }

    if(targetAngle < -MAX_TURRET_ANGLE){
      targetAngle = -MAX_TURRET_ANGLE;
    }

    targetTurretHeading = targetAngle;
    turret.set(turretPID.run(turretHeading, targetTurretHeading));
  }

  //sets the shooter to the correct spid based on target or min or max
  public void setShooterRPM(double targetRPM){

    if(targetRPM > MAX_SHOOTER_RPM){
      targetRPM = MAX_SHOOTER_RPM;
    }

    if(targetRPM < MIN_SHOOTER_RPM){
      targetRPM = MIN_SHOOTER_RPM;
    }

    targetSpeedRPM = targetRPM;
    setShooterSpeed(shooterPID.run(shooterRPM, targetRPM));
  }

  //clips a value based on range
  public double clip(double val, double range){
    if(val > range){
      val = range;
    } 

    if(val < -range){
      val = -range;
    }

    return val;
  }

  //makes solinoid out and down 
  public void collectorUpDown(){
    // lower collector
    if (minion.dpadUp() || collectorIsRunning){
        lowerCollector.set(DoubleSolenoid.Value.kForward);
        collectorIsDown = true;

        // if manually lowered, reset timer
        //if (gaCoDrive2.dpadUp()) {
          timer.reset();
        //}

    } else if (minion.dpadDown() || (!collectorIsRunning && timer.get() > 3)){
        lowerCollector.set(DoubleSolenoid.Value.kReverse);
        collectorIsDown = false;
    }
  }

  public void updateVariables(){
    shooterRPM = shooterEncoder.getVelocity();
    turretHeading = turretEncoder.getPosition()/TURRET_REVS_PER_DEGREE + turretHeadingModifier;
    lastLowerBallDetectorState = lowerBallDetected;
    lastUpperBallDetectorState = upperBallDetected;

    //inverse detector states so that true indicates that a ball has been detected
    lowerBallDetected = !lowerBallDetector.get();
    upperBallDetected = !upperBallDetector.get();

    /** say its ready to fire if
     * target is visible
     * shooter RPM is within tolerance
     * the distance to the target is within tolerance
     * the angle to the target is within tolerance
     *  */
    if(turretVision.targetVisible && Math.abs(shooterRPM - targetSpeedRPM) < SHOOTER_RPM_TOLERANCE &&
     turretVision.getDistanceFromTarget() > MIN_DISTANCE_TO_TARGET &&
     turretVision.getDistanceFromTarget() < MAX_DISTANCE_TO_TARGET &&
     Math.abs(turretVision.x) < TURRET_DEGREE_TOLERANCE){
      readyToShoot = true;
    } else {
      readyToShoot = false;
    }

    //check to see if the lower state did show a ball and is now not showing a ball and if it is add one 
    if(!lowerBallDetected && lastLowerBallDetectorState){
      ballsInIndex++;
    }

    if(!upperBallDetected && lastUpperBallDetectorState){
      ballsInIndex--;
    }
  }

  public void setTurretHeading(double newHeading){
    turretEncoder.setPosition(0);
    turretHeading         = newHeading;
    targetTurretHeading   = newHeading;
    turretHeadingModifier = newHeading;
  }

  //turn turret
  public void turnTurretPID(){

    if(copilot.dpad() == 0){
      targetTurretHeading = -driveTrain.robotHeading;
    } else if (copilot.dpad() == 45){
      targetTurretHeading = -driveTrain.robotHeading + 15;
    } else if (copilot.dpad() == -45){
      targetTurretHeading = -driveTrain.robotHeading - 15;
    } else if (copilot.dpad() == -90){
      targetTurretHeading = -driveTrain.robotHeading - 30;
    } else if (copilot.dpad() == 90){
      targetTurretHeading = -driveTrain.robotHeading + 30;
    }
    
    //move the target angle right if right d pad is pressed and left if left d pad is pressed
    if (minion.dpadLeft()) {
      targetTurretHeading -= 1;
    }

    if (minion.dpadRight()){
      targetTurretHeading += 1;
    }
      
    //run the PID loop 
    turnTurretTo(targetTurretHeading);
  }

  //turns collector on
  public void runCollector (){
    collector.set(COLLECTOR_SPEED);
    collectorIsRunning = true;
    timer.reset();
  }

  //runs the transfer
  public void runTransfer (double upper, double lower){
    upperTransfer.set(upper);
    lowerTransfer.set(lower);
  }

  //runs collector in reverse
  public void reverseCollector (){
    collector.set(-COLLECTOR_SPEED);
  }

  //stops collector
  public void stopCollector(){
    collector.set(0);
    collectorIsRunning = false;
  }

  //set the shooter to a given speed in RPM
  public void setShooterSpeed (double speed){
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  //return a motor power for a give distance from the target, it is capped at 10 and 40 feet
  public double getShooterRPM(double distanceFromTargetFT){
    if(distanceFromTargetFT <= MIN_DISTANCE_TO_TARGET){
      distanceFromTargetFT = MIN_DISTANCE_TO_TARGET;
    }

    if(distanceFromTargetFT >= MAX_DISTANCE_TO_TARGET){
      distanceFromTargetFT = MAX_DISTANCE_TO_TARGET;
    }

    tempRPM = 24958.1 + -3699.15 * distanceFromTargetFT + (226.289 * Math.pow(distanceFromTargetFT, 2))
              +(- 5.84237 * Math.pow(distanceFromTargetFT, 3)) + (0.054857 * Math.pow(distanceFromTargetFT, 4));
    return tempRPM;
  }
   
  //ajust shooter rpm and enables
  public void shooterOnRPM(){
      
    if(minion.y()){
      targetSpeedRPM = 4000;
    }

    if (minion.rightTrigger()) {
      targetSpeedRPM -=  100;
    }
    
    if (minion.rightBumper()) {
      targetSpeedRPM +=  100;
    }
      
    if (targetSpeedRPM > 6200 ){
      targetSpeedRPM = 6200;
    }

    if (targetSpeedRPM < 0){
      targetSpeedRPM = 0;
    } 
      
    if (shooterPIDEnabled) {
      setShooterRPM(targetSpeedRPM);
    } else {
      setShooterSpeed(0); 
    }
  }

  //returns turrett headings
  public double getTurretHeading(){
    return turretHeading;
  }

  //modifies
  public void modifyTurretPIDProportional(){
    turretPID.modifyProportional(Math.abs(driveTrain.getHeadingChange()) * TURRET_PROPORTIONAL_GAIN);
  }

  //indexes the balls
  public void indexBalls(){

    reverseCollector = false; //stops collector
    runCollector     = false;

    if (copilot.rightTrigger()){
      runCollector = true; //if trigger is pressed it runs collectors
    } else if (copilot.rightBumper()) {
      reverseCollector = true;
    }

    if (copilot.b()){
      runTransfer(1,1);
      fireOneFlag       = false;
      prepairToFireFlag = false;
    } else if (copilot.y()){
      runTransfer(-1,-1);
      fireOneFlag       = false;
      prepairToFireFlag = false;
    } else if (copilot.leftTrigger()){
        if(readyToShoot || !visionEnabled){
          runTransfer(1,1);
        } else {
          runTransfer(0, 0);
        }
      runCollector      = false;
      reverseCollector  = false;
      fireOneFlag       = false;
      prepairToFireFlag = false;
      //check to see if it should be firing one and then if the top sensor was just detecting a ball and is now not seeing a ball meaning one has just been fired
    } else if(fireOneFlag && !(!upperBallDetected && lastUpperBallDetectorState) && (readyToShoot || !visionEnabled)){
        runTransfer(1,1);
        runCollector      = true;
        prepairToFireFlag = false;
        //check to see if it should be preparing to fire and make sure a ball has not been detected at the top
      } else if(prepairToFireFlag && !upperBallDetected){
        runTransfer(.8,.8);
        runCollector = true;
        fireOneFlag  = false;
        //check to see if a ball is detected at the bottom and on is not detected at the top
      } else if(lowerBallDetected && !upperBallDetected){
        runTransfer(1,0);
        runCollector      = true;
        fireOneFlag       = false;
        prepairToFireFlag = false;
    } else {
        runTransfer(0,0);
        fireOneFlag       = false;
        prepairToFireFlag = false;
    }

    // Run the collector unless there is a ball at either end of transfer
    if(runCollector && !(upperBallDetected || lowerBallDetected)){
      runCollector();
    } else if (reverseCollector){
      reverseCollector();
    } else {
      stopCollector();
    }
  }

  @Override
  public void teleopPeriodic(){
    //Driver 1 - (button/trigger) track and collect
    //Driver 1 (button) fire 1
    //Driver 1 (trigger) fire all
    //Driver 2 - (button) put down/up collector
    //Driver 2 - (button) collector on/off
    //Driver 2 (button) run storage system

    indexBalls();

    if(copilot.a()){
      //setting prepairToFireFlag equal to true will prepair to fire and then be set back to false when it is done prepairing to fire
      prepairToFireFlag = true;
    }

    if(copilot.leftBumper()){
      //setting fireOneFlag equal to true will prepair to fire and then be set back to false when it has fired one
      fireOneFlag = true;
    }

    collectorUpDown();

    if(minion.a() && !minionLastAState){
      if(visionEnabled) {
        visionEnabled = false;
        //turn off limelight LED
      } else {
        visionEnabled = true;
        //turn on limelight LED
      }
    }
    minionLastAState = minion.a();

    if(turretVision.targetVisible && visionEnabled){
      modifyTurretPIDProportional();
      turnTurretTo(turretHeading + turretVision.x);
      setShooterRPM(getShooterRPM(turretVision.getDistanceFromTarget()));
    } else {
      turnTurretPID();
      shooterOnRPM();
    }
  }

  @Override
  public void show() {
    // display values on SmartDashboard
    SmartDashboard.putNumber("Target RPM", targetSpeedRPM);
    SmartDashboard.putNumber("Shooter RPM", shooterRPM);
    SmartDashboard.putNumber("turret heading", turretHeading);
    SmartDashboard.putNumber("turret target heading", targetTurretHeading);
    SmartDashboard.putNumber("distance to target", turretVision.getDistanceFromTarget());
    SmartDashboard.putNumber("Turret Required Correction", turretVision.x);
    SmartDashboard.putNumber("Temp RPM", tempRPM);
    SmartDashboard.putNumber("Balls In Robot", ballsInIndex);
    SmartDashboard.putBoolean("Vision Enabled", visionEnabled);


    SmartDashboard.putBoolean("Ready to Fire", readyToShoot);
    SmartDashboard.putBoolean("Lower Ball Detector", lowerBallDetected);
    SmartDashboard.putBoolean("Upper Ball Detector", upperBallDetected);
  }

/// auto functions

}