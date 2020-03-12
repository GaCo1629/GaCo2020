/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
//import com.revrobotics.CANSparkMaxLowLevel.FollowConfig;

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

  // Subsystem Objects
  private Vision     turretVision;
  private DriveTrain driveTrain;
  private Controller controller;

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

  private Timer collectorTimer = new Timer();  
  private Timer indexingTimer = new Timer();  

   //proportional, integral, derivative, forwardFeedInRPM, integralActiveZone, tolerance, angleWrapOn, name
  PIDController shooterPID = new PIDController(.0004,.000001,.0002,5400,200, 0, false, "Shooter");
  PIDController turretPID  = new PIDController(.04, .001, 0, 0, 10, .05, false, "Turret"); 

  // Subsystem Constants ========================================================================

  private final double COLLECTOR_SPEED = 1.0;
  private final double TURRET_REVS_PER_DEGREE   = 1.27866;
  private final double MIN_DISTANCE_TO_TARGET   = 10;
  private final double MAX_DISTANCE_TO_TARGET   = 40;
  private final double MAX_SHOOTER_RPM          = 5700;
  private final double MIN_SHOOTER_RPM          = 2000;
  private final double MAX_TURRET_ANGLE         = 100;
  private final double TURRET_PROPORTIONAL_GAIN = .002;
  private final double TURRET_INTEGRAL_GAIN     = 0;
  
  private final double SHOOTER_RPM_TOLERANCE    = 50;
  private final double TURRET_DEGREE_TOLERANCE  = .5;

  private static final int L_SHOOTER_ID  = 21;
  private static final int R_SHOOTER_ID  = 20;
  private static final int TURRET_ID     = 10;
  private static final int COLLECTOR_ID  = 1;
  private static final int U_TRANSFER_ID = 3;
  private static final int L_TRANSFER_ID = 2;

  // Subsystem Variables

  private boolean visionEnabled;
  private boolean autoTurretEnabled;
  private boolean autoRPMEnabled;

  public boolean correctRPM;
  public boolean correctTurretHeading;
  public boolean validDistance;
  public boolean readyToShoot;
  public boolean upperBallDetected;
  public boolean lowerBallDetected;
  public double  ballsInIndex;
  public int     ballsFired;

  private boolean lastUpperBallDetectorState;
  private boolean lastLowerBallDetectorState;

  private double targetSpeedRPM;
  private double shooterRPM;
  private double automatedShooterRPMModifier;

  private double turretHeading;
  private double targetTurretHeading;
 
  private boolean prepairToFireFlag;
  private boolean fireOneFlag;

  private boolean runCollector;
  private boolean reverseCollector;
  private boolean collectorIsRunning;
  private boolean fire;

  private double shooterMotorPower;
  private double tempRPM;

  //constructor
  public FuelSystem () {
  }

  private void initializeAllMembersToSafeValues() {
    // Subsystem Variables
    visionEnabled            = false;
    autoTurretEnabled        = false;
    autoRPMEnabled           = false;

    correctRPM               = false;
    correctTurretHeading     = false;
    validDistance            = false;
    readyToShoot             = false;
    upperBallDetected        = false;
    lowerBallDetected        = false;
    ballsInIndex             = 0;
    ballsFired               = 0;

    lastUpperBallDetectorState = false;
    lastLowerBallDetectorState = false;

    targetSpeedRPM          = 0;
    shooterRPM              = 0;
    automatedShooterRPMModifier = 0;

    targetTurretHeading     = getTurretHeading();
  
    prepairToFireFlag       = false;
    fireOneFlag             = false;

    runCollector            = false;
    reverseCollector        = false;
    collectorIsRunning      = false;
    fire                    = false;

    shooterMotorPower       = 0;
    tempRPM                 = 0 ;

    setTurretPower(0);
    stopCollector(); 
    runTransfer(0,0);
    setShooterSpeed(0);
  }
 
  //initalize fuel system 
  public void init(Controller controller, Vision turretVision, DriveTrain driveTrain){
    this.controller = controller;
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
    lowerCollector.set(DoubleSolenoid.Value.kReverse);

    collectorTimer.start();  
    collectorTimer.reset();

    indexingTimer.start();
    indexingTimer.reset();

    initializeAllMembersToSafeValues();
  }

  public void robotPeriodic(){
  
  }

  @Override
  public void autonomousInit() {
    initializeAllMembersToSafeValues();
    setTurretHeading(90);  // Auto always starts with shooter off to right side
  }
  
  @Override
  public void teleopInit() {
    initializeAllMembersToSafeValues();
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

    if(targetRPM == 0){
      targetRPM = 0;
    }else if(targetRPM > MAX_SHOOTER_RPM){
      targetRPM = MAX_SHOOTER_RPM;
    }else if(targetRPM < MIN_SHOOTER_RPM){
      targetRPM = MIN_SHOOTER_RPM;
    }

    targetSpeedRPM = targetRPM;
    shooterMotorPower = shooterPID.run(shooterRPM, targetRPM);
    if(shooterMotorPower < 0){
      shooterMotorPower = 0;
    }
    setShooterSpeed(shooterMotorPower);  // SLOPPY CODE.  Call twice??
    
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
    if (controller.collectorDown){
        lowerCollector.set(DoubleSolenoid.Value.kForward);
        collectorTimer.reset();

    } else if (controller.collectorUp || (!collectorIsRunning && collectorTimer.get() > 3)){
        lowerCollector.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void setBallsInRobot(int ballsInRobot){
    ballsInIndex = ballsInRobot;
  }

  public void updateVariables(){
    shooterRPM = shooterEncoder.getVelocity();
    turretHeading = (turretEncoder.getPosition()/TURRET_REVS_PER_DEGREE);
    lastLowerBallDetectorState = lowerBallDetected;
    lastUpperBallDetectorState = upperBallDetected;

    if(controller.turretTargetMinus3 || controller.turretTargetPlus3 || controller.turretTargetPlusPoint1 || controller.turretTargetMinusPoint1 || controller.longRangeShooterDefult){
      autoTurretEnabled = false;
    }

    if(controller.setShooterRPM3400 || controller.setShooterRPM3800 || controller.setShooterRPM4200 || controller.setShooterRPM4600 || controller.longRangeShooterDefult || controller.shooterRPMMinus100 || controller.shooterRPMPlus100){
      autoRPMEnabled = false;
    }

    //inverse detector states so that true indicates that a ball has been detected
    lowerBallDetected = !lowerBallDetector.get();
    upperBallDetected = !upperBallDetector.get();

    if(autoRPMEnabled || autoTurretEnabled){
      visionEnabled = true;
    } else {
      visionEnabled = false;
    }

    if(!autoRPMEnabled){
      validDistance = true;
    } else {
      validDistance = (turretVision.getDistanceFromTarget() > MIN_DISTANCE_TO_TARGET && turretVision.getDistanceFromTarget() < MAX_DISTANCE_TO_TARGET);
    }

    /** say its ready to fire if
     * target is visible
     * shooter RPM is within tolerance
     * the distance to the target is within tolerance
     * the angle to the target is within tolerance
     *  */
    correctTurretHeading = Math.abs(turretHeading - targetTurretHeading) < TURRET_DEGREE_TOLERANCE;
    correctRPM = Math.abs(shooterRPM - targetSpeedRPM) < SHOOTER_RPM_TOLERANCE;
    if(turretVision.targetVisible && correctRPM && validDistance && correctTurretHeading){
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
      ballsFired++;
      fire = false;
    }

    if(reverseCollector && !lowerBallDetected && lastLowerBallDetectorState){
      ballsInIndex--;
    }
  }

  public void setTurretHeading(double newHeading){
    turretEncoder.setPosition(newHeading * TURRET_REVS_PER_DEGREE);
    turretHeading         = newHeading;
    targetTurretHeading   = newHeading;
  }

  //turn turret
  public void turnTurretPID(){

    if(controller.turretSetFieldCentric0){
      targetTurretHeading = -driveTrain.robotHeading;
    } else if (controller.turretSetFieldCentricPositive15){
      targetTurretHeading = -driveTrain.robotHeading + 15;
    } else if (controller.turretSetFieldCentricNegative15){
      targetTurretHeading = -driveTrain.robotHeading - 15;
    } else if (controller.turretSetFieldCentricNegative30){
      targetTurretHeading = -driveTrain.robotHeading - 30;
    } else if (controller.turretSetFieldCentricPositive30){
      targetTurretHeading = -driveTrain.robotHeading + 30;
    }
    
    //move the target angle right if right d pad is pressed and left if left d pad is pressed
    if (controller.turretTargetMinusPoint1) {
      targetTurretHeading -= .1;
    }

    if (controller.turretTargetPlusPoint1){
      targetTurretHeading += .1;
    }

    if (controller.turretTargetPlus3){
      targetTurretHeading += 3;
    }  
    
    if (controller.turretTargetMinus3){
      targetTurretHeading -= 3;
    }

    //run the PID loop 
    turnTurretTo(targetTurretHeading);
  }

  //turns collector on
  public void runCollector (){
    collector.set(COLLECTOR_SPEED);
    collectorIsRunning = true;
    collectorTimer.reset();
  }

  //runs the transfer
  public void runTransfer (double upper, double lower){
    upperTransfer.set(upper);
    lowerTransfer.set(lower);
  }

  //runs collector in reverse
  public void reverseCollector (){
    collector.set(-COLLECTOR_SPEED);
    collectorIsRunning = true;
    collectorTimer.reset();
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

    if (controller.shooterRPMMinus100) {
      targetSpeedRPM -=  100;
    }
    
    if (controller.shooterRPMPlus100) {
      targetSpeedRPM +=  100;
    }

    if(controller.setShooterRPM0){
      targetSpeedRPM = 0;
    }
      
    if (targetSpeedRPM > 6200 ){
      targetSpeedRPM = 6200;
    }

    if (targetSpeedRPM < 0){
      targetSpeedRPM = 0;
    } 
      
    setShooterRPM(targetSpeedRPM);
  }

  //returns turrett headings
  public double getTurretHeading(){
    return turretHeading;
  }

  //indexes the balls
  public void indexBalls(){

    reverseCollector = false; //stops collector
    runCollector     = false;

    if (controller.collectorOn){
      runCollector = true; //if trigger is pressed it runs collectors
    } else if (controller.collectorReverse) {
      reverseCollector = true;
    }

    //see if the forward override is running
    if (controller.runTransfer){
      runTransfer(1,1);
      fireOneFlag       = false;
      prepairToFireFlag = false;
      fire              = false;

    //see if the backward override is running
    } else if (controller.reverseTransfer){
      runTransfer(-1,-1);
      fireOneFlag       = false;
      prepairToFireFlag = false;
      fire              = false;
    
    //see if fire all is enabled
    } else if (controller.fireAll){
      if(!upperBallDetected){
        runTransfer(.6,.6);
      } else if((readyToShoot || !visionEnabled) && correctRPM && correctTurretHeading){
        fire = true;
        runTransfer(1,1);
      } else {
        if(fire){
          runTransfer(1,1);
        } else {
          runTransfer(0,0);
        }
      }
      fireOneFlag       = false;
      prepairToFireFlag = false;
    
    //check to see if it should be firing one and then if the top sensor was just detecting a ball and is now not seeing a ball meaning one has just been fired
    } else if(fireOneFlag){
      if(!upperBallDetected){
        runTransfer(.6,.6);
      } else if ((readyToShoot || !visionEnabled)&& correctRPM && correctTurretHeading) {
        fire = true;
        runTransfer(1,1);
      } else {
        if(fire){
          runTransfer(1,1);
        } else {
          runTransfer(0,0);
        }     
      }
      prepairToFireFlag = false;

      if(!upperBallDetected && lastUpperBallDetectorState){
        fireOneFlag = false;
      }

    //check to see if it should be preparing to fire and make sure a ball has not been detected at the top
    } else if(prepairToFireFlag && !upperBallDetected){
      runTransfer(.6,.6);
      runCollector = true;
      fireOneFlag  = false;
      fire         = false;

    //check to see if a ball is detected at the bottom and on is not detected at the top
    } else if(lowerBallDetected && !upperBallDetected){
      runTransfer(1,.1);
      runCollector      = true;
      fireOneFlag       = false;
      prepairToFireFlag = false;
      fire              = false;

    //otherwise stop the transfer and reset all the booleans
    } else {
      runTransfer(0,0);
      fireOneFlag       = false;
      prepairToFireFlag = false;
      fire              = false;

    }

    // Run the collector unless there is a ball at the top of the transfer
    if(runCollector && !(upperBallDetected)){
      runCollector();
    } else if (reverseCollector){
      reverseCollector();
    } else {
      stopCollector();
    }
  }

  public void toggleVision(){
    if(!autoTurretEnabled || !autoRPMEnabled){
      autoTurretEnabled = true;
      autoRPMEnabled    = true;
    } else {
      autoTurretEnabled = false;
      autoRPMEnabled    = false;
    }
  }

  public double getShooterRPM(){
    return shooterRPM;
  }

  public void turnOffShooter(){
    leftShooter.set(0);
    rightShooter.set(0);
  }

  public void setTurretPower(double power){
    turret.set(0);
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

    if(controller.automatedRPMMinus25){
      automatedShooterRPMModifier -= 25;
    }
    if(controller.automatedRPMPlus25){
      automatedShooterRPMModifier += 25;
    }

    if(controller.resetTurretHeading){
      setTurretHeading(0);
    }

    if(controller.runUpBalls){
      //setting prepairToFireFlag equal to true will prepair to fire and then be set back to false when it is done prepairing to fire
      prepairToFireFlag = true;
    }

    if(controller.fireOne){
      //setting fireOneFlag equal to true will prepair to fire and then be set back to false when it has fired one
      fireOneFlag = true;
    }

    if(controller.setShooterRPM3400){
      targetSpeedRPM = 3400;
      autoRPMEnabled = false;
    }

    if(controller.setShooterRPM3800){
      targetSpeedRPM = 3800;
      autoRPMEnabled = false;
    }

    if(controller.setShooterRPM4200){
      targetSpeedRPM = 4200;
      autoRPMEnabled = false;
    }

    if(controller.setShooterRPM4600){
      targetSpeedRPM = 4600;
      autoRPMEnabled = false;
    }

    if(controller.longRangeShooterDefult){
      targetSpeedRPM = 4450;
      targetTurretHeading = -driveTrain.robotHeading - 10;

      autoRPMEnabled    = false;
      autoTurretEnabled = false;
    }


    collectorUpDown();

    if(controller.toggleAutoVision){
      toggleVision();
    }

    if(controller.zoomLimelight){
      turretVision.zoomInLimelight();
    } else {
      turretVision.zoomOutLimelight();
    }

    if(turretVision.targetVisible && visionEnabled){
        if(autoRPMEnabled){
         setRPMBasedOnVision();
        } else {
          shooterOnRPM();
        }

        if(autoTurretEnabled) {
          turnTurretToVisionTarget();
        } else {
          turnTurretPID();
        }

    } else {
      turnTurretPID();
      shooterOnRPM();
    }
  }
  
  public void setRPMBasedOnVision(){
    setShooterRPM(getShooterRPM(turretVision.getDistanceFromTarget()) + automatedShooterRPMModifier);
  }

  public void turnTurretToVisionTarget(){
    turnTurretTo(turretHeading + turretVision.x);
  }

  public void turnOffVision(){
    autoRPMEnabled = false;
    autoTurretEnabled = false;
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
    SmartDashboard.putNumber("Automated Shooter RPM Modifier", automatedShooterRPMModifier);
    SmartDashboard.putBoolean("Automated Turret Heading Enabled", autoTurretEnabled);
    SmartDashboard.putBoolean("Automated RPM Enabled", autoRPMEnabled);

    SmartDashboard.putBoolean("Ready to Fire", readyToShoot);
    SmartDashboard.putBoolean("Lower Ball Detector", lowerBallDetected);
    SmartDashboard.putBoolean("Upper Ball Detector", upperBallDetected);
    SmartDashboard.putBoolean("Correct Turret Heading", correctTurretHeading);
    SmartDashboard.putBoolean("Correct RPM", correctRPM);
    SmartDashboard.putNumber("Shots fire", ballsFired);


    SmartDashboard.putNumber("Shooter Power Input", shooterMotorPower);
  }

/// auto functions

}