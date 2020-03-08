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

  private GaCoDrive pilot;
  private GaCoDrive copilot;
  private GaCoDrive minion;
  private Controller controller;

  private boolean visionEnabled     = true;
  private boolean autoTurretEnabled = true;
  private boolean autoRPMEnabled    = true;

  private Vision     turretVision;
  private DriveTrain driveTrain;

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

  private final double SHOOTER_RPM_TOLERANCE    = 50;
  private final double TURRET_DEGREE_TOLERANCE  = .5;

  private static final int L_SHOOTER_ID  = 21;
  private static final int R_SHOOTER_ID  = 20;
  private static final int TURRET_ID     = 10;
  private static final int COLLECTOR_ID  = 1;
  private static final int U_TRANSFER_ID = 3;
  private static final int L_TRANSFER_ID = 2;

  private double targetSpeedRPM     = 0;
  private double shooterRPM         = 0;
  private double automatedShooterRPMModifier = 0;

  private double turretHeading         = 0;
  private double targetTurretHeading   = 0;
  private double turretHeadingModifier = 0;
  private double ballsInIndex          = 0;

  private boolean prepairToFireFlag = false;
  private boolean fireOneFlag       = false;

  public boolean correctRPM           = false;
  public boolean correctTurretHeading = false;
  public boolean validDistance        = false;
  public boolean readyToShoot         = false;
  private boolean runCollector        = false;
  private boolean reverseCollector    = false;
  private boolean collectorIsRunning  = false;
  private boolean fire                = false;

  private double shooterMotorPower = 0;

  private double tempRPM;

  private Timer collectorTimer = new Timer();  
  private Timer indexingTimer = new Timer();  


  //proportional, integral, derivative, forwardFeedInRPM, integralActiveZone, tolerance, angleWrapOn, name
  PIDController shooterPID = new PIDController(.0004,.000001,.0002,5400,200, 0, false, "Shooter");
  PIDController turretPID  = new PIDController(.04, .001, 0, 0, 10, .05, false, "Turret");

  //constructor
  public FuelSystem () {
  }

 
  //initalize fuel system 
  public void init(GaCoDrive pilot, GaCoDrive copilot, GaCoDrive minion, Controller controller, Vision turretVision, DriveTrain driveTrain){
    this.pilot   = pilot;
    this.copilot = copilot;
    this.minion  = minion;
    this.controller = controller;
    this.turretVision   = turretVision;
    this.driveTrain     = driveTrain;

    //initialize motors
    leftShooter  = new CANSparkMax(L_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightShooter = new CANSparkMax(R_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    turret       = new CANSparkMax(TURRET_ID,    CANSparkMaxLowLevel.MotorType.kBrushless);

    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    
    //leftShooter.Config();
   // rightShooter.Config();
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

    //reset shooter RPM variables
    shooterRPM          = 0;
    targetSpeedRPM      = 2300;
    autoTurretEnabled   = true;
    autoRPMEnabled      = true;
    fire                = false;

    lowerCollector.set(DoubleSolenoid.Value.kReverse);

    readyToShoot = false;

    collectorTimer.start();    
    indexingTimer.start();

    collectorTimer.reset();
    indexingTimer.reset();
  }

  public void robotPeriodic(){
  
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
    shooterMotorPower = shooterPID.run(shooterRPM, targetRPM);
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
    if (controller.collectorDown){
        lowerCollector.set(DoubleSolenoid.Value.kForward);
        collectorTimer.reset();

    } else if (controller.collectorUp || (!collectorIsRunning && collectorTimer.get() > 3)){
        lowerCollector.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void updateVariables(){
    shooterRPM = shooterEncoder.getVelocity();
    turretHeading = turretEncoder.getPosition()/TURRET_REVS_PER_DEGREE + turretHeadingModifier;
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
      fire = false;
    }

    if(reverseCollector && !lowerBallDetected && lastLowerBallDetectorState){
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
      runTransfer(1,0);
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

    // Run the collector unless there is a ball at either end of transfer
    if(runCollector && !(upperBallDetected || lowerBallDetected)){
      runCollector();
    } else if (reverseCollector){
      reverseCollector();
    } else {
      stopCollector();
    }
  }

  public double getShooterRPM(){
    return shooterRPM;
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
      turretHeading = 0;
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

    if(turretVision.targetVisible && visionEnabled){
        if(autoRPMEnabled){
          setShooterRPM(getShooterRPM(turretVision.getDistanceFromTarget()) + automatedShooterRPMModifier);
        } else {
          shooterOnRPM();
        }

        if(autoTurretEnabled) {
          turnTurretTo(turretHeading + turretVision.x);
        } else {
          turnTurretPID();
        }

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
    SmartDashboard.putNumber("Automated Shooter RPM Modifier", automatedShooterRPMModifier);
    SmartDashboard.putBoolean("Automated Turret Heading Enabled", autoTurretEnabled);
    SmartDashboard.putBoolean("Automated RPM Enabled", autoRPMEnabled);

    SmartDashboard.putBoolean("Ready to Fire", readyToShoot);
    SmartDashboard.putBoolean("Lower Ball Detector", lowerBallDetected);
    SmartDashboard.putBoolean("Upper Ball Detector", upperBallDetected);
    SmartDashboard.putBoolean("Correct Turret Heading", correctTurretHeading);
    SmartDashboard.putBoolean("Correct RPM", correctRPM);



    SmartDashboard.putNumber("Shooter Power Input", shooterMotorPower);
  }

/// auto functions

}