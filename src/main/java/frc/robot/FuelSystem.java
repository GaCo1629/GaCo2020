/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.PIDController;

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

  private DriverStation driverStation;
  private DriverStation driverStation2;
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

  private final double TRANSFER_SPEED  = 1;
  private final double COLLECTOR_SPEED = .8;

  private final double TURRET_SPEED           = 0.1;
  private final double TURRET_REVS_PER_DEGREE = 1.27866;
  private final double MIN_DISTANCE_TO_TARGET = 10;
  private final double MAX_DISTANCE_TO_TARGET = 40;
  private final double MAX_SHOOTER_RPM        = 5700;
  private final double MIN_SHOOTER_RPM        = 3000;
  private final double MAX_TURRET_ANGLE       = 100;

  private final double SHOOTER_RPM_TOLERANCE   = 75;
  private final double TURRET_DEGREE_TOLERANCE = .2;

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

  private double lastTime    = 0;
  private boolean firstTime  = true;

  public boolean readyToShoot = false;

  private double tempRPM;

  private Timer timer = new Timer();  

  PIDController shooterPID = new PIDController(.0003,.000001,.0005,5700,500, 0, false, "Shooter");
  PIDController turretPID  = new PIDController(.04, .001, 0, 0, 2, .05, false, "Turret");

  
  //constructor
  public FuelSystem () {
        
  }

  //initalize fuel system 
  public void init(DriverStation driverStation, DriverStation driverStation2, Vision turretVision, DriveTrain driveTrain){
    this.driverStation  = driverStation;
    this.driverStation2 = driverStation2;
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

    //invert collector so that positive values move balls up twords the shooter
    lowerTransfer.setInverted(true);
    turret.setInverted(true);

    lowerCollector = new DoubleSolenoid(1,1,0);

    //reset turret heading variables
    turretHeading       = 0;
    targetTurretHeading = 0;
    turretPIDEnabled    = false;

    //reset shooter RPM variables
    shooterRPM          = 0;
    targetSpeedRPM      = 2300;
    shooterPIDEnabled   = false;

    lowerCollector.set(DoubleSolenoid.Value.kReverse);

    readyToShoot = false;

    firstTime = true;
    timer.reset();
  }
  
  @Override
  public void teleopInit() {

  }

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

  public double clip(double val, double range){
    if(val > range){
      val = range;
    } 

    if(val < -range){
      val = -range;
    }

    return val;
  }

  public void toggleSolenoid(){
    if (driverStation2.dpadUp()){
      lowerCollector.set(DoubleSolenoid.Value.kForward);
  } else if (driverStation2.dpadDown()){
      lowerCollector.set(DoubleSolenoid.Value.kReverse);
  }
  }

  public void updateVariables(){
    shooterRPM = shooterEncoder.getVelocity();
    turretHeading = turretEncoder.getPosition()/TURRET_REVS_PER_DEGREE + turretHeadingModifier;

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
  }

  public void setTurretHeading(double newHeading){
    turretEncoder.setPosition(0);
    turretHeading         = newHeading;
    targetTurretHeading   = newHeading;
    turretHeadingModifier = newHeading;
  }

  //turn turret
  public void turnTurretPID(){
  
    //enable turret PID if left stick button is pressed and disable it if right stick is pressed
    if(driverStation.dpadUp()){
      turretPIDEnabled = true;
    } else if (driverStation.dpadDown()){
      turretPIDEnabled = false;
    }
    
    //move the target angle right if right d pad is pressed and left if left d pad is pressed
    if (driverStation.dpadRight()) {
      targetTurretHeading -= .5;
    }

    if (driverStation.dpadLeft()){
      targetTurretHeading += .5;
    }
      
      //run the PID loop if it has been enabled
    if(turretPIDEnabled) {
      turnTurretTo(targetTurretHeading);
    } else {
      turret.set(0);
    }
  }

  public void runTransfer (){
    upperTransfer.set(TRANSFER_SPEED);
    lowerTransfer.set(TRANSFER_SPEED);
  }

  public void runCollector (){
    collector.set(COLLECTOR_SPEED);
  }

  public void reverseTransfer (){
    upperTransfer.set(-TRANSFER_SPEED);
    lowerTransfer.set(-TRANSFER_SPEED);
 
  }

  public void stopTransfer(){
    upperTransfer.set(0);
    lowerTransfer.set(0);
  }

  public void reverseCollector (){
    collector.set(-COLLECTOR_SPEED);
  }

  public void stopCollector(){
    collector.set(0);
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

  public void shooterOnRPM(){
    if (driverStation.y()){
      shooterPIDEnabled = true;
    } else if (driverStation.a()){
      shooterPIDEnabled = false;
    }
      
    if (driverStation.b()) {
      targetSpeedRPM +=  10;
    }
    
    if (driverStation.x()) {
      targetSpeedRPM -=  10;
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

  public double getTurretHeading(){
    return turretHeading;
  }

  @Override
  public void teleopPeriodic(){
    //Driver 1 - (button/trigger) track and collect
    //Driver 1 (button) fire 1
    //Driver 1 (trigger) fire all
    //Driver 2 - (button) put down/up collector
    //Driver 2 - (button) collector on/off
    //Driver 2 (button) run storage system

    if (driverStation.rightTrigger()){
      runTransfer();
      runCollector();
    } else if (driverStation.leftTrigger()){
      reverseTransfer();
      reverseCollector();
    } else {
      stopTransfer();
      stopCollector();
    }

    toggleSolenoid();


    if(driverStation2.leftBumper() && turretVision.targetVisible){
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
    SmartDashboard.putBoolean("Ready to Fire", readyToShoot);

  }
}