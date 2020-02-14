/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.PIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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

  DriverStation driverStation;
  DriverStation driverStation2;

  private VictorSP lowerTransfer;
  private VictorSP upperTransfer;
  private VictorSP collector;

  private CANSparkMax turret;
  private CANSparkMax leftShooter; 
  private CANSparkMax rightShooter;

  private CANEncoder shooterEncoder;
  private CANEncoder turretEncoder;
  private DoubleSolenoid collectorState;

  private final double TRANSFER_SPEED  = 1;
  private final double COLLECTOR_SPEED = .5;

  private final double TURRET_SPEED           = 0.1;
  private final double TURRET_REVS_PER_DEGREE = 1.27866;
  private final double MIN_DISTANCE_TO_TARGET = 10;
  private final double MAX_DISTANCE_TO_TARGET = 40;
  private final double MAX_TURRET_ANGLE       = 100;

  private static final int L_SHOOTER_ID  = 21;
  private static final int R_SHOOTER_ID  = 20;
  private static final int TURRET_ID     = 10;
  private static final int COLLECTOR_ID  = 1;
  private static final int U_TRANSFER_ID = 3;
  private static final int L_TRANSFER_ID = 2;

  private boolean shooterPIDEnabled = false;
  private double targetSpeedRPM     = 0;

  private double turretHeading       = 0;
  private double targetTurretHeading = 0;
  private boolean turretPIDEnabled   = false;
  double turretHeadingModifier       = 0;

  PIDController shooterPID = new PIDController(.0004,.000001,.00005,5700,500, 0, false, "Shooter");
  PIDController turretPID  = new PIDController(.05, 0, 0, 0, 2, .01, false, "Turret");

  Vision turretLimelght;

  //constructor
  public FuelSystem () {
        
  }

  //initalize fuel system 
  public void init(DriverStation driverStation, DriverStation driverStation2, Vision frontLimelight){
    this.driverStation  = driverStation;
    this.driverStation2 = driverStation2;
    this.turretLimelght = frontLimelight;

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

   //collectorState = new DoubleSolenoid(1,0,1);

   //reset turret heading variables
    turretHeading       = 0;
    targetTurretHeading = 0;
    turretPIDEnabled    = false;
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

  public double clip(double val, double range){
    if(val > range){
      val = range;
    } 

    if(val < -range){
      val = -range;
    }

    return val;

  }

  public void updateTurretHeading(){
    turretHeading = turretEncoder.getPosition()/TURRET_REVS_PER_DEGREE + turretHeadingModifier;
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
    if(driverStation2.a()){
      turretPIDEnabled = true;
    } else if (driverStation2.b()){
      turretPIDEnabled = false;
    }
    
    //move the target angle right if right d pad is pressed and left if left d pad is pressed
    if (driverStation2.x()) {
      targetTurretHeading -= .5;
    }

    if (driverStation2.y()){
      targetTurretHeading += .5;
    }
      
      //run the PID loop if it has been enabled
    if(turretPIDEnabled) {
      turnTurretTo(targetTurretHeading);
    } else {
      turret.set(0);
    }
  }

  public void turnTurret(){
    if (driverStation.dpadLeft()) {
      turret.set(-TURRET_SPEED);
    }else if (driverStation.dpadRight()){
      turret.set(TURRET_SPEED);
    }else{
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
  public double getShooterPower(double distanceFromTargetFT){
    if(distanceFromTargetFT <= MIN_DISTANCE_TO_TARGET){
      distanceFromTargetFT = MIN_DISTANCE_TO_TARGET;
    }

    if(distanceFromTargetFT >= MAX_DISTANCE_TO_TARGET){
      distanceFromTargetFT = MAX_DISTANCE_TO_TARGET;
    }

    return (85776 + -21115 * distanceFromTargetFT + 2199 * Math.pow(distanceFromTargetFT, 2)
    + -119.00 * Math.pow(distanceFromTargetFT, 3) + 3.580000 * Math.pow(distanceFromTargetFT, 4)
    + -0.0563 * Math.pow(distanceFromTargetFT, 5) + 0.000363 * Math.pow(distanceFromTargetFT, 6));


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
      setShooterSpeed(shooterPID.run(shooterEncoder.getVelocity(), targetSpeedRPM));
    } else {
      setShooterSpeed(0); 
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
    shooterOnRPM();
    if(driverStation.rightTrigger()){
      runTransfer();
      runCollector();
    } else if (driverStation.leftTrigger()){
      reverseTransfer();
      reverseCollector();
    } else {
      stopTransfer();
      stopCollector();
    }

    if(driverStation2.leftBumper()){
      turnTurretTo(turretHeading + turretLimelght.x);
    } else {
      turnTurretPID();
    }
  }

  @Override
  public void show() {
    // display values on SmartDashboard
    SmartDashboard.putNumber("Target RPM", targetSpeedRPM);
    SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
    SmartDashboard.putBoolean("enable", shooterPIDEnabled);
    SmartDashboard.putNumber("encoder value turret", turretEncoder.getPosition());
    SmartDashboard.putNumber("turret heading", turretHeading);
    SmartDashboard.putNumber("turret target heading", targetTurretHeading);
  }
}