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
  private final double COLLECTOR_SPEED = 0.2;

  private final double TURRET_SPEED                     = 0.1;
  private final double TURRET_REVS_PER_DEGREE           = 1.27866;
  private final double TURRET_ENCODER_COUNTS_PER_REV    = 4096;
  private final double TURRET_ENCODER_COUNTS_PER_DEGREE = 3203.35351071;
  private final double TURRET_DEGREES_TOLERANCE         = .2;

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

  PIDController shooterPID = new PIDController(.0005,.000001,.00005,5700,500, "Shooter");
  PIDController turretPID  = new PIDController(.005, 0, 0, 0, 5, "Turret");

  //constructor
  public FuelSystem () {
        
  }

  //initalize fuel system 
  public void init(DriverStation driverStation){
    this.driverStation = driverStation;

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

   //collectorState = new DoubleSolenoid(1,0,1);

   //reset turret heading variables
    turretHeading       = 0;
    targetTurretHeading = 0;
  }
    

  //turn the turret to a given angle
  public void turnTurretTo(double angle){
    if(Math.abs(angle - turretHeading) > TURRET_DEGREES_TOLERANCE){
      turret.set(turretPID.run(turretHeading, targetTurretHeading));
    } else {
      turret.set(0);
    }
  }

  public void updateTurretHeading(){
    turretHeading = turretEncoder.getPosition()/TURRET_ENCODER_COUNTS_PER_DEGREE;
  }

  //turn turret
  public void turnTurret(){
    
    //enable turret PID if left stick button is pressed and disable it if right stick is pressed
    if(driverStation.leftStick()){
      turretPIDEnabled = true;
    } else if (driverStation.rightStick()){
      turretPIDEnabled = false;
    }
    
    //move the target angle right if right d pad is pressed and left if left d pad is pressed
    if (driverStation.dpadLeft()) {
      if(targetTurretHeading > -100){
        targetTurretHeading -= .5;
      }
    }
    if (driverStation.dpadRight()){
      if(targetTurretHeading < 100){
        targetTurretHeading += .5;
        }
      }
      
    //run the PID loop if it has been enabled
    if(turretPIDEnabled){
      turnTurretTo(targetTurretHeading);
    }
  }

  public void runTransfer (boolean run){
    if (run) {
      upperTransfer.set(TRANSFER_SPEED);
      lowerTransfer.set(TRANSFER_SPEED);
    }else{
      upperTransfer.set(0);
      lowerTransfer.set(0);
    }
  }

  public void runCollector (boolean run){
    if (run) {
      collector.set(COLLECTOR_SPEED);
    }else{
      collector.set(0);
    }
  }

  //set the shooter to a given speed in RPM
  public void setShooterSpeed (double speed){
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  //use the limelight to find the reflective tape
  public void findTarget(){

  }

  //use the target location to aim the turret twords the target and set the shooter to the correct speed
  public void aimAtTarget(){

  }

  //use the reflective tape location and the heading of the shooter to find the robots location on the field
  public void getRobotLocation(){

  }

  //tun on the collector motors to on so that balls are sucked into the robot
  public void collectorIntake(){

  }

  //turn off the collector motors
  public void collectorOff(){

  }

  //tun on the collector motors to on so that balls are pushed out of the robot
  public void collectorOuttake(){

  }

  //use the psunamatics to put the collector down into the intaking position
  public void collectorDown(){

  }

  //use the psunamatics to put the collector up so it is inside the robot frame
  public void collectorUp(){

  }

  //use the back limelight to track balls outside the robot
  public void trackBalls(){

  }

  //run the transfer motors so that one ball is taken up into the transfer system
  public void indexFuel(){

  }

  //check to see if a ball is ready to be taken into the transfer system
  public void readyToIndex(){

  }

  //run inxed system in reverse until the last ball in the system is at the very bottom of the transfer mechanism
  public void resetIndexSystem(){

  }

  //run the transfer mechanism so that one ball is fired
  public void fireOne(){

  }

  //constatny run the transfer mechanism so that all balls are fired
  public void rapidFire(){

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
    runTransfer(driverStation.rightTrigger());
    runCollector(driverStation.rightTrigger());
    turnTurret();
  }

  @Override
  public void show() {
    // display values on SmartDashboard
    SmartDashboard.putNumber("Target RPM", targetSpeedRPM);
    SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
    SmartDashboard.putBoolean("enable", shooterPIDEnabled);
    SmartDashboard.putNumber("encoder value turret", turretEncoder.getPosition());
    SmartDashboard.putNumber("counts per rev turret", turretEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber("turret heading", turretHeading);
    SmartDashboard.putNumber("target turret heading", targetTurretHeading);
  }
}
