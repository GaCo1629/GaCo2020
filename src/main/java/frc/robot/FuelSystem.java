/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/



package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

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

    private CANSparkMax turret;
    private VictorSP lowerTransfer;
    private VictorSP upperTransfer;
    private VictorSP collector;
    private CANSparkMax leftShooter; 
    private CANSparkMax rightShooter;
    private CANEncoder shooterEncoder;
    private CANEncoder turretEncoder;
    private DoubleSolenoid collectorState;

  

    final double TRANSFER_SPEED = 1;
    final double COLLECTOR_SPEED = 0.2;
    final double TURRETT_SPEED = 0.4;
    final double TURRETT_CLICKS = 20;
    
    private static final int L_SHOOTER_ID = 21;
    private static final int R_SHOOTER_ID = 20;
    private static final int TURRET_ID =10;
    private static final int COLLECTOR_ID = 1;
    private static final int U_TRANSFER_ID=3;
    private static final int L_TRANSFER_ID= 2;



    public double m_setpoint = 0.1;  
    public double m_speed = 0.0;
    public boolean m_enable = false;
  

    //constructor
    public  FuelSystem () {
        
    }

    //initalize fuel system 
    public void init(DriverStation driverStation){
        this.driverStation = driverStation;
        // initialize motor

        leftShooter  = new CANSparkMax(L_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkMax(R_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        turret = new CANSparkMax(TURRET_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();
        
        leftShooter.setInverted(true);
      
      
        // Encoder object created to display position/velocity values
        shooterEncoder = leftShooter.getEncoder(); 
        
        turretEncoder = turret.getEncoder();

        //Victor SP
        upperTransfer = new VictorSP(U_TRANSFER_ID);
        lowerTransfer = new VictorSP(L_TRANSFER_ID);

        collector = new VictorSP(COLLECTOR_ID);

        lowerTransfer.setInverted(true);

       // collectorState = new DoubleSolenoid(1,0,1);

        SmartDashboard.putNumber("Setpoint", 0);
        SmartDashboard.putNumber("Speed", 0);
        SmartDashboard.putNumber("ENCODER", 0);


        
    }
    

    //turn the turret to a given angle
    public void turnTurretTo(double angle){

    }

    //turn turret
    public void turnTurret(){

      if (driverStation.dpadLeft()) {
        turret.set(TURRETT_SPEED);
    }else if (driverStation.dpadRight()){
      turret.set(-TURRETT_SPEED);
    }else{
      turret.set(0);
    }
    }
  

   // public void turnTurretTicks(){

    /*  if (driverStation.dpadUp()) {
        new int pos= turretEncoder.getPosition()
        while(turretEncoder.getPosition() < )
    
        turret.set(TURRETT_SPEED);
        turretEncoder.getPosition()
    }else if (driverStation.dpadDown()){
      turret.set(-TURRETT_SPEED);
    }else{
      turret.set(0);
    }
    } */

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

    public void changeShooterSetpoint(){
        if (driverStation.b()) {
            m_setpoint +=  0.002;
          }
          
          if (driverStation.x()) {
            m_setpoint -=  0.002;
          }
      
          if (m_setpoint > 1.0 ) 
            m_setpoint = 1.0;
          if (m_setpoint < 0) m_setpoint = 0;
      
    }

    //use a pid loop to tune the target speed
    public void tuneShooterSpeedPID(){

    }

    //use the limelight to find the reflective tape
    public void findTarget(){

    }

    //use the target location to aim the turrett twords the target and set the shooter to the correct speed
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

   

    public void shooterOn(){
        if (driverStation.y()){
            m_enable = true;
          } else if (driverStation.a()){
            m_enable = false;
        }
      
        if (m_enable) {
            setShooterSpeed(m_setpoint);
          
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
        shooterOn();  
        changeShooterSetpoint();
        runTransfer(driverStation.rightTrigger());
        runCollector(driverStation.rightTrigger());
        turnTurret();
      
        show();
    }


    public void show() {
          // display PID coefficients on SmartDashboard
          SmartDashboard.putNumber("Setpoint", m_setpoint);
          SmartDashboard.putNumber("Speed", shooterEncoder.getVelocity());
          SmartDashboard.putBoolean("enable", m_enable);
          SmartDashboard.putNumber("encoder value turrett", turretEncoder.getPosition());

     }
     
}
