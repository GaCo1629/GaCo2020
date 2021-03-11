/*-----------------------------*/
/* Copyright (c) 2020 GaCo     */
/*-----------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;

public class Auto extends Subsystem {

    // intiliaze the needed classes
    private DriveTrain    driveTrain;
    private FuelSystem    fuelSystem;
    private Vision        turretVision;

    private AutoMode selAutoMode = AutoMode.NONE;
    private Integer selDistance;
    private Integer selDelayTime;

    public Timer driveTime= new Timer();
    
    // private NumBalls selNumBalls;
    private SendableChooser <AutoMode> autoMode = new SendableChooser<>();
    private SendableChooser <Integer> distance = new SendableChooser<>();
    private SendableChooser <Integer> delayTime = new SendableChooser<>();

    private ArrayList<Step> path = new ArrayList<>();

    public void init(DriveTrain driveTrain, FuelSystem fuelSystem, Vision turretVision){
        this.driveTrain = driveTrain;
        this.fuelSystem = fuelSystem;
        this.turretVision = turretVision;

        //set up autonomus options
        autoMode.setDefaultOption("Simple Shoot", AutoMode.SIMPLE_SHOOT);
        autoMode.addOption("Simple Shoot", AutoMode.SIMPLE_SHOOT);
        autoMode.addOption("Smart Shoot", AutoMode.SMART_SHOOT);
        autoMode.addOption("Drive Path", AutoMode.DRIVE_PATH);
        autoMode.addOption("None", AutoMode.NONE);
        SmartDashboard.putData("Auto Mode", autoMode);
        
     
        distance.setDefaultOption("5 feet fwd", 60);
        distance.addOption("5 feet fwd", 60);
        distance.addOption("5 feet rev", -60);
        distance.addOption("none", 0);

        delayTime.setDefaultOption("0", 0);
        delayTime.addOption("0", 0);
        delayTime.addOption("1", 1);
        delayTime.addOption("2", 2);
        delayTime.addOption("3", 3);
        delayTime.addOption("4", 4);
        delayTime.addOption("5", 5);
        delayTime.addOption("6", 6);
        delayTime.addOption("7", 7);
        delayTime.addOption("8", 8);
        delayTime.addOption("9", 9);
        delayTime.addOption("10", 10);
        delayTime.addOption("11", 11);
        delayTime.addOption("12", 12);
        delayTime.addOption("13", 13);
        delayTime.addOption("14", 14);
       
         SmartDashboard.putData("distance", distance);
         SmartDashboard.putData("delay", delayTime);

        SmartDashboard.putString("selAutoMode", ".---");
    }

    @Override
    public void autonomousInit(){
      // Determine the desired Autonomous Action
      selAutoMode = autoMode.getSelected();
      selDistance = distance.getSelected();
      selDelayTime = delayTime.getSelected();
      currentShooterState = SMShooting.INIT;
      driveTime.start();
      driveTime.reset();

      //selNumBalls = numBalls.getSelected();
      fuelSystem.setBallsInRobot(3);

      switch (selAutoMode){

        case NONE:
          SmartDashboard.putString("selAutoMode", "None selecter");
          break;

        case SIMPLE_SHOOT:
          SmartDashboard.putString("selAutoMode", "simple shoot selected");
          /*shootNow =true;
          shotsWanted = 3;*/
          break;

        case SMART_SHOOT:
          SmartDashboard.putString("selAutoMode", "smart shoot selected");
          /*shootNow =true;
          shotsWanted = 3;*/
          break;

        case DRIVE_PATH:
          //(StepMode initMode, double initSpeed, double initDistance, double initHeading, double initTimeout){
          SmartDashboard.putString("selAutoMode", "Drive Path Selected");
          path.clear();
          path.add(new Step(StepMode.STRAIGHT, 0.25, 1.0, 0.0, 2.0));
          //path.add(new Step(StepMode.PIVOT, 1.0, 0, 90.0, 5.0));
          path.add(new Step(StepMode.STOP, 0, 0, 0, 0));
          driveTrain.setPath(path); 
          break; 

        default:
          break;
        }
        driveTrain.setPath(path);
    }
    
    @Override
    public void autonomousPeriodic(){
        /*switch (selAutoMode){

            case NONE:
           
            break;
      
            case SIMPLE_SHOOT:
            if (runSimpleShooter() != SMShooting.INIT ){
              runSimpleShooter();
            }
            break;
      
            case SMART_SHOOT:
            if (runSmartShooter() != SMShooting.INIT ){
              runSmartShooter();
            }
            break;
      
          }*/
    }
    
  //=============================================================\\
  //            actual auto functions.                            \\
  //=============================================================  \\

  public boolean useLimeLight = true;
  public boolean shootNow = false;
  public int shotCounter = 0;
  public int shotsWanted = 0;


  //SHOOTING STATE MACHINE
  enum SMShooting{
   INIT,
   GETTING_READY,
   SHOOTING, 
   DRIVING
  }

  public SMShooting currentShooterState = SMShooting.INIT;

  public SMShooting runSimpleShooter(){
    
    switch (currentShooterState){
      case INIT:

        //check for time to shoot
        if (shootNow && (driveTime.get() >= selDelayTime)){
          fuelSystem.setShooterRPM(4000);
          fuelSystem.turnTurretTo(0);
          shootNow = false;
          currentShooterState = SMShooting.GETTING_READY;
        }
        break;

      case GETTING_READY:
        //check ready to shoot
        if(fuelSystem.correctRPM && fuelSystem.correctTurretHeading){
          fuelSystem.runTransfer(.9, .9);
          currentShooterState = SMShooting.SHOOTING;
        }else{
          fuelSystem.setShooterRPM(4000);
          fuelSystem.turnTurretTo(0);

          //turns on transfer if uper ball is not detected. 
          if(!fuelSystem.upperBallDetected){
            fuelSystem.runTransfer(.9, .9);
          } else {
            fuelSystem.runTransfer(0, 0);
          }
        }
        break;

      case SHOOTING:
        //check if we have shot desired shots
        if (fuelSystem.ballsFired >= shotsWanted){
          fuelSystem.setShooterRPM(0);
          fuelSystem.runTransfer(0, 0);
          fuelSystem.setShooterSpeed(0);
          // are we driveing
          if (selDistance == 0){
            currentShooterState = SMShooting.INIT;
          } else {
            //Drive based on the user selceted input
            driveTime.reset();
            currentShooterState = SMShooting.DRIVING;
          }
        }else{
          fuelSystem.setShooterRPM(4000);
          fuelSystem.turnTurretTo(0);
          fuelSystem.setTurretPower(0);
          if (fuelSystem.correctRPM){
            fuelSystem.runTransfer(.9, .9);
          } else {
            fuelSystem.runTransfer(0, 0);
          }
        }
        break;

      case DRIVING:
          if (driveTime.get() > 1){
            driveTrain.moveRobot(0,0);
            currentShooterState = SMShooting.INIT;
          } else {
            driveTrain.moveRobot(0.4*Math.signum(selDistance) , 0);   
          }

        break;

       
    }
    return currentShooterState;
  }
  //////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  public SMShooting runSmartShooter(){
    
    switch (currentShooterState){
      case INIT:
        //check for time to shoot
        if (shootNow && (driveTime.get() >= selDelayTime)){
          fuelSystem.setShooterRPM(4000);
          fuelSystem.turnTurretTo(0);
          shootNow = false;
          currentShooterState = SMShooting.GETTING_READY;
        }
        break;

      case GETTING_READY:
        //check ready to shoot
        if (turretVision.targetVisible){
          fuelSystem.setRPMBasedOnVision();
          fuelSystem.turnTurretToVisionTarget();
        } else {
          fuelSystem.setShooterRPM(4000);
          fuelSystem.turnTurretTo(0);
        }
        if(fuelSystem.readyToShoot){
          fuelSystem.runTransfer(.7, .7);
          currentShooterState = SMShooting.SHOOTING;
        } else {
          //turns on transfer if uper ball is not detected. 
          if(!fuelSystem.upperBallDetected){
            fuelSystem.runTransfer(.7, .7);
          } else {
            fuelSystem.runTransfer(0, 0);
          }
        }
        break;

      case SHOOTING:
        //check if we have shot desired shots
        if (turretVision.targetVisible){
          fuelSystem.setRPMBasedOnVision();
          fuelSystem.turnTurretToVisionTarget();
        } else {
          fuelSystem.setShooterRPM(4000);
          fuelSystem.turnTurretTo(0);
        }

        if (fuelSystem.ballsFired >= shotsWanted){
          fuelSystem.setShooterRPM(0);
          fuelSystem.runTransfer(0, 0);
          fuelSystem.setShooterSpeed(0);
          fuelSystem.setTurretPower(0);
          if (selDistance == 0){
            currentShooterState = SMShooting.INIT;
          } else {
            //Drive based on the user selceted input
            driveTime.reset();
            currentShooterState = SMShooting.DRIVING;
          }
        }else{
          
          if (fuelSystem.correctRPM){
            fuelSystem.runTransfer(.7, .7);
          } else {
            fuelSystem.runTransfer(0, 0);
          }
        }
        break;

      case DRIVING:
        if (driveTime.get() > 1){
          driveTrain.moveRobot(0,0);
          currentShooterState = SMShooting.INIT;
        } else {
          driveTrain.moveRobot(0.4*Math.signum(selDistance) , 0);   
        }

        break;
       
    }
    return currentShooterState;
  }

}
