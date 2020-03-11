/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.NumBalls;
import frc.robot.AutoMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  //int controllers
  private GaCoDrive pilot        = new GaCoDrive();
  private GaCoDrive copilot      = new GaCoDrive();
  private GaCoDrive minion       = new GaCoDrive(); 
  private Controller controller  = new Controller();

  //init subsystems
  private DriveTrain    driveTrain   = new DriveTrain();
  private FuelSystem    fuelSystem   = new FuelSystem();
  private ColorWheel    colorWheel   = new ColorWheel();
  private Climber       climber      = new Climber();
  private Vision        turretVision = new Vision("limelight-turret");
  //private Auto          auto         = new Auto();
  public Timer timeout = new Timer();
  
  private AutoMode selAutoMode = AutoMode.NONE;
  
  // private NumBalls selNumBalls;
  private SendableChooser <AutoMode> autoMode = new SendableChooser<>();
  // private SendableChooser <NumBalls> numBalls = new SendableChooser<>();


  @Override
  public void robotInit() {

    //initalize all classes
    pilot.init(0, true);
    copilot.init(1, false);
    minion.init(2, false);
    controller.init(pilot, copilot, minion);

    //initalize all subsystems
    turretVision.init();
    driveTrain.init(controller, turretVision, fuelSystem);
    fuelSystem.init(controller, turretVision, driveTrain);
    colorWheel.init(controller);
    climber.init(controller);  

    //set up autonomus options
    autoMode.setDefaultOption("simple shoot", AutoMode.SIMPLE_SHOOT);
    autoMode.addOption("simple shoot", AutoMode.SIMPLE_SHOOT);
    autoMode.addOption("Smart shoot", AutoMode.SMART_SHOOT);
    autoMode.addOption("none", AutoMode.NONE);
    SmartDashboard.putData("auto mode", autoMode);
     
    //numBalls choser
   // numBalls.setDefaultOption("three", NumBalls.THREE);
   // numBalls.addOption("three", NumBalls.THREE);
   // numBalls.addOption("six", NumBalls.SIX);
   // numBalls.addOption("ten", NumBalls.TEN);
   // SmartDashboard.putData("number of balls", numBalls);

   SmartDashboard.putString("selAutoMode", ".---");
  
  }
  
  @Override
  public void robotPeriodic(){    
    
    // Do all system wide monitoring here
    controller.readInputs();
    turretVision.updateTarget();
    driveTrain.readSensors();
    fuelSystem.updateVariables();
    climber.updateValues();

    driveTrain.show();
    fuelSystem.show();
    climber.show();
  }


  @Override
  public void autonomousInit() {
    driveTrain.autonomousInit();
    fuelSystem.autonomousInit();
    colorWheel.autonomousInit();

    // Determine the desired Autonomous Action
    selAutoMode = autoMode.getSelected();
    currentShooterState = SMShooting.INIT;

    //selNumBalls = numBalls.getSelected();
    fuelSystem.setBallsInRobot(3);

    switch (selAutoMode){

      case NONE:
      SmartDashboard.putString("selAutoMode", "None selecter");
      break;

      case SIMPLE_SHOOT:
      SmartDashboard.putString("selAutoMode", "simple shoot selected");
      shootNow =true;
      shotsWanted = 2;
      break;

      case SMART_SHOOT:
      SmartDashboard.putString("selAutoMode", "smart shoot selected");
      shootNow =true;
      shotsWanted = 2;
      break;
    }
  }

  @Override
  public void autonomousPeriodic() {
    
    switch (selAutoMode){

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

    }
  }

  @Override
  public void teleopInit() {
    driveTrain.teleopInit();
    fuelSystem.teleopInit();
    colorWheel.teleopInit();
    climber.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.teleopPeriodic();
    fuelSystem.teleopPeriodic();
    colorWheel.teleopPeriodic();
    climber.teleopPeriodic();
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
   SHOOTING
  }

  public SMShooting currentShooterState = SMShooting.INIT;

  public SMShooting runSimpleShooter(){
    
    switch (currentShooterState){
      case INIT:
        //check for time to shoot
        if (shootNow){
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
          currentShooterState = SMShooting.INIT;
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
       
    }
    return currentShooterState;
  }
  //////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  public SMShooting runSmartShooter(){
    
    switch (currentShooterState){
      case INIT:
        //check for time to shoot
        if (shootNow){
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
          fuelSystem.runTransfer(.9, .9);
          currentShooterState = SMShooting.SHOOTING;
        } else {
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
          currentShooterState = SMShooting.INIT;
        }else{
          
          if (fuelSystem.correctRPM){
            fuelSystem.runTransfer(.9, .9);
          } else {
            fuelSystem.runTransfer(0, 0);
          }
        }
        break;
       
    }
    return currentShooterState;
  }


  private void simpleShoot(int shots){
  
    
   
  }
 
}