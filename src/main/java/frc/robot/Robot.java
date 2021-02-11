/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
  private Auto          auto         = new Auto();
  public Timer timeout = new Timer();
  
 
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
    auto.init(driveTrain, fuelSystem, turretVision);
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
    auto.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {
    auto.autonomousPeriodic();  
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
}