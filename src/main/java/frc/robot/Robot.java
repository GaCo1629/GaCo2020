/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  //int controllers
  private GaCoDrive pilot        = new GaCoDrive();
  private GaCoDrive copilot      = new GaCoDrive();
  private GaCoDrive minion       = new GaCoDrive(); 
  //init subsystems
  private DriveTrain    driveTrain   = new DriveTrain();
  private FuelSystem    fuelSystem   = new FuelSystem();
  //private ColorWheel    colorWheel   = new ColorWheel();
  private Climber       climber      = new Climber();
  private Vision        turretVision = new Vision("limelight-turret");
  private Auto          auto         = new Auto();
  private Controller    controller   = new Controller();

  @Override
  public void robotInit() {
    //initalize all classes
    pilot.init(0, true);
    copilot.init(1, false);
    minion.init(2, false);
    driveTrain.init(pilot, copilot, minion, controller, turretVision, fuelSystem);
    fuelSystem.init(pilot, copilot, minion, controller, turretVision, driveTrain);
    //colorWheel.init(pilot, copilot, minion);
    climber.init(pilot, copilot, minion, controller);  
    controller.init(pilot, copilot, minion);
  }
  
  @Override
  public void robotPeriodic(){
    turretVision.updateTarget();

    fuelSystem.updateVariables();
    driveTrain.readSensors();
    climber.updateValues();

    driveTrain.show();
    fuelSystem.show();
    climber.show();
  }


  @Override
  public void autonomousInit() {
    // TODO Auto-generated method stub
    super.autonomousInit();
    auto.autonomousInit();
  }

 @Override
  public void autonomousPeriodic() {
    // TODO Auto-generated method stub
    super.autonomousPeriodic();
    auto.autonomousPeriodic();
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    driveTrain.setHeading(0);
    fuelSystem.setTurretHeading(0);
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.teleopPeriodic();
    fuelSystem.teleopPeriodic();
    //colorWheel.teleopPeriodic();
    climber.teleopPeriodic();
    controller.getButtons();
  }  
}