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
  private Controller controller  = new Controller();

  //init subsystems
  private DriveTrain    driveTrain   = new DriveTrain();
  private FuelSystem    fuelSystem   = new FuelSystem();
  private ColorWheel    colorWheel   = new ColorWheel();
  private Climber       climber      = new Climber();
  private Vision        turretVision = new Vision("limelight-turret");
  private Auto          auto         = new Auto();
  

  @Override
  public void robotInit() {

    //initalize all classes
    pilot.init(0, true);
    copilot.init(1, false);
    minion.init(2, false);
    controller.init(pilot, copilot, minion);

    //initalize all subsystems
    driveTrain.init(controller, turretVision, fuelSystem);
    fuelSystem.init(controller, turretVision, driveTrain);
    colorWheel.init(controller);
    climber.init(controller);  
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

    auto.show();
  }


  @Override
  public void autonomousInit() {
    // TODO Auto-generated method stub
    //super.autonomousInit();
    driveTrain.autonomousInit();
    super.autonomousInit();
    auto.init(driveTrain, fuelSystem, turretVision);
    auto.autonomousInit();
  }

 @Override
  public void autonomousPeriodic() {
    // TODO Auto-generated method stub
   // super.autonomousPeriodic();
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
    controller.readInputs();
    
    driveTrain.teleopPeriodic();
    fuelSystem.teleopPeriodic();
    colorWheel.teleopPeriodic();
    climber.teleopPeriodic();
  }  
}