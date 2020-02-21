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
  private DriverStation pilot        = new DriverStation();
  private DriverStation copilot      = new DriverStation();
  private DriverStation minion       = new DriverStation(); 
  private DriveTrain    driveTrain   = new DriveTrain();
  private FuelSystem    fuelSystem   = new FuelSystem();
  private ColorWheel    colorWheel   = new ColorWheel();
  private Climber       climber      = new Climber();
  private Vision        turretVision = new Vision("limelight");
  private PurePursuit   purePursuit  = new PurePursuit();

  @Override
  public void robotInit() {
    //initalize all classes
    pilot.init(0, true);
    copilot.init(1, false);
    minion.init(2, false);
    driveTrain.init(pilot, turretVision, fuelSystem);
    fuelSystem.init(pilot, minion, turretVision, driveTrain);
    colorWheel.init(copilot);
    purePursuit.init(driveTrain);
    //climber.init(driverStation);  
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
    colorWheel.teleopPeriodic();
    //climber.teleopPeriodic();
  }  
  
  @Override
  public void robotPeriodic(){
    turretVision.updateTarget();

    fuelSystem.updateVariables();
    driveTrain.updateVariables();

    driveTrain.show();
    fuelSystem.show();
  }
}
