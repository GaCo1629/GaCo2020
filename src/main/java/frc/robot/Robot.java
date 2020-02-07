/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {

  private DriverStation  pilot          = new DriverStation();
  private DriverStation  copilot        = new DriverStation();
  private DriveTrain     driveTrain     = new DriveTrain();
  private FuelSystem     fuelSystem     = new FuelSystem();
  private ColorWheel     colorWheel     = new ColorWheel();
  private Climber        climber        = new Climber();


  @Override
  public void robotInit() {

    //initalize all classes
    pilot.init(0);
    copilot.init(1);
    driveTrain.init(pilot);
    fuelSystem.init(pilot);
    colorWheel.init(copilot);
    //climber.init(driverStation);
    
  }

  @Override
  public void teleopPeriodic() {

    driveTrain.teleopPeriodic();
    fuelSystem.teleopPeriodic();
    colorWheel.teleopPeriodic();
    //climber.teleopPeriodic();
    
  }      
}
