/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.driveTrain;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private driverStation  driverStation  = new driverStation();
  private driveTrain     driveTrain     = new driveTrain();
  private fuelSystem     fuelSystem     = new fuelSystem();
  private colorWheel     colorWheel     = new colorWheel();
  private climber        climber        = new climber();

  @Override
  public void robotInit() {

    //initalize all classes
    driverStation.init();
    driveTrain.init(driverStation);
    fuelSystem.init(driverStation);
    colorWheel.init(driverStation);
    climber.init(driverStation);
    
  }

  @Override
  public void teleopPeriodic() {

    driveTrain.move();
    fuelSystem.move();
    colorWheel.move();
    climber.move();
    
  }      
}
