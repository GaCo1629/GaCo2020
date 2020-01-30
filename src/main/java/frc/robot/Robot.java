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

  private driveTrain     driveTrain     = new driveTrain();
  private fuelSystem     fuelSystem     = new fuelSystem();
  private colorWheel     colorWheel     = new colorWheel();
  private climber        climber        = new climber();

  @Override
  public void robotInit() {

    //initalize all classes
    driveTrain.init();
    fuelSystem.init();
    colorWheel.init();
    climber.init();
    
  }

  @Override
  public void teleopPeriodic() {

    driveTrain.move();
    fuelSystem.move();
    colorWheel.move();
    climber.move();
    
  }      
}
