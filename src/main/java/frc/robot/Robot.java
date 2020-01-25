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

  private BotBits botBits = new BotBits();

  public double m_setpoint = 0.25;  
  public double m_speed = 0.0;
  public boolean m_enable = false;

  @Override
  public void robotInit() {

    botBits.init();
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Setpoint", 0);
    SmartDashboard.putNumber("Speed", 0);
  }

  @Override
  public void teleopPeriodic() {
    
    if (botBits.b()) {
      m_setpoint +=  0.002;
    }
    
    if (botBits.x()) {
      m_setpoint -=  0.002;
    }

    if (m_setpoint > 1.0 ) 
      m_setpoint = 1.0;
    if (m_setpoint < 0) m_setpoint = 0;

    SmartDashboard.putNumber("Setpoint", m_setpoint * 100);
   
    
    if (botBits.y()){
      m_enable = true;
    } else if (botBits.a()){
      m_enable = false;
    }

    if (m_enable) {
      botBits.setShooterSpeed(m_setpoint);
    
    } else {
      botBits.setShooterSpeed(0); 
    }

    botBits.show();
    
  }      
}
