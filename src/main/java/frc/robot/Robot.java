/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private static final int deviceIDl = 11;
  private static final int deviceIDr = 12;
  private CANSparkMax m_lmotor; 
  private CANSparkMax m_rmotor;
  private CANEncoder m_encoder;

  public double m_setpoint = 0.25;  
  public double m_speed = 0.0;
  public boolean m_enable = false;

  final int PAD_Y = 4 ;
  final int PAD_X = 3 ;
  final int PAD_A = 1 ;
  final int PAD_B = 2 ;
  
  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // initialize motor
    m_lmotor = new CANSparkMax(deviceIDl, MotorType.kBrushless);
    m_rmotor = new CANSparkMax(deviceIDr, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_lmotor.restoreFactoryDefaults();
    m_rmotor.restoreFactoryDefaults();

    // Encoder object created to display position values
    m_encoder = m_lmotor.getEncoder();

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Setpoint", m_setpoint);
    SmartDashboard.putNumber("Speed", m_speed);
    

  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);

    
    if (m_stick.getRawButton(PAD_B)) {
      m_setpoint +=  0.002;
    }
    
    if (m_stick.getRawButton(PAD_X)) {
      m_setpoint -=  0.002;
    }

    if (m_setpoint > 1.0 ) m_setpoint = 1.0;
    if (m_setpoint < 0) m_setpoint = 0;

    SmartDashboard.putNumber("Setpoint", m_setpoint * 100);
   
    
    if (m_stick.getRawButton(PAD_Y)) {
      m_enable = true;
    } else if (m_stick.getRawButton(PAD_A)){
      m_enable = false;
    }

    if (m_enable) {
      m_lmotor.set(m_setpoint);
      m_rmotor.set(-m_setpoint);
    } else {
      m_lmotor.set(0);
      m_rmotor.set(0);     
    }



    m_speed = m_encoder.getVelocity();

    SmartDashboard.putNumber("Speed", m_speed);
  }      
}
