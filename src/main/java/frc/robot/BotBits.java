/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Public class to contain all the hardware elements (BotBits)
public class BotBits {

    public Joystick stick;
    public static final int leftShooterCANid = 11;
    public static final int rightShooterCANid = 12;
    public CANSparkMax leftShooter; 
    public CANSparkMax rightShooter;
    public CANEncoder encoder;
        
    public BotBits () {
    }

    public void init() {
        stick = new Joystick(0);

        // initialize motor
        leftShooter = new CANSparkMax(leftShooterCANid, MotorType.kBrushless);
        rightShooter = new CANSparkMax(rightShooterCANid, MotorType.kBrushless);
        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();
    
        // Encoder object created to display position/velocity values
        encoder = leftShooter.getEncoder();    
    }

    public void show() {
       SmartDashboard.putNumber("Speed", encoder.getVelocity());
    }
    
}