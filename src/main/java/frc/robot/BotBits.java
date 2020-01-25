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
    public CANSparkMax leftShooter; 
    public CANSparkMax rightShooter;
    public CANEncoder encoder;
    
    final int PAD_Y = 4 ;
    final int PAD_X = 3 ;
    final int PAD_A = 1 ;
    final int PAD_B = 2 ;
    public static final int leftShooterCANid = 11;
    public static final int rightShooterCANid = 12;
        
    public  BotBits () {
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
    
    public void setShooterSpeed (double speed){
        leftShooter.set(speed);
        rightShooter.set(-speed);
    }

    public boolean y(){
        return stick.getRawButton(PAD_Y);
    }

    public boolean x(){
        return stick.getRawButton(PAD_X);
    }

    public boolean a(){
        return stick.getRawButton(PAD_A);
    }

    public boolean b(){
        return stick.getRawButton(PAD_B);
    }
}
