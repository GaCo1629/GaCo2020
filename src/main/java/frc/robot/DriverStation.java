/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Public class to contain all the hardware elements (BotBits)
public class DriverStation {

    
    public Joystick stick;
    
    final int PAD_Y = 4 ;
    final int PAD_X = 3 ;
    final int PAD_A = 1 ;
    final int PAD_B = 2 ;
    final int R_BUMPER = 6;

    //constructor
    public  DriverStation () {
    }

    //initialzie all controllers and buttons
    public void init() {
        stick = new Joystick(0);
    
   

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

    public boolean rightBumper(){
        return stick.getRawButton(R_BUMPER);
    }
    
}
