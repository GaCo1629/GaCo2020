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
    final int L_BUMPER = 5;
    final int R_TRIGGER = 8;
    final int L_TRIGGER = 7;

    final int R_STICK =12;
    final int R_STICK_V = 4 ;
    final int R_STICK_H = 3 ;


    final int L_STICK =11;
    final int L_STICK_V = 2 ;
    final int L_STICK_H = 1 ;

    final int DPAD_UP= 13;
    final int DPAD_DOWN= 15;
    final int DPAD_LEFT= 14;
    final int DPAD_RIGHT= 16;



        


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

    public boolean leftBumper(){
        return stick.getRawButton(L_BUMPER);
    }

    public boolean rightTrigger(){
        return stick.getRawButton(R_TRIGGER);
    }

    public boolean leftTrigger(){
        return stick.getRawButton(L_TRIGGER);
    }

    public boolean dpadUp(){
        return stick.getRawButton(DPAD_UP);
    }

    public boolean dpadDown(){
        return stick.getRawButton(DPAD_DOWN);
    }

    public boolean dpadLeft(){
        return stick.getRawButton(DPAD_LEFT);
    }

    public boolean dpadRight(){
        return stick.getRawButton(DPAD_RIGHT);
    }

    public double getleftStickX(){
        return stick.getRawAxis(L_STICK_H);
    }
    
    public double getLeftStickY(){
        return stick.getRawAxis(L_STICK_V);
   }

    public double getRightStickX(){
        return stick.getRawAxis(R_STICK_H);
}

    public double getRightStickY(){
        return stick.getRawAxis(R_STICK_V);
}
    public boolean leftStick(){
     return stick.getRawButton(L_STICK);
    }

    public boolean rightStick(){
        return stick.getRawButton(R_STICK);
       }

    
    
}
