/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

// Public class to contain all the hardware elements (BotBits)
public class DriverStation {

    //CONTROLLERS ARE PROGRAMMED IN XBOX MODE
    //set the switch on the back of the logitech 310 controller to X
    
    private Joystick stick;
    
    final int PAD_Y = 4 ;
    final int PAD_X = 3 ;
    final int PAD_A = 1 ;
    final int PAD_B = 2 ;

    final int R_BUMPER = 6;
    final int L_BUMPER = 5;

    final int R_TRIGGER = 3;
    final int L_TRIGGER = 2;

    final int R_STICK   = 12;
    final int R_STICK_V = 5 ;
    final int R_STICK_H = 4 ;

    final int L_STICK   = 11;
    final int L_STICK_V = 1 ;
    final int L_STICK_H = 0 ;

    //constructor
    public  DriverStation () {
    }

    //initialzie all controllers and buttons
    public void init(int portNum) {
        stick = new Joystick(portNum);
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
        return (stick.getRawAxis(R_TRIGGER) > .5);
    }

    public boolean leftTrigger(){
        return (stick.getRawAxis(L_TRIGGER) > .5);
    }

    public double leftTriggerAnalog(){
        return stick.getRawAxis(L_TRIGGER);
    }

    public double rightTriggerAnalog(){
        return stick.getRawAxis(R_TRIGGER);
    }

    public boolean dpadUp(){
        return (stick.getPOV() == 0);
    }

    public boolean dpadDown(){
        return (stick.getPOV() == 180);
    }

    public boolean dpadLeft(){
        return (stick.getPOV() == 270);
    }

    public boolean dpadRight(){
        return (stick.getPOV() == 90);
    }

    public double getleftStickX(){
        return stick.getRawAxis(L_STICK_H);
    }
    
    public double getLeftStickY(){
        //flipped y so joystick gives positive when pushed forwards
        return -stick.getRawAxis(L_STICK_V);
   }

    public double getRightStickX(){
        return stick.getRawAxis(R_STICK_H);
}

    public double getRightStickY(){
        //flipped y so joystick gives positive when pushed forwards
        return -stick.getRawAxis(R_STICK_V);
}
    public boolean leftStick(){
     return stick.getRawButton(L_STICK);
    }

    public boolean rightStick(){
        return stick.getRawButton(R_STICK);
       }

    
    
}
