/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

// Public class to contain all the hardware elements (BotBits)
public class GaCoDrive{

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

    final int R_STICK   = 9;
    final int R_STICK_V = 5 ;
    final int R_STICK_H = 4 ;

    final int L_STICK   = 10;
    final int L_STICK_V = 1 ;
    final int L_STICK_H = 0 ;

    final int L3 = 9;
    final int R3 = 10;

    private boolean isLogitech;

    //constructor
    public  GaCoDrive () {
    }

    //initialzie all controllers and buttons
    public void init(final int portNum, final boolean isLogitech) {
        stick = new Joystick(portNum);
        this.isLogitech = isLogitech;
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
        if(isLogitech){
            return (stick.getPOV() == 0);
        } else {
            return (stick.getPOV() == 270);
        }
    }

    public boolean dpadDown(){
        if(isLogitech){
            return (stick.getPOV() == 180);
        } else {
            return (stick.getPOV() == 90);
        }  
    }

    public boolean dpadLeft(){
        if(isLogitech){
            return (stick.getPOV() == 270);
        } else {
            return (stick.getPOV() == 180);
        }  
    }   

    public boolean dpadRight(){
        if(isLogitech){
            return (stick.getPOV() == 90);
        } else {
            return (stick.getPOV() == 0);
        }  
    }    

    public boolean dpadUpLeft(){
        if(isLogitech){
            return (stick.getPOV() == 315);
        } else {
            return (stick.getPOV() == 225);
        }  
    }  

    public boolean dpadUpRight(){
        if(isLogitech){
            return (stick.getPOV() == 45);
        } else {
            return (stick.getPOV() == 315);
        }  
    }

    public boolean dpadDownLeft(){
        if(isLogitech){
            return (stick.getPOV() == 225);
        } else {
            return (stick.getPOV() == 135);
        }  
    }

    public boolean dpadDownRight(){
        if(isLogitech){
            return (stick.getPOV() == 135);
        } else {
            return (stick.getPOV() == 45);
        }  
    }

    public double getDpad(){
        return stick.getPOV();
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

    public boolean l3(){
        return stick.getRawButton(L3);
    }

    public boolean r3(){
        return stick.getRawButton(R3);
    }
    
    
}
