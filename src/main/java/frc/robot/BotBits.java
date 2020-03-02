/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/
// dont use this class it is old
package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Public class to contain all the hardware elements (BotBits)
public class BotBits {

    public Joystick stick;
    //shooter motors
    public CANSparkMax leftShooter; 
    public CANSparkMax rightShooter;
    //left drive train
    public CANSparkMax leftDrive1;
    //right drive train 
    public CANSparkMax rightDrive1;
    // 
    public CANSparkMax turret;
    
    public TalonSRX lowerTransfer;
    public TalonSRX upperTransfer;
    public TalonSRX collecter;



    public CANEncoder encoder;
    
    final int PAD_Y = 4 ;
    final int PAD_X = 3 ;
    final int PAD_A = 1 ;
    final int PAD_B = 2 ;
    final int R_BUMPER = 6;
    final double TRANSFER_SPEED = 0.7;
    final double SHOOTER_SPEED = 0.7;
    public static final int leftShooterCANid = 20;
    public static final int rightShooterCANid = 21;
        
    public  BotBits () {
    }

    public void init() {
        stick = new Joystick(0);

        // initialize motor
        leftShooter = new CANSparkMax(leftShooterCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkMax(rightShooterCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();
        rightShooter.setInverted(true);
    
        // Encoder object created to display position/velocity values
        encoder = leftShooter.getEncoder();    

        //talon
        upperTransfer = new TalonSRX(10);
        upperTransfer.set(ControlMode.PercentOutput, 10);

        lowerTransfer = new TalonSRX(11);
        lowerTransfer.set(ControlMode.PercentOutput, 11);

        collecter = new TalonSRX(12);
        collecter.set(ControlMode.PercentOutput, 12);
    }

    public void show() {
       SmartDashboard.putNumber("Speed", encoder.getVelocity());
    }
    
    public void setShooterSpeed (double speed){
        leftShooter.set(speed);
        rightShooter.set(speed);
    }
    public void runTransfer (boolean run){
        if (run) {
            upperTransfer.set(ControlMode.PercentOutput, TRANSFER_SPEED);
            lowerTransfer.set(ControlMode.PercentOutput, TRANSFER_SPEED);
     }
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
