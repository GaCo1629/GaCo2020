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
public class Climber {

    DriverStation driverStation;

    // constructor
    public Climber(){
    }


    public void init(DriverStation driverStation) {
        this.driverStation = driverStation;
        //initialize both climbing motors
        //initalize center of gravity adjustor motor
    }

    //extnnd up to get ready to climb
    public void extendUp(){

    }

    //reset the climber to its origional position
    public void reset(){

    }

    //pull down on climber with full power
    public void climb(){

    }

    public void move() {
        //Driver 2 - (Button) Extend Up Climber
        //Driver 2 - (Button) Climb up
        //Driver 2 (Stick/buttons) reposition robot position
    }
}
