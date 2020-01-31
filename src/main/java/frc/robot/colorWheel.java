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
public class ColorWheel {

    DriverStation driverStation;

    // constructor
    public ColorWheel() {
    }

    public void init(DriverStation driverStation) {
        this.driverStation = driverStation;

        //initialize motor to spin wheel
        //initialize psunamatics to push out wheel spinner
        //initialize camera/color sensor
    }

    //run the motor and use the sensor/camera so that the table turns 3.5 rotations
    public void turnRotations(){

    }

    //run the motor and use the sensor/camera so that the table turns to red
    public void turnToRed(){

    }

    //run the motor and use the sensor/camera so that the table turns to blue
    public void turnToBlue(){
        
    }

    //run the motor and use the sensor/camera so that the table turns to green
    public void turnToGreen(){
        
    }

    //run the motor and use the sensor/camera so that the table turns to yellow
    public void turnToYellow(){
        
    }


    public void move() {
        //Driver 2 - (Button) extend out mechanism
        //Driver 2 - (Button) Spin wheel 3.5 rotations
        //Driver 2 - (Button) Spin wheel to red
        //Driver 2 - (Button) Spin wheel to blue
        //Driver 2 - (Button) Spin wheel to green
        //Driver 2 - (Button) Spin wheel to yellow
    }
}
