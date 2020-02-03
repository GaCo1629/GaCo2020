/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.lang.Math;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Public class to contain all the hardware elements (BotBits)
public class ColorWheel {


    DriverStation driverStation;

    private CANSparkMax colorMotor;
    private CANEncoder colorMotorEncoder;
    //private Solenoid colorArm;
    private DoubleSolenoid colorArm;
    private final int colorMotorCANid  = 26;

    private final double colorMotorDiameter = 6;
    private final double colorWheelDiameter = 32;
    private double colorMotorCircumference;
    private double colorWheelCircumference;
    private double maxRPM;
    private double maxPower;
    public double color;
    public double curColor;
    public int wheelCount;
    



    // constructor
    public ColorWheel() {
    }

    public void init(DriverStation driverStation) {
        this.driverStation = driverStation;

        colorMotor  = new CANSparkMax(colorMotorCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        colorMotor.restoreFactoryDefaults();

        colorMotorEncoder = colorMotor.getEncoder();
        /*colorArm = new Solenoid();
        *This line depends on which line the piston is plugged into ^
        */
        colorArm = new DoubleSolenoid(1,2,3);
        colorArm.set(DoubleSolenoid.Value.kReverse);
        //This line has placeholder values ^

        colorMotorCircumference = colorMotorDiameter * Math.PI;
        colorWheelCircumference = colorWheelDiameter * Math.PI;
        maxRPM = (60 * colorMotorCircumference)/colorWheelCircumference;
        maxPower = (maxRPM)/5676;

        //Find the maximum speed at which the wheel is allowed to spin.

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
