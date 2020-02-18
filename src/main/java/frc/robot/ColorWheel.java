/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.I2C;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.lang.Math;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.VictorSP;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


// Public class to contain all the hardware elements (BotBits)
public class ColorWheel {

    DriverStation driverStation;

    private VictorSP colorMotor;
    //private Solenoid colorArm;
    private DoubleSolenoid colorArm;
    private final int COLOR_MOTOR_ID  = 0;
    
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private final double colorMotorDiameter = 4;
    private final double colorWheelDiameter = 32;
    private final double fullRPM = 21020;
    private double colorMotorCircumference;
    private double colorWheelCircumference;
    private double maxRPM;
    private double maxPower;
    private double pastTime = 0;

    private Color detectedColor;
    private Color color;
    private ColorMatchResult match;
    private ColorMatchResult firstMatch;
    private ColorMatchResult prevMatch;

    public String origColor;
    public String curColor;
    public String colorString;
    public int wheelCount;
    
    Timer time = new Timer();

    private boolean firstTime = false;
    private double startTime = 0;
    private int flagTurn = 0;
    private int yellow = 0;
    private int green = 0;
    private int blue = 0;
    private int red = 0;


    // constructor
    public ColorWheel() {
    }

    public void init(DriverStation driverStation) {
        this.driverStation = driverStation;

        colorMotor  = new VictorSP(COLOR_MOTOR_ID);


        colorArm = new DoubleSolenoid(1,2,3);
        colorArm.set(DoubleSolenoid.Value.kReverse);
        //This line has placeholder values ^

        colorMotorCircumference = colorMotorDiameter * Math.PI;
        colorWheelCircumference = colorWheelDiameter * Math.PI;
        maxRPM      = (60 * colorWheelCircumference)/colorMotorCircumference;
        maxPower    = /*(maxRPM)/fullRPM;*/ 0.5;

        //Find the maximum speed at which the wheel is allowed to spin.

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        //initialize motor to spin wheel
        //initialize psunamatics to push out wheel spinner
        //initialize camera/color sensor

        time.reset();
        firstTime = false;

    }

    //run the motor and use the sensor/camera so that the table turns 3.5 rotations
    public void turnRotations(){
        wheelCount = 0;
        //Reset the wheel count for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        detectedColor   = m_colorSensor.getColor();
        firstMatch      = m_colorMatcher.matchClosestColor(detectedColor);
        //put out the color arm and save the first color
        colorMotor.set(maxPower);
        //Turn on the motor to max power (im currently looking into a way to set a max RPM value)
        while (wheelCount < 7){
            detectedColor = m_colorSensor.getColor();
            match = m_colorMatcher.matchClosestColor(detectedColor);
            //Save the current color
            if (match != prevMatch){
                //If the robot is looking at a different color than it just was, continue
                if (match.color == firstMatch.color){
                    wheelCount++;
                    //if the color currently being looked at is the first color, add one to wheel count
                }
            }
            prevMatch = match;
            //set the current color to the previous color
        }
        //when 3.5 rotations have occured, exit the loop
        colorMotor.set(0);
        colorArm.set(DoubleSolenoid.Value.kReverse);
        //turn the motor off and retract 
    }
/*
    //run the motor and use the sensor/camera so that the table turns to red
    public void turnToRed(){
        wheelCount = 0;
        //re sets wheelFount for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        colorMotor.set(maxPower * 0.5);
        //extends the color arm and sets the power to half the max RPM
        while (wheelCount < 1){
            detectedColor = m_colorSensor.getColor();
            match = m_colorMatcher.matchClosestColor(detectedColor);
            if (match.color == kBlueTarget){
                wheelCount++;
                //if the color is equal to the necessary ofset, exit the loop
            }
        }
        colorMotor.set(0);
        colorArm.set(DoubleSolenoid.Value.kReverse);

    }

    //run the motor and use the sensor/camera so that the table turns to blue
    public void turnToBlue(){
        wheelCount = 0;
        //re sets wheelFount for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        colorMotor.set(maxPower * 0.5);
        //extends the color arm and sets the power to half the max RPM
        while (wheelCount < 1){
            detectedColor = m_colorSensor.getColor();
            match = m_colorMatcher.matchClosestColor(detectedColor);
            if (match.color == kRedTarget){
                wheelCount++;
                //if the color is equal to the necessary ofset, exit the loop
            }
        }
        colorMotor.set(0);
        colorArm.set(DoubleSolenoid.Value.kReverse);
    }

    //run the motor and use the sensor/camera so that the table turns to green
    public void turnToGreen(){
        wheelCount = 0;
        //re sets wheelFount for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        colorMotor.set(maxPower * 0.5);
        //extends the color arm and sets the power to half the max RPM
        while (wheelCount < 1){
            detectedColor = m_colorSensor.getColor();
            match = m_colorMatcher.matchClosestColor(detectedColor);
            if (match.color == kYellowTarget){
                wheelCount++;
                //if the color is equal to the necessary ofset, exit the loop
            }
        }
        colorMotor.set(0);
        colorArm.set(DoubleSolenoid.Value.kReverse);
    }
*/
    //run the motor and use the sensor/camera so that the table turns to yellow
    public void turnToYellow(){
        wheelCount = 0;
        //re sets wheelFount for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        colorMotor.set(maxPower * 0.5);
        //extends the color arm and sets the power to half the max RPM
        if (wheelCount < 1){
            detectedColor = m_colorSensor.getColor();
            match = m_colorMatcher.matchClosestColor(detectedColor);
            if (match.color == kGreenTarget){
                wheelCount++;
                //if the color is equal to the necessary ofset, exit the flag loop
                yellow = 0;
            }
        }
        if (yellow == 0){
            colorMotor.set(0);
            colorArm.set(DoubleSolenoid.Value.kReverse);
        }
    }



    public void teleopPeriodic() {
        //Driver 2 - (Button) extend out mechanism
        //Driver 2 - (Button) Spin wheel 3.5 rotations
        //Driver 2 - (Button) Spin wheel to red
        //Driver 2 - (Button) Spin wheel to blue
        //Driver 2 - (Button) Spin wheel to green
        //Driver 2 - (Button) Spin wheel to yellow

        


        if (driverStation.x()){
            colorArm.set(DoubleSolenoid.Value.kForward);
        } else if (driverStation.y()){
            colorArm.set(DoubleSolenoid.Value.kReverse);
        }

        if (driverStation.leftTrigger()){

            SmartDashboard.putString("Turn to:", "Test");

            if(firstTime == false){
                firstTime = true;
                startTime = time.get();
                colorMotor.set(.5);
                colorArm.set(DoubleSolenoid.Value.kForward);
            }


            if(time.get() - startTime > 5) {
                colorMotor.set(0);
                colorArm.set(DoubleSolenoid.Value.kReverse);
                SmartDashboard.putString("Turn to:", "");
            }

            
            
            
        } /* else if (driverStation.a()){
            SmartDashboard.putString("Turn to:", "");

        } else if (driverStation.b()){
            SmartDashboard.putString("Turn to:", "");

        } */else if (driverStation.rightTrigger()){
            SmartDashboard.putString("Turn to:", "Yellow");
            yellow = 1;
        } else if (driverStation.leftBumper()){
            colorMotor.set(0.5);
            SmartDashboard.putString("colorMotorOn", "Foward");
        } else if (driverStation.rightBumper()){
            colorMotor.set(-0.5);
            SmartDashboard.putString("colorMotorOn", "Reverse");
        } else {
            colorMotor.set(0);
            SmartDashboard.putString("colorMotorOn", "Off");
        }
        
        if (yellow == 1){
            turnToYellow();
        }

        if (driverStation.l3()){
            SmartDashboard.putString("Turn to rotation", "On");
        } else {
            SmartDashboard.putString("Turn to rotation", "Off");
        }

        {
            detectedColor = m_colorSensor.getColor();
            match = m_colorMatcher.matchClosestColor(detectedColor);
            if (match.color == kBlueTarget){
                colorString = "Blue";
            } else if (match.color == kYellowTarget){
                colorString = "Yellow";
            } else if (match.color == kRedTarget){
                colorString = "Red";
            } else if (match.color == kGreenTarget){
                colorString = "Green";
            }

            SmartDashboard.putNumber("Red", detectedColor.red);
            SmartDashboard.putNumber("Green", detectedColor.green);
            SmartDashboard.putNumber("Blue", detectedColor.blue);
            SmartDashboard.putNumber("Confidence", match.confidence);
            SmartDashboard.putString("Detected Color", colorString);
        }
    }
}
