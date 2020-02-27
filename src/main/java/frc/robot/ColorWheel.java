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

    GaCoDrive gaCoDrive;

    private VictorSP colorMotor;
    //private Solenoid colorArm;
    private DoubleSolenoid colorArm;
    private final int COLOR_MOTOR_ID  = 0;
    
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    //private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kRedTarget = ColorMatch.makeColor(0.49, 0.37, 0.114);
    //private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    private final Color kYellowTarget = ColorMatch.makeColor(0.31, 0.56, 0.113);
    private final Color kBlackTarget = ColorMatch.makeColor(0, 0, 0);

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
    private Color lastSmartColor;
    private Color match;
    private Color firstMatch;
    private Color prevMatch;
    private Color smartColor;
    private ColorMatchResult tempMatch;

    public String origColor;
    public String curColor;
    public String colorString;
    public int wheelCount;
    
    Timer time = new Timer();
    Timer lookTime = new Timer();

    private boolean firstTime = false;
    private double endTime = 0;
    private double curTime = 0;
    private int flagTurn = 0;
    private int yellow = 0;
    private int green = 0;
    private int blue = 0;
    private int red = 0;
    public int colorFlag = 0;
    String gameData;

    public Rot position = Rot.Init;
    public CRot yellowE = CRot.Init;
    public CRot redE = CRot.Init;
    public CRot blueE = CRot.Init;
    public CRot greenE = CRot.Init;


    // constructor
    public ColorWheel() {
    }

    public void init(GaCoDrive gaCoDrive) {
        this.gaCoDrive = gaCoDrive;

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
        m_colorMatcher.addColorMatch(kBlackTarget);
        //initialize motor to spin wheel
        //initialize psunamatics to push out wheel spinner
        //initialize camera/color sensor

        time.reset();
        time.start();
        firstTime = false;
        lastSmartColor = kBlackTarget;
        smartGetColor();

    }

    //run the motor and use the sensor/camera so that the table turns 3.5 rotations
    public void turnRotations(){
        /*wheelCount = 0;
        //Reset the wheel count for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        detectedColor   = smartGetColor();
        firstMatch      = m_colorMatcher.matchClosestColor(detectedColor);
        //put out the color arm and save the first color
        colorMotor.set(maxPower);
        //Turn on the motor to max power (im currently looking into a way to set a max RPM value)
        if (wheelCount < 7){
            detectedColor = smartGetColor();
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
        } else {
            flagTurn = 0;
        }
        //when 3.5 rotations have occured, exit the loop
        if (flagTurn == 0){
        colorMotor.set(0);
        colorArm.set(DoubleSolenoid.Value.kReverse);
        }
        //turn the motor off and retract */
        switch (position){
            case Init :
                //reset variables if needed, acitvate through a button
                SmartDashboard.putString("Wheel State", "Init");
                
                wheelCount = 0;
                
                if (gaCoDrive.l3()){
                    SmartDashboard.putString("Turn to rotation", "On");
                    colorArm.set(DoubleSolenoid.Value.kForward);
                    endTime = time.get() + 0.5;
                    position = Rot.Extend_Arm;
                }
                break;

            case Extend_Arm :
                SmartDashboard.putString("Wheel State", "Extend Arm");
                if (time.get() >= endTime){
                    position = Rot.Arm_Extended;
                }
                break;

            case Arm_Extended :
                SmartDashboard.putString("Wheel State", "Arm Extended");
                firstMatch      = smartGetColor();
                prevMatch       = firstMatch;
                position        = Rot.Color_Recieved;
                break;

            case Color_Recieved :
                SmartDashboard.putString("Wheel State", "Color Recieved");
                colorMotor.set(0.5);
                position = Rot.Turning;
                break;

            case Turning :
                SmartDashboard.putString("Wheel State", "Wheel Turning");
                if (wheelCount < 7){
                    match = smartGetColor();
                    //Save the current color
                    if (match != prevMatch){
                        //If the robot is looking at a different color than it just was, continue
                        if ((match == kBlueTarget)/* && (match.confidence >= 0.95)*/){
                            wheelCount++;
                            //if the color currently being looked at is the first color, add one to wheel count
                        }
                    }
                    prevMatch = match;
                    //set the current color to the previous color
                } else {
                    colorMotor.set(0);
                    position = Rot.Arm_Retracted;
                    endTime = time.get() + 0.5;
                }
                break;

            case Arm_Retracted :
                SmartDashboard.putString("Wheel State", "Arm Retracted");
                colorArm.set(DoubleSolenoid.Value.kReverse);
                curTime = time.get();
                if (time.get() >= endTime){
                    position = Rot.Init;
                }
                break;
        }
    }
/*
    //run the motor and use the sensor/camera so that the table turns to red
    public void turnToRed(){
        wheelCount = 0;
        //re sets wheelCount for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        colorMotor.set(maxPower * 0.5);
        //extends the color arm and sets the power to half the max RPM
        while (wheelCount < 1){
            detectedColor = smartGetColor();
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
        //re sets wheelCount for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        colorMotor.set(maxPower * 0.5);
        //extends the color arm and sets the power to half the max RPM
        while (wheelCount < 1){
            detectedColor = smartGetColor();
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
        //re sets wheelCount for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        colorMotor.set(maxPower * 0.5);
        //extends the color arm and sets the power to half the max RPM
        while (wheelCount < 1){
            detectedColor = smartGetColor();
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
        /*wheelCount = 0;
        //re sets wheelCount for use
        colorArm.set(DoubleSolenoid.Value.kForward);
        colorMotor.set(maxPower * 0.5);
        //extends the color arm and sets the power to half the max RPM
        if (wheelCount < 1){
            detectedColor = smartGetColor();
            match = m_colorMatcher.matchClosestColor(detectedColor);
            if (match.color == kGreenTarget){
                wheelCount++;
            }
        } else {
            yellow = 0;
            //if the color is equal to the necessary ofset, exit the flag loop
        }
        if (yellow == 0){
            colorMotor.set(0);
            colorArm.set(DoubleSolenoid.Value.kReverse);
        }
        switch (yellowE){
            case Init:
                SmartDashboard.putString("Wheel State", "Init");
                if (wheelCount != 0){
                    wheelCount = 0;
                }
                if (driverStation.rightTrigger()){
                    SmartDashboard.putString("Turn to:", "Yellow");
                    startTime = time.get();
                    yellowE = CRot.Extend_Arm;
                }
            break;
            case Extend_Arm:
                colorArm.set(DoubleSolenoid.Value.kForward);
            break;
            case Arm_Extended:
            
            break;
            case Color_Recieved:
            
            break;
            case Turning:
            
            break;
            case Arm_Retracted:
            
            break;
        }*/

    }

    public void matchColor(){
        //gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'R' :
                    colorFlag = 1;
                break;
                case 'B' :
                    colorFlag = 2;
                break;
                case 'Y' :
                    colorFlag = 3;
                break;
                case 'G' :
                    colorFlag = 4;
                break;
                default :
                    colorFlag = -1;
                break;
            }
        } else {
        //Code for no data received yet
        }
    }

    private Color smartGetColor(){
         smartColor = m_colorSensor.getColor();
         tempMatch = m_colorMatcher.matchClosestColor(smartColor);

         if (tempMatch.confidence >= 0.96 ){
            lastSmartColor = tempMatch.color;
         }
         SmartDashboard.putString("Color Seen", lastSmartColor.toString());
         return lastSmartColor;
    }


    public void teleopPeriodic() {
        //Driver 2 - (Button) extend out mechanism
        //Driver 2 - (Button) Spin wheel 3.5 rotations
        //Driver 2 - (Button) Spin wheel to red
        //Driver 2 - (Button) Spin wheel to blue
        //Driver 2 - (Button) Spin wheel to green
        //Driver 2 - (Button) Spin wheel to yellow

        


        if (gaCoDrive.x()){
            colorArm.set(DoubleSolenoid.Value.kForward);
        } else if (gaCoDrive.y()){
            colorArm.set(DoubleSolenoid.Value.kReverse);
        }

        if (gaCoDrive.leftTrigger()){

            SmartDashboard.putString("Turn to:", "Test"); //Red

            /*if(firstTime == false){
                firstTime = true;
                startTime = time.get();
                colorMotor.set(.5);
                colorArm.set(DoubleSolenoid.Value.kForward);
            }


            if(time.get() - startTime > 5) {
                colorMotor.set(0);
                colorArm.set(DoubleSolenoid.Value.kReverse);
                SmartDashboard.putString("Turn to:", "Test");
            }*/

            
            
            
        } /* else if (gaCoDrive.a()){
            SmartDashboard.putString("Turn to:", "");

        } else if (gaCoDrive.b()){
            SmartDashboard.putString("Turn to:", "");

        } */else if (gaCoDrive.rightTrigger()){
            SmartDashboard.putString("Turn to:", "Yellow");
            yellow = 1;
        } else if (gaCoDrive.leftBumper()){
            colorMotor.set(0.5);
            SmartDashboard.putString("colorMotorOn", "Foward");
        } else if (gaCoDrive.rightBumper()){
            colorMotor.set(-0.5);
            SmartDashboard.putString("colorMotorOn", "Reverse");
        } else {
            //colorMotor.set(0);
            //SmartDashboard.putString("colorMotorOn", "Off");
        }
        
        /*if (yellow == 1){
            turnToYellow();
        }

        if (gaCoDrive.l3()){
            SmartDashboard.putString("Turn to rotation", "On");
            flagTurn = 1;
        } else {
            SmartDashboard.putString("Turn to rotation", "Off");
        }*/
        turnRotations();

        
            if (lastSmartColor == kBlueTarget){
                colorString = "Blue";
            } else if (lastSmartColor == kYellowTarget){
                colorString = "Yellow";
            } else if (lastSmartColor == kRedTarget){
                colorString = "Red";
            } else if (lastSmartColor == kGreenTarget){
                colorString = "Green";
            } else if (lastSmartColor == kBlackTarget){
                colorString = "Black";
            }

            /*if (colorFlag != 0){
                matchColor();
            }*/

            SmartDashboard.putNumber("Red", smartColor.red);
            SmartDashboard.putNumber("Green", smartColor.green);
            SmartDashboard.putNumber("Blue", smartColor.blue);
            SmartDashboard.putNumber("Confidence", tempMatch.confidence);
            SmartDashboard.putString("Detected Color", colorString);
            SmartDashboard.putNumber("Rotations", wheelCount);
        

    }
}
