/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.VictorSP;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.DriverStation;


// Public class to contain all the hardware elements (BotBits)
public class ColorWheel extends Subsystem{

    private GaCoDrive pilot;
    private GaCoDrive copilot;
    private GaCoDrive minion;
    private Controller controller;

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

    private double endTime = 0;
    String gameData;

    public Colors colorFlag;
    public Rot position = Rot.Init;
    
    public CRot yellowE = CRot.Init;
    public CRot redE = CRot.Init;
    public CRot blueE = CRot.Init;
    public CRot greenE = CRot.Init;


    // constructor
    public ColorWheel() {
    }

    public void init(GaCoDrive pilot, GaCoDrive copilot, GaCoDrive minion, Controller controller) {
        this.pilot   = pilot;
        this.copilot = copilot;
        this.minion  = minion;
        this.controller = controller;

        colorMotor  = new VictorSP(COLOR_MOTOR_ID);


        colorArm = new DoubleSolenoid(1,2,3);
        colorArm.set(DoubleSolenoid.Value.kReverse);
        //This line has placeholder values ^


        //Find the maximum speed at which the wheel is allowed to spin.

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        m_colorMatcher.addColorMatch(kBlackTarget);
        //initialize motor to spin wheel
        //initialize psunamatics to push out wheel spinner
        //initialize camera/color sensor
    }


    @Override 
    public void teleopInit(){
        time.reset();
        time.start();
        lastSmartColor = kBlackTarget;
        smartGetColor();

        position = Rot.Init;
        colorFlag = Colors.Black;
        wheelCount = 0;
        
        colorMotor.set(0);
        colorArm.set(DoubleSolenoid.Value.kReverse);

        stopColorArm();
    }

    //run the motor and use the sensor/camera so that the table turns 3.5 rotations
    public void turnRotations(){
        switch (position){
            case Init :
                //reset variables if needed, acitvate through a button
                SmartDashboard.putString("Wheel State", "Init");
                
                wheelCount = 0;
                
                if (controller.runColorWheelRotations){
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
                if (wheelCount < 8){
                    match = smartGetColor();
                    //Save the current color
                    if (match != prevMatch){
                        //If the robot is looking at a different color than it just was, continue
                        if ((match == kBlueTarget)){
                            wheelCount++;
                            //if the color currently being looked at is the first color, add one to wheel count
                        }
                    }
                    prevMatch = match;
                    //set the current color to the previous color
                } else {
                    colorMotor.set(0);
                    endTime = time.get() + 0.5;
                    position = Rot.Arm_Retracted;
                }
                break;

            case Arm_Retracted :
                SmartDashboard.putString("Wheel State", "Arm Retracted");
                if (time.get() >= endTime){
                    colorArm.set(DoubleSolenoid.Value.kReverse);
                    position = Rot.Init;
                }
                break;
        }
    }

    //run the motor and use the sensor/camera so that the table turns to red
    public void turnToRed(){
       switch (redE){
            case Init :
                //reset variables if needed, acitvate through a button
                SmartDashboard.putString("Color State", "Init");
                
                wheelCount = 0;
                
                if (controller.runColorWheelPosition && (colorFlag == Colors.Red)){
                    SmartDashboard.putString("Turn to rotation", "On");
                    colorArm.set(DoubleSolenoid.Value.kForward);
                    endTime = time.get() + 0.5;
                    redE    = CRot.Extend_Arm;
                }
                break;

            case Extend_Arm :
                SmartDashboard.putString("Color State", "Extend Arm");
                if (time.get() >= endTime){
                    redE = CRot.Arm_Extended;
                }
                break;

            case Arm_Extended :
                SmartDashboard.putString("Wheel State", "Arm Extended");
                firstMatch      = smartGetColor();
                prevMatch       = firstMatch;
                redE            = CRot.Color_Recieved;
                break;

            case Color_Recieved :
                SmartDashboard.putString("Wheel State", "Color Recieved");
                colorMotor.set(0.5);
                redE    = CRot.Turning;
                break;

            case Turning :
                SmartDashboard.putString("Wheel State", "Wheel Turning");
                if (wheelCount < 1){
                    match = smartGetColor();
                    //Save the current color
                    if (match != prevMatch){
                        //If the robot is looking at a different color than it just was, continue
                        if ((match == kBlueTarget)){
                            wheelCount++;
                            //if the color currently being looked at is Green, add one to wheel count
                        }
                    }
                    prevMatch = match;
                    //set the current color to the previous color
                } else {
                    colorMotor.set(0);
                    endTime = time.get() + 0.5;
                    redE    = CRot.Arm_Retracted;
                }
                break;

            case Arm_Retracted :
                SmartDashboard.putString("Wheel State", "Arm Retracted");
                if (time.get() >= endTime){
                    colorArm.set(DoubleSolenoid.Value.kReverse);
                    redE = CRot.Init;
                }
                break;
        }

    }

    //run the motor and use the sensor/camera so that the table turns to blue
    public void turnToBlue(){
        switch (blueE){
            case Init :
                //reset variables if needed, acitvate through a button
                SmartDashboard.putString("Color State", "Init");
                
                wheelCount = 0;
                
                if (controller.runColorWheelPosition && (colorFlag == Colors.Blue)){
                    SmartDashboard.putString("Turn to rotation", "On");
                    colorArm.set(DoubleSolenoid.Value.kForward);
                    endTime = time.get() + 0.5;
                    blueE   = CRot.Extend_Arm;
                }
                break;

            case Extend_Arm :
                SmartDashboard.putString("Color State", "Extend Arm");
                if (time.get() >= endTime){
                    blueE = CRot.Arm_Extended;
                }
                break;

            case Arm_Extended :
                SmartDashboard.putString("Wheel State", "Arm Extended");
                firstMatch      = smartGetColor();
                prevMatch       = firstMatch;
                blueE           = CRot.Color_Recieved;
                break;

            case Color_Recieved :
                SmartDashboard.putString("Wheel State", "Color Recieved");
                colorMotor.set(0.5);
                blueE    = CRot.Turning;
                break;

            case Turning :
                SmartDashboard.putString("Wheel State", "Wheel Turning");
                if (wheelCount < 1){
                    match = smartGetColor();
                    //Save the current color
                    if (match != prevMatch){
                        //If the robot is looking at a different color than it just was, continue
                        if ((match == kRedTarget)){
                            wheelCount++;
                            //if the color currently being looked at is Green, add one to wheel count
                        }
                    }
                    prevMatch = match;
                    //set the current color to the previous color
                } else {
                    colorMotor.set(0);
                    endTime = time.get() + 0.5;
                    blueE   = CRot.Arm_Retracted;
                }
                break;

            case Arm_Retracted :
                SmartDashboard.putString("Wheel State", "Arm Retracted");
                if (time.get() >= endTime){
                    colorArm.set(DoubleSolenoid.Value.kReverse);
                    blueE = CRot.Init;
                }
                break;
        }
    }

    //run the motor and use the sensor/camera so that the table turns to green
    public void turnToGreen(){
        switch (greenE){
            case Init :
                //reset variables if needed, acitvate through a button
                SmartDashboard.putString("Color State", "Init");
                
                wheelCount = 0;
                
                if (controller.runColorWheelPosition && (colorFlag == Colors.Green)){
                    SmartDashboard.putString("Turn to rotation", "On");
                    colorArm.set(DoubleSolenoid.Value.kForward);
                    endTime = time.get() + 0.5;
                    greenE  = CRot.Extend_Arm;
                }
                break;

            case Extend_Arm :
                SmartDashboard.putString("Color State", "Extend Arm");
                if (time.get() >= endTime){
                    greenE = CRot.Arm_Extended;
                }
                break;

            case Arm_Extended :
                SmartDashboard.putString("Wheel State", "Arm Extended");
                firstMatch      = smartGetColor();
                prevMatch       = firstMatch;
                greenE          = CRot.Color_Recieved;
                break;

            case Color_Recieved :
                SmartDashboard.putString("Wheel State", "Color Recieved");
                colorMotor.set(0.5);
                greenE    = CRot.Turning;
                break;

            case Turning :
                SmartDashboard.putString("Wheel State", "Wheel Turning");
                if (wheelCount < 1){
                    match = smartGetColor();
                    //Save the current color
                    if (match != prevMatch){
                        //If the robot is looking at a different color than it just was, continue
                        if ((match == kYellowTarget)){
                            wheelCount++;
                            //if the color currently being looked at is Green, add one to wheel count
                        }
                    }
                    prevMatch = match;
                    //set the current color to the previous color
                } else {
                    colorMotor.set(0);
                    endTime = time.get() + 0.5;
                    greenE  = CRot.Arm_Retracted;
                }
                break;

            case Arm_Retracted :
                SmartDashboard.putString("Wheel State", "Arm Retracted");
                if (time.get() >= endTime){
                    colorArm.set(DoubleSolenoid.Value.kReverse);
                    greenE = CRot.Init;
                }
                break;
        }
    }

    //run the motor and use the sensor/camera so that the table turns to yellow
    public void turnToYellow(){
        switch (yellowE){
            case Init :
                //reset variables if needed, acitvate through a button
                SmartDashboard.putString("Color State", "Init");
                
                wheelCount = 0;
                
                if (controller.runColorWheelPosition && (colorFlag == Colors.Yellow)){
                    SmartDashboard.putString("Turn to rotation", "On");
                    colorArm.set(DoubleSolenoid.Value.kForward);
                    endTime = time.get() + 0.5;
                    yellowE = CRot.Extend_Arm;
                }
                break;

            case Extend_Arm :
                SmartDashboard.putString("Color State", "Extend Arm");
                if (time.get() >= endTime){
                    yellowE = CRot.Arm_Extended;
                }
                break;

            case Arm_Extended :
                SmartDashboard.putString("Wheel State", "Arm Extended");
                firstMatch      = smartGetColor();
                prevMatch       = firstMatch;
                yellowE         = CRot.Color_Recieved;
                break;

            case Color_Recieved :
                SmartDashboard.putString("Wheel State", "Color Recieved");
                colorMotor.set(0.5);
                yellowE = CRot.Turning;
                break;

            case Turning :
                SmartDashboard.putString("Wheel State", "Wheel Turning");
                if (wheelCount < 1){
                    match = smartGetColor();
                    //Save the current color
                    if (match != prevMatch){
                        //If the robot is looking at a different color than it just was, continue
                        if ((match == kGreenTarget)){
                            wheelCount++;
                            //if the color currently being looked at is Green, add one to wheel count
                        }
                    }
                    prevMatch = match;
                    //set the current color to the previous color
                } else {
                    colorMotor.set(0);
                    endTime = time.get() + 0.5;
                    yellowE = CRot.Arm_Retracted;
                }
                break;

            case Arm_Retracted :
                SmartDashboard.putString("Wheel State", "Arm Retracted");
                if (time.get() >= endTime){
                    colorArm.set(DoubleSolenoid.Value.kReverse);
                    yellowE = CRot.Init;
                }
                break;
        }

    }

    public void stopColorArm(){
        position = Rot.Init;
        redE     = CRot.Init;
        yellowE  = CRot.Init;
        blueE    = CRot.Init;
        greenE   = CRot.Init;
        colorArm.set(DoubleSolenoid.Value.kReverse);
        colorMotor.set(0);
    }

    public void matchColor(){
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'R' :
                    colorFlag = Colors.Red;
                break;
                case 'B' :
                    colorFlag = Colors.Blue;
                break;
                case 'Y' :
                    colorFlag = Colors.Yellow;
                break;
                case 'G' :
                    colorFlag = Colors.Green;
                break;
                default :
                    colorFlag = Colors.Black;
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

        
        turnRotations();
        turnToRed();
        turnToBlue();
        turnToGreen();
        turnToYellow();

        if (minion.r3() || minion.l3()){
            if ((position != Rot.Init) && (redE != CRot.Init)){
                SmartDashboard.putString("Wheel Conflicts", "TRUE");
                stopColorArm();
            } else if ((position != Rot.Init) && (yellowE != CRot.Init)){
                SmartDashboard.putString("Wheel Conflicts", "TRUE");
                stopColorArm();
            } else if ((position != Rot.Init) && (greenE != CRot.Init)){
                SmartDashboard.putString("Wheel Conflicts", "TRUE");
                stopColorArm();
            } else if ((position != Rot.Init) && (blueE != CRot.Init)){
                SmartDashboard.putString("Wheel Conflicts", "TRUE");
                stopColorArm();
            } else {
                SmartDashboard.putString("Wheel Conflicts", "FALSE");
            }
        }

        
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




            SmartDashboard.putNumber("Red", smartColor.red);
            SmartDashboard.putNumber("Green", smartColor.green);
            SmartDashboard.putNumber("Blue", smartColor.blue);
            SmartDashboard.putNumber("Confidence", tempMatch.confidence);
            SmartDashboard.putString("Detected Color", colorString);
            SmartDashboard.putNumber("Rotations", wheelCount);
            SmartDashboard.putString("Req Color", colorFlag.toString());
        

    }
}
