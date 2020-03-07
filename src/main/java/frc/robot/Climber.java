/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// Public class to contain all the hardware elements (BotBits)
public class Climber extends Subsystem{

    private final int LEFT_LIFT_CAN_ID  = 11;
    private final int RIGHT_LIFT_CAN_ID = 18;

    private final int TOP_ENCODER_POSITION   = 560;

    //min rpm speed
    private final double STOPPED_MOTOR_SPEED = 10;

    private CANSparkMax leftLift;
    private CANSparkMax rightLift;

    private CANEncoder leftLiftEncoder;
    private CANEncoder rightLiftEncoder;

    private double leftLiftPosition;
    private double rightLiftPosition;

    private double lastLeftLiftPosition  = 0;
    private double lastRightLiftPosition = 0;

    private Timer timer = new Timer();
    private double currentTime = 0;
    private boolean firstLoop = true;

    private GaCoDrive pilot;
    private GaCoDrive copilot;
    private GaCoDrive minion;
    private Controller controller;

    //proportional, integral, derivative, forwardFeedInRPM, integralActiveZone, tolerance, angleWrapOn, name
    PIDController climberPID = new PIDController(.2, .005, 0, 0, 5, .01, false, "climber");

    // constructor
    public Climber(){
    }


    public void init(GaCoDrive pilot, GaCoDrive copilot, GaCoDrive minion, Controller controller) {
        this.pilot   = pilot;
        this.copilot = copilot;
        this.minion  = minion;
        this.controller = controller;

        leftLift  = new CANSparkMax(LEFT_LIFT_CAN_ID,  CANSparkMaxLowLevel.MotorType.kBrushless);
        rightLift = new CANSparkMax(RIGHT_LIFT_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftLift.restoreFactoryDefaults();
        rightLift.restoreFactoryDefaults();

        leftLiftEncoder  = leftLift.getEncoder();
        rightLiftEncoder = rightLift.getEncoder();
        //initialize both climbing motors
        //initalize center of gravity adjustor motor
        resetEncoders();
        timer.reset();
        timer.start();
    }

    //extnnd up to get ready to climb
    public void extendUp(){
        if(leftLiftPosition < TOP_ENCODER_POSITION){
            setPower(1);
        } else {
            setPower(0);
        }
    }

    //reset the climber to its origional position
    public void resetClimber(){
        if(firstLoop){
            timer.reset();
            firstLoop = false;
            rightLift.set(-.2);
            leftLift.set(-.2);
        }

        currentTime = timer.get();

        if(currentTime > .5){
            if(Math.abs(leftLiftEncoder.getVelocity()) < STOPPED_MOTOR_SPEED){
                leftLift.set(0);
                leftLiftEncoder.setPosition(0);
            }
            if(Math.abs(rightLiftEncoder.getVelocity()) < STOPPED_MOTOR_SPEED){
                rightLift.set(0);
                rightLiftEncoder.setPosition(0);
            }
        }
    }

    public void runClimberTest(){
        if(minion.rightTrigger()){
            setPower(1);
        } else if (minion.rightBumper()) {
            setPower(-1);
        } else {
            setPower(0);
        }
    }

    public void setPower(double power){
        leftLift.set(power);
        rightLift.set(climberPID.run(rightLiftPosition, leftLiftPosition));
    }

    public void resetEncoders(){
        leftLiftEncoder.setPosition(0);
        rightLiftEncoder.setPosition(0);
    }

    @Override
    public void teleopPeriodic() {

        if(controller.homeClimber){
            resetClimber();
        } else if(controller.autoClimberUp){
            extendUp();
            firstLoop = true;
        } else if(controller.manualClimberUp){
            setPower(1);
            firstLoop = true;
        } else if(controller.manualClimberDown){
            setPower(-1);
            firstLoop = true;
        } else {
            setPower(0);
            firstLoop = true;
        }

        if(minion.leftStick()){
            resetEncoders();
        }
        //Driver 2 - (Button) Extend Up Climber
        //Driver 2 - (Button) Climb up
        //Driver 2 (Stick/buttons) reposition robot position
    }

    public void updateValues(){
        lastLeftLiftPosition  = leftLiftPosition;
        lastRightLiftPosition = rightLiftPosition;
        leftLiftPosition  = leftLiftEncoder.getPosition();
        rightLiftPosition = rightLiftEncoder.getPosition();
    }

    public void show(){
        SmartDashboard.putNumber ("Climber Left Encoder", leftLiftPosition);
        SmartDashboard.putNumber ("Climber Right Encoder", rightLiftPosition);
        SmartDashboard.putNumber ("Climber Left Encoder Velocity", leftLiftEncoder.getVelocity());
        SmartDashboard.putNumber ("Climber Right Encoder Velocity", rightLiftEncoder.getVelocity());
    }
}
