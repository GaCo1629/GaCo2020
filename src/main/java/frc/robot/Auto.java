/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*-----------------------------*/

package frc.robot;
import frc.robot.EndPosition;
import frc.robot.StartPosition;
import frc.robot.NumBalls;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Auto extends Subsystem {
    //selctable choosers to tell auto
    private SendableChooser <StartPosition> startPosition = new SendableChooser<>();
    private SendableChooser <NumBalls> numBalls = new SendableChooser<>();
    private SendableChooser <EndPosition> endPosition = new SendableChooser<>();

    private DriveTrain    driveTrain   = new DriveTrain();
    private FuelSystem    fuelSystem   = new FuelSystem();
    private Vision        turretVision = new Vision("limelight");
    private PurePursuit   purePursuit  = new PurePursuit();
    
    @Override
    public void autonomousInit() {
     
       
    }

    @Override
    public void autonomousPeriodic(){

    }

    @Override
    public void show(){
        
    }


    ///// AUTO FUNCTIONS\\\\\  


}
