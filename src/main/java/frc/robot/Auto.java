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
import edu.wpi.first.wpilibj.Timer;



public class Auto extends Subsystem {
    //selctable choosers to tell auto
    private SendableChooser <StartPosition> startPosition = new SendableChooser<>();
    private SendableChooser <NumBalls> numBalls = new SendableChooser<>();
    private SendableChooser <EndPosition> endPosition = new SendableChooser<>();

    private StartPosition selStartPosition;
    private EndPosition selEndPosition;
    private NumBalls selNumBalls;
   
    private Timer timeout; // timer

    private DriveTrain    driveTrain   = new DriveTrain();
    private FuelSystem    fuelSystem   = new FuelSystem();
    private Vision        turretVision = new Vision("limelight");
    private PurePursuit   purePursuit  = new PurePursuit();

    public void init(){
        //start postion choser
        startPosition.setDefaultOption("None", StartPosition.NONE);
        startPosition.addOption("Center", StartPosition.CENTER);
        startPosition.addOption("Far Trench", StartPosition.FAR_TRENCH);
        startPosition.addOption("Close Trench", StartPosition.CLOSE_TRENCH);
         SmartDashboard.putData("Start Position", startPosition);

         //end  postion choser
        endPosition.setDefaultOption("None", EndPosition.NONE);
        endPosition.addOption("Center", EndPosition.CENTER);
        endPosition.addOption("Far Trench", EndPosition.FAR_TRENCH);
        endPosition.addOption("Close Trench", EndPosition.ClOSE_TRENCH);
        SmartDashboard.putData("End Position", endPosition);

          //numBalls choser
          numBalls.setDefaultOption("None", NumBalls.NONE);
          numBalls.addOption("Center", NumBalls.THREE);
          numBalls.addOption("Far Trench", NumBalls.SIX);
          numBalls.addOption("Close Trench", NumBalls.TEN);
          SmartDashboard.putData("number of balls", numBalls);
  

    }
    

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

    private void driveForward(double distance, double timeOut){

    }


}
