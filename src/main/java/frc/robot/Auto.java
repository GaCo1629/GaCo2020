/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*-----------------------------*/

package frc.robot;

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


    private StartPosition selStartPosition;
 
    private NumBalls selNumBalls;
   
    private Timer timeout; // timer

// intiliaze the needed classes
    private DriveTrain    driveTrain   = new DriveTrain();
    private FuelSystem    fuelSystem   = new FuelSystem();
    private Vision        turretVision = new Vision("limelight");
   

    public void init(){
        //start postion choser
        startPosition.setDefaultOption("None", StartPosition.NONE);
        startPosition.addOption("Center", StartPosition.CENTER);
        startPosition.addOption("Far Trench", StartPosition.FAR_TRENCH);
        startPosition.addOption("Close Trench", StartPosition.CLOSE_TRENCH);
         SmartDashboard.putData("Start Position", startPosition);
         
           //numBalls choser
           numBalls.setDefaultOption("None", NumBalls.NONE);
           numBalls.addOption("Center", NumBalls.THREE);
           numBalls.addOption("Far Trench", NumBalls.SIX);
           numBalls.addOption("Close Trench", NumBalls.TEN);
           SmartDashboard.putData("number of balls", numBalls);

 
    }

    @Override
    public void autonomousInit(){
        selStartPosition = startPosition.getSelected();
        selNumBalls = numBalls.getSelected();
     

    }

    @Override
    public void autonomousPeriodic(){
        switch (selStartPosition){
            //run code for figuring out where to go based on having center postion
            case CENTER: 
            centerLogic();
            break;

            //runs logic for if we start infront of our color trence
            case CLOSE_TRENCH:
            closeLogic();
            break;

            // run logic for if we start in other teams trench
            case FAR_TRENCH:
            farLogic();
            break;

            // do nothing if no balls
            case NONE:
            //do nothing
            break;
            }

        }
       


//shows values
    @Override
    public void show(){

        startPosition.setDefaultOption("None", StartPosition.NONE);
        startPosition.addOption("Center", StartPosition.CENTER);
        startPosition.addOption("Far Trench", StartPosition.FAR_TRENCH);
        startPosition.addOption("Close Trench", StartPosition.CLOSE_TRENCH);
         SmartDashboard.putData("Start Position", startPosition);
         
           //numBalls choser
           numBalls.setDefaultOption("None", NumBalls.NONE);
           numBalls.addOption("Center", NumBalls.THREE);
           numBalls.addOption("Far Trench", NumBalls.SIX);
           numBalls.addOption("Close Trench", NumBalls.TEN);
           SmartDashboard.putData("number of balls", numBalls);
    }


    ///// AUTO FUNCTIONS \\\\\  
    ///auto logic
    //sets and does cases based on start position
    private void centerLogic(){
    switch (selNumBalls){
        
        case NONE:
        
        

    }

    }
    private void closeLogic(){
        switch (selNumBalls){

        }
    

    }
    private void farLogic(){
        switch (selNumBalls){

        }
    

    }
// AUTO MOVE


}
