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



public class Auto extends TimedRobot {
    //selctable choosers to tell auto
    private SendableChooser <StartPosition> startPosition = new SendableChooser<>();
    private SendableChooser <NumBalls> numBalls = new SendableChooser<>();


    private StartPosition selStartPosition;
 
    private NumBalls selNumBalls;
   
    public Timer timeOut; // timer

// intiliaze the needed classes
    private DriveTrain    driveTrain;
    private FuelSystem    fuelSystem;
    private Vision        turretVision;
   

    public void init(DriveTrain driveTrain, FuelSystem fuelSystem, Vision turretVision){
        //start postion choser
        this.driveTrain = driveTrain;
        this.fuelSystem = fuelSystem;
        this.turretVision = turretVision;
        show();
    }

    @Override
    public void autonomousInit(){
        selStartPosition = startPosition.getSelected();
        selNumBalls = numBalls.getSelected();
     

    }

    @Override
    public void autonomousPeriodic(){
        fuelSystem.autoAimSpeed();
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
    
    public void show(){

        startPosition.setDefaultOption("None", StartPosition.NONE);
        startPosition.addOption("Center", StartPosition.CENTER);
        startPosition.addOption("Far Trench", StartPosition.FAR_TRENCH);
        startPosition.addOption("Close Trench", StartPosition.CLOSE_TRENCH);
         SmartDashboard.putData("Start Position", startPosition);
         
           //numBalls choser
           numBalls.setDefaultOption("None", NumBalls.NONE);
           numBalls.addOption("three", NumBalls.THREE);
           numBalls.addOption("six", NumBalls.SIX);
           numBalls.addOption("ten", NumBalls.TEN);
           SmartDashboard.putData("number of balls", numBalls);
    }


    ///// AUTO FUNCTIONS \\\\\  
    ///auto logic
    //sets and does cases based on start position
    private void centerLogic(){
    switch (selNumBalls){
        
        case NONE:
        timeOut.reset();
        fuelSystem.runTransfer(1, 1);
        Timer.delay(4);
        fuelSystem.runTransfer(0, 0);
        Timer.delay(4);
        break;

        case THREE:
        //should set speed to max if no vision and shoot all with vision if possible
        timeOut.reset();
        if (!turretVision.targetVisible){
            fuelSystem.setShooterRPM(5500);
        }
        fuelSystem.autoFireAll(5000);
        

        break;

        case SIX:
        timeOut.reset();
        break;

        case TEN:
        timeOut.reset();

        break;
      
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
