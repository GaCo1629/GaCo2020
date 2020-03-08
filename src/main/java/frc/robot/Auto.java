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
    private int flag = 0;
   
    private Timer timeout; // timer

// intiliaze the needed classes
    private DriveTrain    driveTrain;
    private FuelSystem    fuelSystem;
    private Vision        turretVision;
   

    public void init(DriveTrain driveTrain, FuelSystem fuelSystem, Vision turretVision){
        //start postion choser
        timeout = new Timer();
        this.driveTrain = driveTrain;
        this.fuelSystem = fuelSystem;
        this.turretVision = turretVision;
        flag = 0;
        show();
    }

    @Override
    public void autonomousInit(){
        selStartPosition = startPosition.getSelected();
        selNumBalls = numBalls.getSelected();
        fuelSystem.setBallsInRobot(3);
     

    }

    @Override
    public void autonomousPeriodic(){
        //fuelSystem.runTransfer(1, 1);
        //Timer.delay(2);
        //fuelSystem.runTransfer(0, 0);
        timeout.reset();
        timeout.start();

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

        startPosition.setDefaultOption("Center", StartPosition.CENTER);
        startPosition.addOption("Center", StartPosition.CENTER);
        startPosition.addOption("Far Trench", StartPosition.FAR_TRENCH);
        startPosition.addOption("Close Trench", StartPosition.CLOSE_TRENCH);
         SmartDashboard.putData("Start Position", startPosition);
         
           //numBalls choser
           numBalls.setDefaultOption("three", NumBalls.THREE);
           numBalls.addOption("three", NumBalls.THREE);
           numBalls.addOption("six", NumBalls.SIX);
           numBalls.addOption("ten", NumBalls.TEN);
           SmartDashboard.putData("number of balls", numBalls);
           SmartDashboard.putNumber("auto flag", flag);

    }


    ///// AUTO FUNCTIONS \\\\\  
    ///auto logic
    //sets and does cases based on start position
    private void centerLogic(){
    switch (selNumBalls){
        
        case NONE:

        break;

        case THREE:
        fuelSystem.turnOffVision();

        if(flag == 0){
            timeout.reset();
            fuelSystem.setShooterRPM(4000);
            fuelSystem.shooterOnRPM();
            fuelSystem.turnTurretTo(-90);
            if(fuelSystem.correctRPM && fuelSystem.correctTurretHeading){
                flag ++;
                fuelSystem.turnTurretTo(fuelSystem.getTurretHeading());
            }
        }
        if(flag == 1){
            fuelSystem.toggleVision();
            flag++;
        }
        if(flag == 2){
            timeout.reset();
            if(turretVision.targetVisible){
                    fuelSystem.setShooterRPM(fuelSystem.getShooterRPM(turretVision.getDistanceFromTarget()));
                } else {
                    fuelSystem.shooterOnRPM();
                }
            if(fuelSystem.readyToShoot){
                fuelSystem.runTransfer(1, 1);
            }
            if(fuelSystem.ballsInIndex < 1){
                flag ++;
            }
            if(timeout.get() > 4){
                if(timeout.get() < 6){
                    fuelSystem.runTransfer(1, 1);
                }
            }
        }
        case SIX:
        break;

        case TEN:
        break;
       // timeout.hasElapsed(3);
      //  for (int i =0 ; i < 3; i++ ){
       // if (fuelSystem.getShooterRPM()  <= 1150 || fuelSystem.getShooterRPM()  >= 950 ){
        //    fuelSystem.runTransfer(.75, .75);
      //  }
       // i++;
      //  }
     //   fuelSystem.runTransfer(0,0);
       // fuelSystem.setShooterRPM(0);
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
