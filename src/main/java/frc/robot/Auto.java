/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*-----------------------------*/
/*
package frc.robot;

import frc.robot.AutoMode;
import frc.robot.NumBalls;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;



public class Auto extends TimedRobot {
    //selctable choosers to tell auto
    private SendableChooser <AutoMode> startPosition = new SendableChooser<>();
    private SendableChooser <NumBalls> numBalls = new SendableChooser<>();


    private int flag = 0;
    private double startLeftDriveEncoder = 0;
    private double startRightDriveEncoder = 0;
   
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
            } else {
                fuelSystem.runTransfer(0, 0);
            }
            if(fuelSystem.ballsInIndex < 1 || timeout.get() > 4){
                flag++;
            }
            if(flag == 3){
                if(timeout.get() < 6 && fuelSystem.ballsInIndex > 0){
                    fuelSystem.runTransfer(1, 1);
                } else {
                    fuelSystem.runTransfer(0, 0);
                    flag++;
                }
            }
            if(flag == 4){
                
            }

        }
        case SIX:
        break;

        case TEN:
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
*/
