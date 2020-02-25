/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo                                                    */
/*-----------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;


public class Auto extends Subsystem {

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

  


}
