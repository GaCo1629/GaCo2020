/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Controller extends Subsystem{

    //IMPORTANT - stick buttons for the non logitech controllers are reversed, left is the bottom button and right is the top one

    GaCoDrive pilot;
    GaCoDrive copilot;
    GaCoDrive minion;

    private boolean lastCopilotX           = false;
    private boolean lastMinionRightTrigger = false;
    private boolean lastMinionRightBumper  = false;


    public boolean fireAll                         = false;
    public boolean fireOne                         = false;
    public boolean runUpBalls                      = false;
    public boolean runTransfer                     = false;
    public boolean reverseTransfer                 = false;
    public boolean collectorOn                     = false;
    public boolean collectorReverse                = false;
    public boolean collectorUp                     = false;
    public boolean collectorDown                   = false;
    public boolean toggleAutoVision                = false;
    public boolean setShooterRPM3400               = false;
    public boolean setShooterRPM3800               = false;
    public boolean setShooterRPM4200               = false;
    public boolean setShooterRPM4600               = false;
    public boolean shooterRPMPlus100               = false;
    public boolean shooterRPMMinus100              = false;
    public boolean longRangeShooterDefult          = false;
    public boolean turretTargetMinusPoint1         = false;
    public boolean turretTargetPlusPoint1          = false;
    public boolean turretTargetPlus3               = false;
    public boolean turretTargetMinus3              = false;
    public boolean resetTurretHeading              = false;
    public boolean automatedRPMPlus25              = false;
    public boolean automatedRPMMinus25             = false;
    public boolean turretSetFieldCentricNegative30 = false;
    public boolean turretSetFieldCentricNegative15 = false;
    public boolean turretSetFieldCentric0          = false;
    public boolean turretSetFieldCentricPositive15 = false;
    public boolean turretSetFieldCentricPositive30 = false;

    public boolean runColorWheelRotations          = false;
    public boolean runColorWheelPosition           = false;
    public boolean runColorWheelLeftManual         = false;
    public boolean runColorWheelRightManual        = false;

    public boolean autoClimberUp                   = false;
    public boolean manualClimberUp                 = false;
    public boolean manualClimberDown               = false;
    public boolean homeClimber                     = false;

    public boolean resetRobotHeading               = false;
    public boolean powerMode                       = false;
    public boolean slowMode                        = false;
    //35

    //constructor
    public  Controller () {
    }

    //initalize hardware for the drive train
    public void init(GaCoDrive pilot, GaCoDrive copilot, GaCoDrive minion){
        this.pilot      = pilot;
        this.copilot    = copilot;
        this.minion     = minion;
    }

    public void getButtons(){
        fireAll                         = false;
        fireOne                         = false;
        runUpBalls                      = false;
        runTransfer                     = false;
        reverseTransfer                 = false;
        collectorOn                     = false;
        collectorReverse                = false;
        collectorUp                     = false;
        collectorDown                   = false;
        toggleAutoVision                = false;
        setShooterRPM3400               = false;
        setShooterRPM3800               = false;
        setShooterRPM4200               = false;
        setShooterRPM4600               = false;
        shooterRPMPlus100               = false;
        shooterRPMMinus100              = false;
        automatedRPMPlus25              = false;
        automatedRPMMinus25             = false;
        longRangeShooterDefult          = false;
        turretTargetMinusPoint1         = false;
        turretTargetPlusPoint1          = false;
        turretTargetPlus3               = false;
        turretTargetMinus3              = false;
        resetTurretHeading              = false;
        turretSetFieldCentricNegative30 = false;
        turretSetFieldCentricNegative15 = false;
        turretSetFieldCentric0          = false;
        turretSetFieldCentricPositive15 = false;
        turretSetFieldCentricPositive30 = false;
    
        runColorWheelRotations          = false;
        runColorWheelPosition           = false;
        runColorWheelLeftManual         = false;
        runColorWheelRightManual        = false;

        autoClimberUp                   = false;
        manualClimberUp                 = false;
        manualClimberDown               = false;
        homeClimber                     = false;
    
        resetRobotHeading               = false;
        powerMode                       = false;
        slowMode                        = false;

    // =============================================================
    // Pilot Buttons
    // =============================================================    
        if(pilot.leftTrigger()){
            collectorReverse = true;
            collectorDown = true;
        }
  
        if(pilot.leftBumper()){
            powerMode = true;
        }
  
        if(pilot.a()){
            manualClimberDown = true;
        }
  
        if(pilot.x()){

        }
  
        if(pilot.b()){

        }
  
        if(pilot.y()){
            manualClimberUp = true;
        }
  
        if(pilot.rightTrigger()){
            collectorOn = true;
            collectorDown = true;
        }
      
        if(pilot.rightBumper()){
            slowMode = true;
        }
  
        if(pilot.dpadUp()){

        }
  
        if(pilot.dpadUpRight()){

        }
  
        if(pilot.dpadRight()){

        }
  
        if(pilot.dpadDownRight()){
  
        }
  
        if(pilot.dpadDown()){
  
        }
  
        if(pilot.dpadDownLeft()){
  
        }
  
        if(pilot.dpadLeft()){

        }
  
        if(pilot.dpadUpLeft()){

        }
  
        if(pilot.leftStick()){
        
        }
  
        if(pilot.rightStick()){
  
        }
  
        // =============================================================
        // Copilot Buttons
        // =============================================================    
        if(copilot.leftTrigger()){
            fireAll = true;
        }

        if(copilot.leftBumper()){
            fireOne = true;
        }

        if(copilot.a()){
            runUpBalls = true;
        }

        if(copilot.x() && !lastCopilotX){
            toggleAutoVision = true;
        }

        if(copilot.b()){
            runTransfer = true;
        }

        if(copilot.y()){
            reverseTransfer = true;
        }

        if(copilot.rightTrigger()){
            collectorOn = true;
            collectorDown = true;
        }
        
        if(copilot.rightBumper()){
            collectorReverse = true;
            collectorDown = true;
        }

        if(copilot.dpadUp()){
            turretSetFieldCentric0 = true;
        }

        if(copilot.dpadUpRight()){
            turretSetFieldCentricPositive15 = true;
        }

        if(copilot.dpadRight()){
            turretSetFieldCentricPositive30 = true;
        }

        if(copilot.dpadDownRight()){

        }

        if(copilot.dpadDown()){

        }

        if(copilot.dpadDownLeft()){

        }

        if(copilot.dpadLeft()){
            turretSetFieldCentricNegative30 = true;
        }

        if(copilot.dpadUpLeft()){
            turretSetFieldCentricNegative15 = true;
        }

        if(copilot.rightStick()){

        }

        //The right stick button is used as a modifier button for the minion controller

        // =============================================================
        // Minion Buttons
        // ============================================================= 

        //change buttons to be secondary controls if the right stick button is pressed
        if(!copilot.leftStick()){

            if(minion.leftTrigger()){
                setShooterRPM3400 = true;
            }

            if(minion.leftBumper()){
                setShooterRPM3800 = true;
            }

            if(minion.a()){
                setShooterRPM4200 = true;
            }

            if(minion.x()){
                setShooterRPM4600 = true;
            }

            if(minion.b()){
                longRangeShooterDefult = true;
            }

            if(minion.y()){
                
            }

            if(minion.rightTrigger() && !lastMinionRightTrigger){
                shooterRPMMinus100 = true;
            }

            if(minion.rightBumper() && !lastMinionRightBumper){
                shooterRPMPlus100 = true;
            }

            if(minion.dpadUp()){
                collectorUp = true;
            }

            if(minion.dpadRight()){
                turretTargetPlusPoint1 = true;
            }

            if(minion.dpadDown()){
                collectorDown = true;
            }

            if(minion.dpadLeft()){
                turretTargetMinusPoint1 = true;
            }

            if(minion.leftStick()){

            }

            if(minion.rightStick()){

            }

        } else {

            if(minion.leftTrigger()){
                autoClimberUp = true;

            }

            if(minion.leftBumper()){
                resetRobotHeading = true;
            }

            if(minion.a()){
                manualClimberUp = true;
            }

            if(minion.x()){
                resetTurretHeading = true;
            }

            if(minion.b()){
                manualClimberDown = true;
            }

            if(minion.y()){
                homeClimber = true;
            }

            if(minion.rightTrigger() && !lastMinionRightTrigger){
                automatedRPMMinus25 = true;
            }

            if(minion.rightBumper() && !lastMinionRightBumper){
                automatedRPMPlus25 = true;
            }

            if(minion.dpadUp()){

            }

            if(minion.dpadRight()){
                turretTargetPlus3 = true;
            }

            if(minion.dpadDown()){
            
            }

            if(minion.dpadLeft()){
                turretTargetMinus3 = true;
            }

            if(minion.leftStick()){
                runColorWheelLeftManual = true;
            }

            if(minion.rightStick()){
                runColorWheelRightManual = true;
            }
        }

        lastCopilotX           = copilot.x();
        lastMinionRightTrigger = minion.rightTrigger();
        lastMinionRightBumper  = minion.rightBumper();
    }
	
}
