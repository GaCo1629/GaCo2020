/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    public double valid;
    public double x;
    public double y;
    public double area;
    public double skew;
    public double width;
    public double height;
    public double pipeline;
    public double dimensional;
    public boolean targetVisible = false;
    private final double MIN_WIDTH = 50;
    private final double MAX_WIDTH = 370;
    private double averageLimelightY = 0;
    private double limelightYCounter = 0;
    private double cycleCounter      = 0;
    
    private String limelightName;

    public  Vision (String name) {
        limelightName = name;
    }
    public double getDistanceFromTarget() {
      double distanceToTarget;
      if(width <= MIN_WIDTH){
        distanceToTarget= 9;
      } else if (width >= MAX_WIDTH){
        distanceToTarget = 41;
      }
      else  {
        distanceToTarget = 106+ (-1.52*width) +( 0.0103*Math.pow(width, 2)) + -0.000033*Math.pow(width, 3) + 0.0000000406*Math.pow(width, 4);
      }
     return distanceToTarget;    
    }

    public void resetAverageVariables(){
      cycleCounter      = 0;
      limelightYCounter = 0;
      averageLimelightY = 0;
    }

    public void updateTarget(){
        //Set Up Tables
     NetworkTable      table = NetworkTableInstance.getDefault().getTable(limelightName);
     NetworkTableEntry tv    = table.getEntry("tv");
     NetworkTableEntry tx    = table.getEntry("tx");
     NetworkTableEntry ty    = table.getEntry("ty");
     NetworkTableEntry thor  = table.getEntry("thor");
     NetworkTableEntry ts    = table.getEntry("ts");
     
        //Set Variable to Current Values
     valid = tv.getDouble(0.0);
     
     if (valid == 1) {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        width = thor.getDouble(0.0);
        skew = ts.getDouble(0.0);
        targetVisible = true;

        limelightYCounter += y;
        cycleCounter ++;

        averageLimelightY = limelightYCounter/cycleCounter;

   
        //Smart Dashboard Display
       
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("TargetWidth", width);
        SmartDashboard.putNumber("TargetSkew", skew);

     } else {
       targetVisible = false;
     }
     SmartDashboard.putBoolean("ValidTarget?", targetVisible);
     SmartDashboard.putNumber("Average Limelight Y", averageLimelightY);

    
     
    }  

}


