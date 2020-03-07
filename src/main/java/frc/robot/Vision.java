/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

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
    private final double MIN_WIDTH = 23;
    private final double MAX_WIDTH = -5;
    private final int LIST_LENGTH_AVERAGE_Y = 10;
    public double averageLimelightY = 0;
    private double limelightCounter = 0;

    private ArrayList<Double> recentLimelightY = new ArrayList<Double>();
    
    private String limelightName;

    public  Vision (String name) {
        limelightName = name;
    }

    public double getDistanceFromTarget() {
      double distanceToTarget;
      if(width <= MIN_WIDTH){
        distanceToTarget= -5.1;
      } else if (width >= MAX_WIDTH){
        distanceToTarget = 23.1;
       }
      else  {
        distanceToTarget = -(.0000174948 * Math.pow(y, 5)) + (.00099589 * Math.pow(y, 4)) - (.022156 * Math.pow(y, 3)) + (.271437 * Math.pow(y, 2)) - (2.54782 * y) + 25.7234;
      //}
      return distanceToTarget;    
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

      //get limelight y average for the last 10 cycles
      recentLimelightY.add(0, y);
      while(recentLimelightY.size() >= LIST_LENGTH_AVERAGE_Y + 1){
        recentLimelightY.remove(LIST_LENGTH_AVERAGE_Y);
      }
      for(int i = 0; i < recentLimelightY.size(); i++){
        limelightCounter += recentLimelightY.get(i);
      }
      averageLimelightY = limelightCounter/recentLimelightY.size();
   
      //Smart Dashboard Display
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("Average of last 10 Limelight y", averageLimelightY);
      SmartDashboard.putNumber("TargetWidth", width);
      SmartDashboard.putNumber("TargetSkew", skew);

    } else {
      targetVisible = false;
      recentLimelightY.clear();        
    }
    SmartDashboard.putBoolean("ValidTarget?", targetVisible);     
  }  
}


