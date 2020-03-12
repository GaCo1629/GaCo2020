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
  public double range;
  public boolean targetVisible = false;
  public boolean limelightIsZoomed = false;

  // Network Table Access
  NetworkTable      table ;
  NetworkTableEntry p ;
  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry thor;
  NetworkTableEntry ts  ;

  private final double MIN_WIDTH = 50;
  private final double MAX_WIDTH = 370;

  private ArrayList<Double> recentLimelightY = new ArrayList<Double>();
  private String limelightName;

  public  Vision (String name) {
    limelightName = name;
  }

  public  void  init() {
    //Set Up Tables
    table = NetworkTableInstance.getDefault().getTable(limelightName);
    p     = table.getEntry("pipeline");
    tv    = table.getEntry("tv");
    tx    = table.getEntry("tx");
    ty    = table.getEntry("ty");
    thor  = table.getEntry("thor");
    ts    = table.getEntry("ts");
  }

  public double getDistanceFromTarget() {
     return range;    
  }

  public void zoomInLimelight() {
    limelightIsZoomed = true;
  }

  public void zoomOutLimelight() {
    limelightIsZoomed = false;
  }

  public void updateTarget() {
 
    if(limelightIsZoomed){
      p.setDouble(1);
    } else {
      p.setDouble(0);
    }
     
    //Set Variable to Current Values
    targetVisible = (tv.getDouble(0.0)  == 1);
     
    if (targetVisible) {
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      width = thor.getDouble(0.0);
      skew = ts.getDouble(0.0);

      double targetWidth = width;

      if (limelightIsZoomed) {
        targetWidth *= 1.5;
      }
  
      if (targetWidth <= MIN_WIDTH) {
        targetWidth = MIN_WIDTH;
      } else if (targetWidth >= MAX_WIDTH) {
        targetWidth = MAX_WIDTH;
      }
  
      range = 106 + 
              (-1.52 * targetWidth) +
              ( 0.0103 * Math.pow(targetWidth, 2)) +
              -0.000033 * Math.pow(targetWidth, 3) + 
              0.0000000406 * Math.pow(targetWidth, 4);
    
      //Smart Dashboard Display
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("TargetWidth", width);
      SmartDashboard.putNumber("TargetSkew", skew);
      SmartDashboard.putNumber("Distance to Target Limelight", range);
    } else {
      recentLimelightY.clear();        
    }

    SmartDashboard.putBoolean("ValidTarget?", targetVisible);     
  }  
}


