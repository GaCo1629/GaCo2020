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
    public double limelightName;

    public  Vision () {

    }


    public void init(String limelightName) {

    }


    public void updateVision(){
     //Set Up Tables
     NetworkTable table = NetworkTableInstance.getDefault().getTable("limelightName");
     NetworkTableEntry tv = table.getEntry("tv");
     NetworkTableEntry tx = table.getEntry("tx");
     NetworkTableEntry ty = table.getEntry("ty");
     NetworkTableEntry thor = table.getEntry("thor");
     NetworkTableEntry ts = table.getEntry("ts");
     
     //Set Variable to Current Values
     valid = tv.getDouble(0.0);
     x = tx.getDouble(0.0);
     y = ty.getDouble(0.0);
     width = thor.getDouble(0.0);
     skew = ts.getDouble(0.0);

     //Smart Dashboard Display
     SmartDashboard.putNumber("ValidTarget?", valid);
     SmartDashboard.putNumber("LimelightX", x);
     SmartDashboard.putNumber("LimelightY", y);
     SmartDashboard.putNumber("TargetWidth", width);
     SmartDashboard.putNumber("TargetSkew", skew);

    }  

}


