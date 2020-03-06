/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/
package frc.robot;

public class Point {

    private double x;
    private double y;
    private double heading;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
        this.heading = 0.0;
    }

    public Point(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX(){
        return this.x;
    }

    public double getY(){
        return this.y;
    }

    public double getHeading(){
        return this.heading;
    }

    public double getDistanceBetween(Point p1, Point p2){
        return Math.sqrt(Math.pow((p2.getX() - p1.getX()),2) + Math.pow((p2.getY() - p1.getY()),2));
    }

    public double getDistanceBetween(Point p1){
        return Math.sqrt(Math.pow((this.getX() - p1.getX()),2) + Math.pow((this.getY() - p1.getY()),2));
    }

    public void set(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void set(double x, double y){
        this.x = x;
        this.y = y;
    }
}