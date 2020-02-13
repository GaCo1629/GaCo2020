/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

public class Path {
    private ArrayList<Point> pathPoints;
    private ArrayList<Line>  pathLines;
    private int numberOfPathPoints;
    private int numberOfPathLines;

    private int marker = 0;

    public Path(double startX, double startY){
        pathPoints.add(new Point(startX, startY));
        numberOfPathPoints = 1;
        numberOfPathLines  = 0;
    }

    public void modifyMarker(int newValue){
        marker = newValue;
    }

    public void add1Marker(){
        marker++;
    }

    public int getMarker(){
        return marker;
    }

    public int getPathPointLength(){
        return numberOfPathPoints;
    }

    public int getPathLineLength(){
        return numberOfPathLines;
    }

    public void addPoint(double X, double Y){
        pathPoints.add(new Point(X, Y));
        pathLines.add(new Line(this.getPoint(numberOfPathPoints), this.getPoint(numberOfPathPoints - 1)));
        numberOfPathPoints++;
        numberOfPathLines++;
    }

    public void getLine(){

    }

    //if 0 it will return the first point in the list
    public Point getPoint(int target){
        return pathPoints.get(target);
    }

    //if 0 it will return the first line in the list
    public Line getLine(int target){
        return pathLines.get(target);
    }
}
