/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/
package frc.robot;

import java.util.ArrayList;

public class PurePursuit {

    final double DISTANCE_TO_TRAVEL = 30;
    final double MINIMUM_DISTANCE_TO_TARGET = 0; //minimum distance to target in inches
    final double TRANSLATION_MODIFIER       = 1;
    final double TURNING_MODIFIER           = 1;

    double remainingDistanceToTravel;

    int counter;

    double axialPower;
    double lateralPower;
    double yawPower;
    
    double robotXPosition;
    double robotYPosition;
    double robotHeading;

    double distanceToXTarget;
    double distanceToYTarget;
    double distanceToTarget;
    double angleChangeRequired;

    double angleToTarget;

    Point robotLocation;

    double shortestDistance = 0;
    int closestLine;

    Point perpendicularIntercept;
    Point targetPoint;

    Line lineToFollow;

    public PurePursuit(){

    }

    public void followPath(Path pathToFollow){

    }

    //takes the robots location and uses it and the line it is on to find the point for the robot to move twords
    public Point getTargetLocation(Line followingLine){

        lineToFollow = followingLine;

        //robotLocation = robot.getRobotLocation();

        //find slope perpendicular to the line to follow that passes through the robots location to find the robots distance from the path to follow line
        perpendicularIntercept = lineToFollow.getPerpendicularIntercept(robotLocation);

        //subtract the distance to the line to follow from the total distance to find what remaining and use it to set target location
        remainingDistanceToTravel = DISTANCE_TO_TRAVEL - robotLocation.getDistanceBetween(perpendicularIntercept);

        //check to see if final point is close enough to get the next point and get the next line to follow
        if (targetPoint.getDistanceBetween(perpendicularIntercept) < remainingDistanceToTravel){
            targetPoint = followingLine.getEnd();
        }else {
            //if its not then check to see if the x values are increasing or decreasing from start to end of the line
            if (lineToFollow.lineMovingRight()) {
                //if they are increasing then move right on the line to the right the remaining distance and set that as the target point
                targetPoint = lineToFollow.getIntersectionPoint1(perpendicularIntercept, remainingDistanceToTravel);
            } else {
                //if they are decreasing then move left on the line to the left the remaining distance and set that as the target point
                targetPoint = lineToFollow.getIntersectionPoint2(perpendicularIntercept, remainingDistanceToTravel);
            }
        }
        return targetPoint;
    }

    //uses a robot location and a target point to get axial lateral and yaw values
    public void setVectors(Point targetP){

        //set variables

        //robotLocation = robot.getRobotLocation();

        robotXPosition = robotLocation.getX();
        robotYPosition = robotLocation.getY();
        robotHeading   = robotLocation.getHeading();

        //get distances to target
        distanceToXTarget = targetP.getX() - robotXPosition;
        distanceToYTarget = targetP.getY() - robotYPosition;

        distanceToTarget = Math.hypot(distanceToXTarget,distanceToYTarget);

        angleToTarget       = angleWrap180(Math.toDegrees(Math.atan2(distanceToYTarget,distanceToXTarget)));
        angleChangeRequired = angleWrap180(angleToTarget - robotHeading);

        if(Math.abs(distanceToTarget) > MINIMUM_DISTANCE_TO_TARGET) {
            axialPower    =  (distanceToYTarget / distanceToTarget) * TRANSLATION_MODIFIER;
            lateralPower  =  (distanceToXTarget / distanceToTarget) * TRANSLATION_MODIFIER;
            yawPower      =  (angleChangeRequired)                  * TURNING_MODIFIER;
        }else{
            axialPower   = 0;
            lateralPower = 0;
            yawPower     = 0;
        }

        //robot.setDriveVectors(axialPower,lateralPower,yawPower);
    }

    public int findClosestLine(Path pathToFollow){
        shortestDistance = 0;

        //robotLocation = robot.getRobotLocation();

        //create each line on the path and check how far the robot is from each one
        for(int i = 0; i < pathToFollow.getPathLineLength() - 1; i++){
            if(shortestDistance > pathToFollow.getLine(i).getDistanceFrom(robotLocation)){
                shortestDistance = pathToFollow.getLine(i).getDistanceFrom(robotLocation);
                closestLine = i;
            }
        }
        return closestLine;
    }

    private double angleWrap180(double angle){
        while(angle <= -180){
            angle += 360;
        }
        while(angle >= 180){
            angle -=360;
        }
        return angle;
    }
}