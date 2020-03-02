/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/
package frc.robot;

// Is this FTC example or frc code?

public class PurePursuit{

    private DriveTrain driveTrain;

    private final double DISTANCE_TO_TRAVEL         = 30;
    private final double MINIMUM_DISTANCE_TO_TARGET = .5; //minimum distance to target in feet
    private final double AXIAL_MODIFIER       = 1;
    private final double YAW_MODIFIER         = 1;
    private final double MAX_MOTOR_POWER      = 0.2;

    private double remainingDistanceToTravel;

    private Line currentLine;

    private double axialPower;
    private double yawPower;
    
    private double robotXPosition;
    private double robotYPosition;
    private double robotHeading;

    private double distanceToXTarget;
    private double distanceToYTarget;
    private double distanceToTarget;
    private double angleChangeRequired;

    private double angleToTarget;

    private double shortestDistance = 0;
    private int closestLine;

    private Point perpendicularIntercept;
    private Point targetPoint;

    public PurePursuit(){

    }

    public void init(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }

    public void followPath(Path pathToFollow){

        if(pathToFollow.getPathLineLength() > 0){

            currentLine = pathToFollow.getLine(pathToFollow.getMarker());
        
            Point target = getTargetLocation(currentLine);

            setVectors(target);

            if(robotAt(target)){
                if(pathToFollow.getMarker() < pathToFollow.getPathLineLength() - 1){
                    pathToFollow.add1Marker();
                }
            }
        }
    }

    //takes the robots location and uses it and the line it is on to find the point for the robot to move twords
    public Point getTargetLocation(Line followingLine){

        //robotLocation = robot.getRobotLocation();

        //find slope perpendicular to the line to follow that passes through the robots location to find the robots distance from the path to follow line
        perpendicularIntercept = followingLine.getPerpendicularIntercept(driveTrain.robotLocation);

        //subtract the distance to the line to follow from the total distance to find what remaining and use it to set target location
        remainingDistanceToTravel = DISTANCE_TO_TRAVEL - driveTrain.robotLocation.getDistanceBetween(perpendicularIntercept);

        //check to see if final point is close enough to get the next point and get the next line to follow
        if ((followingLine.getEnd()).getDistanceBetween(perpendicularIntercept) < remainingDistanceToTravel){
            targetPoint = followingLine.getEnd();
        } else {
            //if its not then check to see if the x values are increasing or decreasing from start to end of the line
            if (followingLine.lineMovingRight()) {
                //if they are increasing then move right on the line to the right the remaining distance and set that as the target point
                targetPoint = followingLine.getIntersectionPoint1(perpendicularIntercept, remainingDistanceToTravel);
            } else {
                //if they are decreasing then move left on the line to the left the remaining distance and set that as the target point
                targetPoint = followingLine.getIntersectionPoint2(perpendicularIntercept, remainingDistanceToTravel);
            }
        }
        return targetPoint;
    }

    //uses a robot location and a target point to get axial lateral and yaw values
    public void setVectors(Point targetP){

        //get need robot location variables
        robotXPosition = driveTrain.x;
        robotYPosition = driveTrain.y;
        robotHeading   = driveTrain.robotHeading;

        //get distances to target
        distanceToXTarget = targetP.getX() - robotXPosition;
        distanceToYTarget = targetP.getY() - robotYPosition;

        //get distance to target and absolute angle to target
        distanceToTarget = Math.hypot(distanceToXTarget,distanceToYTarget);
        angleToTarget    = angleWrap180(Math.toDegrees(Math.atan2(distanceToYTarget,distanceToXTarget)));
        
        //get the required angle change and wrap it at 90 so it will move backwards if it is closer that way 
        angleChangeRequired = angleWrap90(angleToTarget - robotHeading);

        //make sure it is outside the distance tolerance
        if(Math.abs(distanceToTarget) > MINIMUM_DISTANCE_TO_TARGET) {

            //check to see if it should move forwards or backwards
            if(angleWrap90(angleToTarget - robotHeading) == angleWrap180(angleToTarget - robotHeading)){
                axialPower =  (distanceToYTarget / distanceToTarget) * AXIAL_MODIFIER;
            } else {
                axialPower = -(distanceToYTarget / distanceToTarget) * AXIAL_MODIFIER;
            }

            yawPower =  (angleChangeRequired) * YAW_MODIFIER;

        }else{
            axialPower   = 0;
            yawPower     = 0;
        }

        //calculate left and right motor powers
        double left  = axialPower + yawPower;
        double right = axialPower - yawPower;

        //scale them down so that the maxium of left/right is equal to a max motor power val
        double max = Math.max(Math.abs(left), Math.abs(right));

        if(max > MAX_MOTOR_POWER){
            left  = (left * MAX_MOTOR_POWER) / max;
            right = (left * MAX_MOTOR_POWER) / max;
        }        

        //robot.setDriveVectors(axialPower,lateralPower,yawPower);
    }

    public int findClosestLine(Path pathToFollow){
        shortestDistance = 0;
        closestLine      = 0;

        //check each line on the path and check how far the robot is from each one
        for(int i = 0; i < pathToFollow.getPathLineLength() - 1; i++){
            if(shortestDistance > pathToFollow.getLine(i).getDistanceFrom(driveTrain.robotLocation)){
                shortestDistance = pathToFollow.getLine(i).getDistanceFrom(driveTrain.robotLocation);
                closestLine = i;
            }
        }
        return closestLine;
    }

    private double angleWrap180(double angle){
        while(angle <= -180){
            angle += 360;
        }
        while(angle > 180){
            angle -=360;
        }
        return angle;
    }

    private double angleWrap90(double angle){
        while(angle <= -90){
            angle += 180;
        }
        while(angle > 90){
            angle -=180;
        }
        return angle;
    }

    public boolean robotAt(Point location){

        if(driveTrain.robotLocation.getDistanceBetween(location) < MINIMUM_DISTANCE_TO_TARGET){
            return true;
        } else {
            return false;
        }
    }
}