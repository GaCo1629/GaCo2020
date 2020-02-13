/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 GaCo. All Rights Reserved.                              */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Line {

    private Point start;
    private Point end;
    private double slope;
    private double yIntercept;

    double tempIntercept;
    double inverseSlope;

    double pointX;
    double pointY;

    double denominatorModifier;

    double xTemp;
    double yTemp;

    //constructor
    public Line(Point start, Point end){
        //a line consists of 2 points, a start and and end
        this.start = start;
        this.end   = end;

        //check to see if the slope is 0 and if it is add .02 to it to negate divide by 0 errors
        if (Math.abs(end.getX()-end.getY()) < .001){
            denominatorModifier = .002;
        }else{
            denominatorModifier = 0;
        }

        //calculate the slope and y intercept
        slope = (end.getY()-start.getY()) / (end.getX()-start.getX() + denominatorModifier);

        //ensure that slope is not = to 0 to avoid future div by 0 errors
        if(slope == 0){
            slope += .001;
        }

        yIntercept = end.getY() - getSlope() * end.getX();
    }

    //returns the point at the start of the line
    public Point getStart(){
        return this.start;
    }

    //returns the point at the end of the line
    public Point getEnd(){
        return this.end;
    }

    //does change in Y over the change in X to find slope
    public double getSlope(){
        return slope;
    }

    //uses slope and a point to find the Y intercept
    public double getYIntercept(){
        return yIntercept;
    }

    //enter a point and return the point on the line that is the perpendicular intercpet
    public Point getPerpendicularIntercept(Point pointToIntersect){

        //find the inverse slope
        inverseSlope = -1/this.getSlope();
        //find the y intercept of the inverse slope
        tempIntercept = pointToIntersect.getY() - inverseSlope * pointToIntersect.getX();

        //find the x and y values
        pointX = (tempIntercept - this.getYIntercept()) / (this.getSlope() - inverseSlope + denominatorModifier);
        pointY = pointX * inverseSlope + tempIntercept;

        //return the intersection point
        return new Point(pointX,pointY);
    }

    //gets disatnce from a point from the closest point on the line
    public double getDistanceFrom(Point point){
        //check to see if the perpendicular intercept is not on the line
        if(this.getPerpendicularIntercept(point).getX() > this.getStart().getX() && this.getPerpendicularIntercept(point).getX() > this.getEnd().getX()
        || this.getPerpendicularIntercept(point).getX() < this.getStart().getX() && this.getPerpendicularIntercept(point).getX() < this.getEnd().getX()){

            //if it is not on the line then check to see what end point to use to see how far the point is on the line
            if(point.getDistanceBetween(point, this.getStart()) > point.getDistanceBetween(point, this.getStart())){
                //if its closer to the start point then use the start point
                return point.getDistanceBetween(point, this.getStart());
            }else{
                //otherwise it is closer to the endpoint so use it instead
                return point.getDistanceBetween(point, this.getEnd());
            }
        }else{
            //otherwise the perpendicular intercept is on the line so just get the distance between the two points
            return point.getDistanceBetween(this.getPerpendicularIntercept(point), point);
        }
    }

    //use a center point and a radius and find the points where the line intersects them
    public Point getIntersectionPoint1(Point center, double radius){
        xTemp = center.getX() + radius / Math.sqrt(1 + Math.pow(this.getSlope(),2));
        yTemp = this.getSlope() * xTemp + this.getYIntercept();

        return new Point (xTemp, yTemp);
    }

    //use a center point and a radius and find the points where the line intersects them
    public Point getIntersectionPoint2(Point center, double radius){
        xTemp = center.getX() - radius / Math.sqrt(1 + Math.pow(this.getSlope(),2));
        yTemp = this.getSlope() * xTemp + this.getYIntercept();

        return new Point (xTemp, yTemp);
    }

    //check to see if the lines starting point is to the left of its end point
    public boolean lineMovingRight(){
        if(0 < this.getEnd().getX() - this.getStart().getX()){
            return true;
        }else{
            return false;
        }
    }
}
