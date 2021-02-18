package frc.robot;

public class Step {
    public StepMode mode;
    public double speed;
    public double distance;
    public double heading;
    public double timeout;
    

    public Step(StepMode initMode, double initSpeed, double initDistance, double initHeading, double initTimeout){
        mode = initMode;
        speed = initSpeed;
        distance = initDistance;
        heading = initHeading;
        timeout = initTimeout;
    }

}
