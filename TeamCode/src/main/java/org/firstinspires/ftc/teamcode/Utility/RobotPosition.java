package org.firstinspires.ftc.teamcode.Utility;

public class RobotPosition {
    public double x;
    public double y;
    public double rot;


    public RobotPosition(double xIn, double yIn, double rotIn){
        x = xIn;
        y = yIn;
        rot = rotIn;

    }

    public RobotPosition(){
        x = 0;
        y = 0;
        rot = 0;
    }

    public static RobotPosition subract(RobotPosition left, RobotPosition right){
        return new RobotPosition(left.x - right.x, left.y - right.y, left.rot - right.rot);
    }


}
