package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.firstinspires.ftc.teamcode.Utility.UltimateGoalImageProcessor;
import org.opencv.core.Mat;

//@Autonomous(name = "blueCarouselAuto")
public class testXRail extends FrogLinearOpMode {
    static RobotHardware robot = null;

    ImageResult imageResult = null;
    static double shooterPower = 0;
    public void initialize() {
        robot = RobotHardware.getInstance();
    }
    @Override
    public void run() {
        robot.Xrail.liftxRail("top");

        sleep(100);

    }
}
