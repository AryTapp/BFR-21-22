package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.UltimateGoalImageProcessor;
import org.opencv.core.Mat;

@Autonomous(name = "pictureAuton")
public class pictureAuton extends FrogLinearOpMode {
    @Override
    public void initialize() {

    }

    @Override
    public void run() {
        RobotHardware robot = RobotHardware.getInstance();
        telemetry.addData("oink:", "before");
        telemetry.update();
        Mat picture = robot.phone.getMat();
        telemetry.addData("oink:", "after");
        telemetry.update();

        UltimateGoalImageProcessor processor = UltimateGoalImageProcessor.getInstance();
        processor.process(picture);
    }

}
