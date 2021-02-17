package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.firstinspires.ftc.teamcode.Utility.UltimateGoalImageProcessor;
import org.opencv.core.Mat;

//13.25 battery

@Autonomous(name = "UltimateGoalAutonRed")
public class UltimateGoalAutonRed extends FrogLinearOpMode {

    ImageResult imageResult = null;

    @Override
    public void initialize() {
        RobotHardware robot = RobotHardware.getInstance();
        robot.wobbleGoalArm.initWobble();

        Mat picture = robot.phone.getMat();
        UltimateGoalImageProcessor processor = UltimateGoalImageProcessor.getInstance();
        imageResult = processor.process(picture);
    }

    @Override
    public void run() {
        RobotHardware robot = RobotHardware.getInstance();
        robot.wobbleGoalArm.grab();

        robot.basket.resetSwiper();
        robot.basket.raiseBasket();

        telemetry.addData("number of rings:", imageResult.numberOfRings);
        telemetry.update();

       /* Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(62, 11))
                .build();

        robot.drive.followTrajectory(trajectory);*/

        robot.shooter.shooterMotor.setPower(.75);

        sleep(3000);

        robot.basket.swipe();

        sleep(750);

        robot.basket.resetSwiper();

        sleep(500);

        robot.basket.swipe();

        sleep(750);

        robot.basket.resetSwiper();

        sleep(500);

        robot.basket.swipe();

        sleep(2000);
    }
}
