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

//13.1-13.8 battery

@Autonomous(name = "UltimateGoalAutonRed")
public class UltimateGoalAutonRed extends FrogLinearOpMode {

    static RobotHardware robot = null;


    ImageResult imageResult = null;
    static double shooterPower = 0.70;
    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();
        robot.wobbleGoalArm.initWobble();

        Mat picture = robot.phone.getMat();
        UltimateGoalImageProcessor processor = UltimateGoalImageProcessor.getInstance();
        imageResult = processor.process(picture);
    }

    @Override
    public void run() {
        robot.wobbleGoalArm.grab();

        robot.basket.resetSwiper();
        robot.basket.raiseBasket();

        telemetry.addData("number of rings:", imageResult.numberOfRings);
        telemetry.update();

        Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(40, 16))
                .build();

        Trajectory trajectory2 = robot.drive.trajectoryBuilder(new Pose2d(40, 16))
                .strafeTo(new Vector2d(63, -8))
                .build();

        robot.drive.followTrajectory(trajectory);

        robot.shooter.shooterMotor.setPower(shooterPower);

        robot.drive.followTrajectory(trajectory2);
/*
        Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(62, -8))
                .build();

        robot.drive.followTrajectory(trajectory);
*/

        shootThreeRings();

        double xOffset = -1 + 62;
        double yOffset = -24 - 8;
        double xParkOffset = 1;

        if(imageResult.numberOfRings == 1){
            xOffset += 24;
            yOffset += 24;
            xParkOffset -= 24;
        }
        else if (imageResult.numberOfRings == 4){
            xOffset += 48;
            yOffset += 0;
            xParkOffset -= 48;
        }
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(new Pose2d(63, -8))
                .strafeTo(new Vector2d(xOffset, yOffset))
                .build();

        robot.drive.followTrajectory(trajectory3);

        robot.wobbleGoalArm.lowerArm();
        sleep(500);
        robot.wobbleGoalArm.release();

        sleep(1000);

        Trajectory trajectory4 = robot.drive.trajectoryBuilder(new Pose2d(xOffset, yOffset))
                .strafeTo(new Vector2d(xParkOffset + xOffset, yOffset))
                .build();

        robot.drive.followTrajectory(trajectory4);
    }

    void shootThreeRings(){
/*        robot.shooter.shooterMotor.setPower(shooterPower);

        sleep(3000);*/

        robot.basket.swipe();

        sleep(500);

        robot.basket.resetSwiper();

        sleep(1000);

        robot.basket.swipe();

        sleep(500);

        robot.basket.resetSwiper();

        sleep(1000);

        robot.basket.swipe();

        sleep(1500);

        robot.shooter.shooterMotor.setPower(0);
    }

}
