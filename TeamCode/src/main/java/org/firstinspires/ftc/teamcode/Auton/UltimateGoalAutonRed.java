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

    double secondWobbleX = 24.5;
    double secondWobbleY = -13;

    double shootPosX = 61;
    double shootPosY = -9;
    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();
        robot.wobbleGoalArm.initWobble();
        robot.basket.lowerBasket();

    }

    @Override
    public void run() {
        Mat picture = robot.phone.getMat();
        UltimateGoalImageProcessor processor = UltimateGoalImageProcessor.getInstance();
        imageResult = processor.process(picture);

        robot.wobbleGoalArm.grab();

        robot.basket.resetSwiper();

        telemetry.addData("number of rings:", imageResult.numberOfRings);
        telemetry.update();

        Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(40, 16), Math.toRadians(0))

                .addDisplacementMarker(() -> {
                    robot.basket.raiseBasket();

                    robot.shooter.shooterMotor.setPower(shooterPower);
                })

                .splineToConstantHeading(new Vector2d(shootPosX, shootPosY), Math.toRadians(0))
                .build();

        robot.drive.followTrajectory(trajectory);


        shootThreeRings();

        double xOffset = -1 + shootPosX;
        double yOffset = -24 + shootPosY;
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

        robot.intake.intakeMotor.setPower(1);

        Trajectory trajectory4 = robot.drive.trajectoryBuilder(new Pose2d(xOffset, yOffset))
                .splineToLinearHeading(new Pose2d(secondWobbleX + 24, secondWobbleY - 6, - Math.PI / 2), 0)
                .addDisplacementMarker(() -> {
                    sleep(1);
                })
                .splineToLinearHeading(new Pose2d(secondWobbleX, secondWobbleY, - Math.PI), - Math.PI / 2)
                .build();
        robot.drive.followTrajectory(trajectory4);

        robot.wobbleGoalArm.lowerArm();
        sleep(400);
        robot.wobbleGoalArm.grab();
        sleep(500);
        robot.wobbleGoalArm.raiseArm();
        sleep(200);
        Trajectory trajectory5 = robot.drive.trajectoryBuilder(new Pose2d(secondWobbleX, secondWobbleY, - Math.PI))
                .splineToLinearHeading(new Pose2d(secondWobbleX + 24, secondWobbleY - 6, - Math.PI / 2), - Math.PI)
                .addDisplacementMarker(() -> {
                    sleep(1);
                })
                .splineToLinearHeading(new Pose2d(xOffset, yOffset, 0), - Math.PI/2)
                .build();
        robot.drive.followTrajectory(trajectory5);

        while (Math.abs(robot.drive.getRawExternalHeading()) > 0.02) {
            robot.drive.turn(0 - robot.drive.getRawExternalHeading());
        }

        robot.wobbleGoalArm.lowerArm();
        sleep(400);
        robot.wobbleGoalArm.release();
        sleep(500);

        robot.intake.intakeMotor.setPower(0);

        double parkOffsetX = 0;

        if(imageResult.numberOfRings == 0){
            parkOffsetX = -4;
        }

        Trajectory trajectory6 = robot.drive.trajectoryBuilder(new Pose2d(xOffset - 10 + parkOffsetX, yOffset))
                .strafeTo(new Vector2d(63, -4))
                .build();

        robot.drive.followTrajectory(trajectory6);

        while (Math.abs(robot.drive.getRawExternalHeading()) > 0.02) {
            robot.drive.turn(0 - robot.drive.getRawExternalHeading());
        }

        sleep(10000);
    }

    void shootThreeRings(){
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
