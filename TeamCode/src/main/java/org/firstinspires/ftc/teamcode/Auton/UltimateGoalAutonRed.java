package org.firstinspires.ftc.teamcode.Auton;

import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.firstinspires.ftc.teamcode.Utility.UltimateGoalImageProcessor;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

//13.1-13.8 battery

@Autonomous(name = "UltimateGoalAutonRed")
public class UltimateGoalAutonRed extends FrogLinearOpMode {

    static RobotHardware robot = null;


    ImageResult imageResult = null;
    static double shooterPower = 0.65;

    // Key robot positions on the field
    Pose2d shootingPos = new Pose2d(59, -20);
    Pose2d wobbleTargetPos = new Pose2d(59, -12);
    Pose2d secondWobblePos = new Pose2d(22.75, -15.5);

    boolean secondWobbleMission = true;

    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();
        robot.wobbleGoalArm.initWobble();
        robot.basket.lowerBasket();

        if (robot.drive.getBatteryVoltage() > 12.0)
            shooterPower = shooterPower * robot.shooter.shooterConstant / robot.drive.getBatteryVoltage();
    }

    @Override
    public void run() {
        Mat picture = robot.phone.getMat();

        UltimateGoalImageProcessor processor = UltimateGoalImageProcessor.getInstance();
        imageResult = processor.process(picture);
        telemetry.addData("Battery voltage:", robot.drive.getBatteryVoltage());
        telemetry.addData("Number of rings:", imageResult.numberOfRings);
        telemetry.update();

        // Figure out where the wobble needs to be.
        setWobbleTargetPosition();

        // Grab the wobble goal and reset the swiper before driving
        robot.wobbleGoalArm.grab();
        robot.basket.resetSwiper();

        // Drive to the shooting positon on a spline path while raising the basket and turn on shooting motor
        Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(28.5, -6), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.basket.raiseBasket();
                    sleep(300);
                    robot.shooter.shooterMotor.setPower(shooterPower);
                })
                .splineToConstantHeading(new Vector2d(shootingPos.getX(), shootingPos.getY()), Math.toRadians(0))
                .build();
        robot.drive.followTrajectory(trajectory);
        // Shoot the rings.
        robot.shootThreeRings();

        // Drive to target zone and drop the wobble goal
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(shootingPos)
                .strafeTo(new Vector2d(wobbleTargetPos.getX(), wobbleTargetPos.getY()))
                .build();
        robot.drive.followTrajectory(trajectory3);
        robot.wobbleGoalArm.lowerArm();
        sleep(500);
        robot.wobbleGoalArm.release();
        sleep(500);

        // Do the second wobble mission if needed
        //if (secondWobbleMission && imageResult.numberOfRings != 4)
        secondWobbleMission();

        double parkOffsetX = 0;
        if(imageResult.numberOfRings == 0){
            parkOffsetX = -10;
        }

        Trajectory trajectory6 = robot.drive.trajectoryBuilder(
                new Pose2d(wobbleTargetPos.getX() - 10 + parkOffsetX, wobbleTargetPos.getY()))
                .strafeTo(new Vector2d(68, 0))
                .build();
        robot.drive.followTrajectory(trajectory6);

        sleep(10000);
    }

    void writeLog(FileWriter fileWriter, String message)
    {
        message += "\n";
        try {
            fileWriter.write(message);
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("message write failed\n", 0);
        }
    }

    void secondWobbleMission()
    {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        String filename = "log.txt";
        File file = new File(path, filename);

        FileWriter logWriter = null;
        try {
            logWriter = new FileWriter(file);
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("File creation failed\n", 0);
        }

        // Reverse intake in hope of removing rings along the way.
        robot.wobbleGoalArm.raiseArm();
        // Move back to get the second wobble goal
        Pose2d intermediateStop = new Pose2d(secondWobblePos.getX() + 20, secondWobblePos.getY() - 12, - Math.PI );
        if(imageResult.numberOfRings == 1){
            intermediateStop = new Pose2d(secondWobblePos.getX() + 10, secondWobblePos.getY() - 12, -Math.PI);
        }
        Trajectory trajectory1 = robot.drive.trajectoryBuilder(wobbleTargetPos)
                .lineToLinearHeading(intermediateStop)
                .build();
        robot.drive.followTrajectory(trajectory1);
        String message = "Heading 1: " + robot.drive.getRawExternalHeading();
        writeLog(logWriter, message);
        telemetry.addData("Heading 1: ", robot.drive.getRawExternalHeading());

        double wobbleOffsetY = 0;
        double wobbleOffsetX = 0;
        if(imageResult.numberOfRings == 1){
            wobbleOffsetY = 4;
            wobbleOffsetX = 0;
        }
        else if(imageResult.numberOfRings == 4){
            wobbleOffsetY = 2;
            wobbleOffsetX = -1;
        }
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(intermediateStop)
                .lineTo(new Vector2d(secondWobblePos.getX() + wobbleOffsetX, secondWobblePos.getY() + wobbleOffsetY))
                .build();
        robot.drive.followTrajectory(trajectory2);

        robot.wobbleGoalArm.lowerArm();
        sleep(500);
        robot.wobbleGoalArm.grab();
        sleep(500);
        robot.wobbleGoalArm.raiseArm();
        sleep(200);
        message = "Heading 2: " + robot.drive.getRawExternalHeading();
        writeLog(logWriter, message);
        telemetry.addData("Heading 2: ", robot.drive.getRawExternalHeading());

        // Drive to the target zone to drop the second wobble goal
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(new Pose2d(secondWobblePos.getX() + wobbleOffsetX, secondWobblePos.getY() + wobbleOffsetY, - Math.PI))
                .lineTo(new Vector2d(intermediateStop.getX(), intermediateStop.getY()))
                .build();
        robot.drive.followTrajectory(trajectory3);
        message = "Heading 3: " + robot.drive.getRawExternalHeading();
        writeLog(logWriter, message);
        telemetry.addData("Heading 3: ", robot.drive.getRawExternalHeading());

        double angleCompensation = -0.09;
        if(imageResult.numberOfRings == 1){
            angleCompensation = -0.115;
        }
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(intermediateStop)
                .lineToLinearHeading(new Pose2d(wobbleTargetPos.getX()-10, wobbleTargetPos.getY(), angleCompensation))
                .build();
        robot.drive.followTrajectory(trajectory4);
        // Pretend that we are aligned.
        robot.drive.setPoseEstimate(new Pose2d(wobbleTargetPos.getX()-10, wobbleTargetPos.getY(), 0));

        robot.wobbleGoalArm.lowerArm();
        sleep(400);
        robot.wobbleGoalArm.release();
        sleep(500);
        message = "Heading 4: " + robot.drive.getRawExternalHeading();
        writeLog(logWriter, message);
        telemetry.addData("Heading 4: ", robot.drive.getRawExternalHeading());

        try {
            logWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("File close failed\n", 0);
        }
        telemetry.update();
    }

    void setWobbleTargetPosition()
    {
        // case 0:
        wobbleTargetPos = wobbleTargetPos.plus(new Pose2d(1, -24));

        switch (imageResult.numberOfRings) {
            case 1:
                wobbleTargetPos = wobbleTargetPos.plus(new Pose2d(24, 24));
                break;
            case 4:
                wobbleTargetPos = wobbleTargetPos.plus(new Pose2d(45, 1));
                break;
        }
    }



}
