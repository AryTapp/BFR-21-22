package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.BFRMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;
import org.firstinspires.ftc.teamcode.Utility.RobotPosition;

import static android.os.SystemClock.sleep;

@TeleOp
public class Teleop extends FrogOpMode {

    private double intakePower = 1;
    private boolean intakeOn = false;
    private double highGoalPower = .65;
    private double powerShotPower = .60;
    private double shooterPower = highGoalPower;
    private boolean shooterStatus = false;
    double lastRobotPositionX = 0;
    double lastRobotPositionY = 0;
    double lastRobotPositionR = 0;

    @Override
    public void initialize() {
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotHardware robot = RobotHardware.getInstance();
        robot.wobbleGoalArm.liftArmServo.setPosition(0.1);
        robot.phone.activateVuf();
        drive.setPoseEstimate(new Pose2d(10.5, -42.75));
        if (robot.drive.getBatteryVoltage() > 12.0)
            shooterPower = 0.70 * robot.shooter.shooterConstant / robot.drive.getBatteryVoltage();
        highGoalPower = highGoalPower * robot.shooter.shooterConstant / robot.drive.getBatteryVoltage();
        powerShotPower = powerShotPower * robot.shooter.shooterConstant / robot.drive.getBatteryVoltage();
    }

    @Override
    public void repeat()  {
        RobotHardware robot = RobotHardware.getInstance();
        BFRMecanumDrive drive = robot.drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake intake = robot.intake;

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y)
                .rotated(-drive.getRawExternalHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_y
                )
        );
        if(gamepad1.right_bumper){
            robot.intake.intake(10, -intakePower);
        }
        if(gamepad1.left_bumper){
            robot.intake.intake(10, intakePower);
        }

        if(gamepad2.dpad_up){
            robot.basket.raiseBasket();
        }

        if(gamepad2.dpad_down){
            robot.basket.lowerBasket();
        }

        if(gamepad2.dpad_left){
            robot.basket.resetSwiper();
        }

        if(gamepad2.dpad_right){
            robot.basket.swipe();
        }

        if(gamepad1.right_trigger > 0){
            if(shooterStatus){
                shooterStatus = false;
            }
            else{
                shooterStatus = true;
            }
        }

        if(gamepad2.a){
            if(shooterPower == highGoalPower){
                shooterPower = powerShotPower;
            }
            else{
                shooterPower = highGoalPower;
            }
        }

        telemetry.addData("power", shooterPower);

        if(shooterStatus){
            robot.shooter.shooterMotor.setPower(shooterPower);
            telemetry.addData("shooter: ", "on");
        }
        else{
            robot.shooter.shooterMotor.setPower(0);
            telemetry.addData("shooter: ", "off");
        }


        if(gamepad1.a) {
            Pose2d robotPos = robot.phone.getCurrentPosition();
            if (robotPos != null) {
                lastRobotPositionX = robotPos.getX();
                lastRobotPositionY = robotPos.getY();
                lastRobotPositionR = robotPos.getHeading();
                drive.setPoseEstimate(robotPos);
            }
            drive.turn(0 - lastRobotPositionR);
        }

        if(gamepad1.x){
            while (Math.abs(robot.drive.getRawExternalHeading()) > 0.01) {
                drive.turn(0 - robot.drive.getRawExternalHeading());
            }
            sleep(200);
            Pose2d robotPos = robot.phone.getCurrentPosition();
            if(robotPos != null){
                lastRobotPositionX = robotPos.getX();
                lastRobotPositionY = robotPos.getY();
                lastRobotPositionR = robotPos.getHeading();
                Pose2d newPos = new Pose2d(robotPos.getX(), robotPos.getY(), 0);
                drive.setPoseEstimate(newPos);
                Trajectory powerShotRight = robot.drive.trajectoryBuilder(newPos)
                        .lineToLinearHeading(new Pose2d(10.5, -25, 0))
                        .build();
                robot.drive.followTrajectory(powerShotRight);
            }
            shooterPower = powerShotPower;
            shooterStatus = true;
        }

        if(gamepad1.y){
            while (Math.abs(robot.drive.getRawExternalHeading()) > 0.01) {
                drive.turn(0 - robot.drive.getRawExternalHeading());
            }
            sleep(200);
            Pose2d robotPos = robot.phone.getCurrentPosition();
            if(robotPos != null){
                lastRobotPositionX = robotPos.getX();
                lastRobotPositionY = robotPos.getY();
                lastRobotPositionR = robotPos.getHeading();
                Pose2d newPos = new Pose2d(robotPos.getX(), robotPos.getY(), 0);
                drive.setPoseEstimate(newPos);
                Trajectory strafeLeft = robot.drive.trajectoryBuilder(newPos)
                        .strafeLeft(7.5)
                        .build();
                robot.drive.followTrajectory(strafeLeft);
            }
        }

        if(gamepad1.b){
            shooterStatus = true;
            shooterPower = highGoalPower;
            while (Math.abs(robot.drive.getRawExternalHeading()) > 0.02) {
                drive.turn(0 - robot.drive.getRawExternalHeading());
            }
            sleep(200);
            Pose2d robotPos = robot.phone.getCurrentPosition();
            if(robotPos != null){
                lastRobotPositionX = robotPos.getX();
                lastRobotPositionY = robotPos.getY();
                lastRobotPositionR = robotPos.getHeading();
                drive.setPoseEstimate(robotPos);
                Trajectory highGoal = robot.drive.trajectoryBuilder(robotPos)
                        .lineToLinearHeading(new Pose2d(10.5, -41, 0))
                        .build();
                robot.drive.followTrajectory(highGoal);
            }
        }

        telemetry.addData("robot position x", lastRobotPositionX);
        telemetry.addData("robot position y", lastRobotPositionY);
        telemetry.addData("robot position rot", lastRobotPositionR);

        // if stick up, raise arm
        if(gamepad2.left_stick_y < 0){
            robot.wobbleGoalArm.raiseArm();
        }
        if(gamepad2.left_stick_y > 0){
            robot.wobbleGoalArm.lowerArm();
        }
        if(gamepad2.y){
            robot.wobbleGoalArm.initArm();
        }

        if(gamepad2.x){
            robot.wobbleGoalArm.release();
        }
        if(gamepad2.b){
            robot.wobbleGoalArm.grab();
        }
        if(gamepad2.a){
            shooterPower = highGoalPower;

            robot.basket.raiseBasket();

            sleep(500);

            robot.basket.swipe();

            sleep(500);

            robot.basket.resetSwiper();

            sleep(1000);

            robot.basket.swipe();

            sleep(500);

            robot.basket.resetSwiper();

            sleep(1000);

            robot.basket.swipe();

            sleep(500);

            robot.basket.resetSwiper();
        }

        //because we have multiple lines of telemetry i'll just put this down here :)
        telemetry.update();
    }
}

