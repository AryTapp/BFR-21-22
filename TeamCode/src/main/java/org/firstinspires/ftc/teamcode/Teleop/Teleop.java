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
@TeleOp
public class Teleop extends FrogOpMode {
    private double intakePower = 80;
    private boolean intakeOn = false;
    private double shooterPower = 60;
    @Override
    public void initialize() {
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void repeat()  {
        //telemetry.addData("Pigs", "%f, %f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        //telemetry.update();
        RobotHardware robot = RobotHardware.getInstance();
        BFRMecanumDrive drive = robot.drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake intake = robot.intake;

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x)
                .rotated(-drive.getRawExternalHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
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

        if(gamepad2.left_bumper){
            robot.shooter.shoot(10, shooterPower);
        }

    }
}

