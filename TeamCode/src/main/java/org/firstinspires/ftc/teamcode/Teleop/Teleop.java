package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.BFRMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;
@TeleOp
public class Teleop extends FrogOpMode {
    @Override
    public void initialize() {
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void repeat()  {
        telemetry.addData("Pigs", "%f, %f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.update();
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        );

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

    }
}

