package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.BFRMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;
@TeleOp
public class Teleop extends FrogOpMode {
    private BFRMecanumDrive drive = null;
    @Override
    public void initialize() {
        drive = new BFRMecanumDrive(hardwareMap);

    }

    @Override
    public void repeat()  {
        telemetry.addData("Pigs", "%f, %f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.update();

        if(gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
            Vector2d direction = new Vector2d(-0.1 * gamepad1.left_stick_x, -0.1 * gamepad1.left_stick_y);
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeTo(direction)
                    .build();
            drive.followTrajectory(trajectory);

            try {
                wait(2000);
            }
            catch(Exception e)
            {}
        }
    }
}

