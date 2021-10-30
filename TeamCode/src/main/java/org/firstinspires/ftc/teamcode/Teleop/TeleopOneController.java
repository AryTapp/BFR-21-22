package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.BFRMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.FourBar;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;

@TeleOp
public class TeleopOneController extends FrogOpMode {
    private double intakePower = 90;
    private boolean intakeOn = false;
    //private double shooterPower = 60;
    private double fourBarPower = 90;
    @Override
   public void initialize() {
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotHardware robot = RobotHardware.getInstance();
        //robot.basket.lowerBasket();
        //robot.basket.resetSwiper();
    }
    @Override
    public void repeat()  {
        RobotHardware robot = RobotHardware.getInstance();
        BFRMecanumDrive drive = robot.drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Intake intake = robot.;
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
        if (gamepad1.dpad_up) {
            robot.fourBar.fourBar(fourBarPower);
        }
        if (gamepad1.dpad_down) {
            robot.fourBar.fourBar(- fourBarPower);
        }
        if(gamepad1.right_bumper){
             robot.intake.intake(10, intakePower);
        }
        if(gamepad1.left_bumper){
            robot.intake.intake(10, -intakePower);
        }

      /*  if(gamepad1.dpad_up){
            robot.basket.raiseBasket();
        }

        if(gamepad1.dpad_down){
            robot.basket.lowerBasket();
        }

        if(gamepad1.dpad_left){
            robot.basket.resetSwiper();
        }

        if(gamepad1.dpad_right){
            robot.basket.swipe();
        }

        if(gamepad1.left_trigger > 0.05){
            robot.shooter.shoot(10, shooterPower);
        }

       */
    }
}

