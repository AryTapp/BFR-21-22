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

    private double intakePower = 1;
    private boolean intakeOn = false;
    private double highGoalPower = .70;
    private double powerShotPower = .67;
    private double shooterPower = highGoalPower;
    private boolean shooterStatus = false;

    @Override
    public void initialize() {
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotHardware robot = RobotHardware.getInstance();
        robot.wobbleGoalArm.liftArmServo.setPosition(0.1);
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

        if(gamepad2.left_bumper){
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
            while (Math.abs(robot.drive.getRawExternalHeading()) > 0.02) {
                drive.turn(0 - robot.drive.getRawExternalHeading());
            }
        }



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

        //because we have multiple lines of telemetry i'll just put this down here :)
        telemetry.update();
    }
}

