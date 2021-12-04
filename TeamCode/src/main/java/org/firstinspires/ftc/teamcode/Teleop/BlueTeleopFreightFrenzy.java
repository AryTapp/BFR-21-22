package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.BFRMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;

@TeleOp
public class BlueTeleopFreightFrenzy extends FrogOpMode {

    private double intakePower = 1;
    private boolean intakeOn = false;
    //private double shooterPower = 60;
    private double carouselPower = 0.6;
    private double xRailPower = 1.0;
    int side = -1;

    @Override
    public void initialize() {
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RobotHardware robot = RobotHardware.getInstance();
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Xrail.xRailMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Xrail.capServo.setPosition(0.8);
    }
    void setSide(int i){
        side=i;
    }

    @Override
    public void repeat() {
        RobotHardware robot = RobotHardware.getInstance();
        BFRMecanumDrive drive = robot.drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double xVal = gamepad1.left_stick_x * side;
        double yVal = gamepad1.left_stick_y * side;
        double zRot = gamepad1.right_stick_y;
        if (gamepad1.right_trigger>0.5) {
            xVal = 0.3*xVal;
            yVal = 0.3*yVal;
            zRot = 0.3*zRot;
        }
        if (gamepad1.left_trigger>0.5) {
            xVal = 0.3*xVal;
            yVal = 0.3*yVal;
            zRot = 0.3*zRot;
        }

        // Intake intake = robot.;
        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
//        Vector2d input = new Vector2d(
//                gamepad1.left_stick_x*side,
//                -gamepad1.left_stick_y*side)
//                .rotated(-drive.getRawExternalHeading());


        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        input.getX(),
//                        input.getY(),
//                        -gamepad1.right_stick_y
//                )
//        );


        Vector2d input = new Vector2d(
                xVal,
                -yVal)
                .rotated(-drive.getRawExternalHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -zRot
                )
        );


        if (gamepad2.left_stick_y > 0.4) {
            robot.Xrail.moveRail(xRailPower, 2);
        } else if (gamepad2.left_stick_y < -0.4) {
            robot.Xrail.moveRail(-xRailPower*0.75, 2);
        }

        if (gamepad1.right_bumper) {
            robot.intake.intake(10, intakePower);
        }
        if (gamepad1.left_bumper) {
            robot.intake.intake(10, -intakePower);
        }
        if (gamepad2.right_bumper) {
            robot.carousel.rotateCarousalTeleOp(carouselPower);
        }
        if (gamepad2.left_bumper) {
            robot.carousel.rotateCarousalTeleOp(-carouselPower);
        }
        if (gamepad2.x) {
            robot.Xrail.xRailServo.setPosition(0.46);
        }
        if (gamepad2.right_stick_x < -0.4) {
            robot.Xrail.dropFreight(0.87);
        }
        if (gamepad2.right_stick_x > 0.4) {
            robot.Xrail.dropFreight(0.03);
        }
        if (gamepad2.a) {
            robot.Xrail.capElement(0.01);
        }
        if (gamepad2.b) {
            robot.Xrail.capElementDown(0.02);
        }
        if (gamepad2.y){
            robot.Xrail.capServo.setPosition(0.18);
        }
        //if blok stuck
        if (gamepad2.dpad_left){
            if (robot.Xrail.xRailServo.getPosition()>0.75){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()+0.0025);
            }
            if (robot.Xrail.xRailServo.getPosition()<0.25){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()-0.0025);
            }
        }
        if (gamepad2.dpad_right){
            if (robot.Xrail.xRailServo.getPosition()>0.75){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()+0.0025);
            }
            if (robot.Xrail.xRailServo.getPosition()<0.25){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()-0.0025);
            }
        }
        if (gamepad2.dpad_down){
            if (robot.Xrail.xRailServo.getPosition()>0.75){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()+0.0025);
            }
            if (robot.Xrail.xRailServo.getPosition()<0.25){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()-0.0025);
            }
        }
        if (gamepad2.dpad_up){
            if (robot.Xrail.xRailServo.getPosition()>0.75){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()+0.0025);
            }
            if (robot.Xrail.xRailServo.getPosition()<0.25){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()-0.0025);
            }
        }
    }
}
