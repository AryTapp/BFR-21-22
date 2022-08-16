package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.BFRMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;

@TeleOp
public class BlueTeleopFreightFrenzy extends FrogOpMode {

    private double intakePower = 1;
    private boolean intakeOn = false;
    private double carouselPower = 0.7;
    private double xRailPower = 1.0;
    int side = -1;

    @Override
    public void initialize() {
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotHardware robot = RobotHardware.getInstance();

        robot.Xrail.xRailMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.Xrail.xRailMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Xrail.capServo.setPosition(0.35);
    }

    void setSide(int i) {
        side = i;
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
        if (gamepad1.right_trigger > 0.5) {
            xVal = 0.3 * xVal;
            yVal = 0.3 * yVal;
            zRot = 0.3 * zRot;
        }
        if (gamepad1.left_trigger > 0.5) {
            xVal = 0.3 * xVal;
            yVal = 0.3 * yVal;
            zRot = 0.3 * zRot;
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

        if (gamepad2.left_stick_y > 0.2) {
            robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad2.right_trigger>0.2 || gamepad1.left_trigger>0.2) {
                robot.Xrail.moveRail(-xRailPower * Math.abs(gamepad2.left_stick_y) * .75, 2);
            } else {
                robot.Xrail.moveRail(-xRailPower*Math.abs(gamepad2.left_stick_y), 2);
            }
        } else if (gamepad2.left_stick_y < -0.2) {
            robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad2.right_trigger>0.2 || gamepad1.left_trigger>0.2) {
                robot.Xrail.moveRail(xRailPower * Math.abs(gamepad2.left_stick_y)*.3, 2);
            } else {
                robot.Xrail.moveRail(xRailPower*Math.abs(gamepad2.left_stick_y)*.5, 2);
            }
        } else {
            robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Xrail.moveRail(0, 2);
        }
        telemetry.addData("xrail Counts", robot.Xrail.xRailMotor.getCurrentPosition());
        double depoDistance = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        double basketDistance = robot.Xrail.basketDistance.getDistance(DistanceUnit.INCH);
        telemetry.addData("Depo Distance (inch)",depoDistance);
        telemetry.addData("Basket Distance (inch)",basketDistance);

        telemetry.update();
        if (gamepad1.right_bumper) {
            if ((robot.Xrail.basketDistance.getDistance(DistanceUnit.INCH) < 2.75) && ((gamepad2.right_trigger > 0.2 || gamepad1.left_trigger > 0.2) )){
                robot.intake.intake(10, intakePower);
            }
            if ((robot.Xrail.basketDistance.getDistance(DistanceUnit.INCH) > 2.75)){
                robot.intake.intake(10, intakePower);
            }
        } else if (gamepad1.left_bumper) {
            if (gamepad2.right_trigger>0.2 || gamepad1.left_trigger>0.2) {
                robot.intake.intake(10, -intakePower*0.5);
            } else {
                robot.intake.intake(10, -intakePower);
            }
        } else {
            robot.intake.intake(10, 0);
        }
        if (gamepad2.right_bumper) {
            if (gamepad2.right_trigger>0.2 || gamepad2.left_trigger>0.2) {
                robot.carousel.rotateCarousalTeleOp(carouselPower*0.35);
            } else {
                robot.carousel.rotateCarousalTeleOp(carouselPower*.45);
            }
        } else if (gamepad2.left_bumper) {
            if (gamepad2.right_trigger>0.2 || gamepad2.left_trigger>0.2) {
                robot.carousel.rotateCarousalTeleOp(-carouselPower*0.35);
            } else {
                robot.carousel.rotateCarousalTeleOp(-carouselPower*.45);
            }
        } else {
            robot.carousel.rotateCarousalTeleOp(0);
        }
        if (gamepad2.x) {
            robot.Xrail.xRailServo.setPosition(0.435);
        }
        if (gamepad2.right_stick_x < -0.4) {
            robot.Xrail.dropFreight(0.87);
        }
        if (gamepad2.right_stick_x > 0.4) {
            robot.Xrail.dropFreight(0.0);
        }
        if (gamepad2.a) {
            if (gamepad2.right_trigger>0.2 || gamepad2.left_trigger>0.2) {
                robot.Xrail.capElement(0.0025);
            } else {
                robot.Xrail.capElement(0.005);
            }
        }
        if (gamepad2.b) {
            if (gamepad2.right_trigger>0.2 || gamepad2.left_trigger>0.2) {
                robot.Xrail.capElementDown(0.0025);
            } else {
                robot.Xrail.capElementDown(0.005);
            }
        }
        if (gamepad2.y){
            robot.Xrail.capServo.setPosition(0.98);
        }
        if (gamepad2.dpad_left || gamepad2.dpad_right){
            if (robot.Xrail.xRailServo.getPosition()>0.75){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()+0.0025);
            }
            if (robot.Xrail.xRailServo.getPosition()<0.25){
                robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition()-0.0025);
            }
        }
    }
}