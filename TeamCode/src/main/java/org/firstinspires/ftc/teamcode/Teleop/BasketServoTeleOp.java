package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.BFRMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;

//@TeleOp
public class BasketServoTeleOp extends FrogOpMode {

    private double intakePower = 1;
    private boolean intakeOn = false;
    //private double shooterPower = 60;
    private double carouselPower = 0.7;
    private double xRailPower = 1.0;
    int side = 1;
    int currentCounts=0;

    @Override
    public void initialize() {
        BFRMecanumDrive drive = RobotHardware.getInstance().drive;
        RobotHardware robot = RobotHardware.getInstance();
        robot.Xrail.capServo.setPosition(0.4);
        robot.Xrail.xRailServo.setPosition(0.4);
    }

    void setSide(int i) {
        side = i;
    }

    @Override
    public void repeat() {
        RobotHardware robot = RobotHardware.getInstance();
        BFRMecanumDrive drive = robot.drive;

        if (gamepad2.left_stick_y > 0.2) {
            robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition() + 0.01);
        } else if (gamepad2.left_stick_y < -0.2) {
            robot.Xrail.xRailServo.setPosition(robot.Xrail.xRailServo.getPosition() - 0.01);
        }
        if (gamepad2.right_stick_y > 0.2) {
            robot.Xrail.capServo.setPosition(robot.Xrail.capServo.getPosition() + 0.01);
        } else if (gamepad2.right_stick_y < -0.2) {
            robot.Xrail.capServo.setPosition(robot.Xrail.capServo.getPosition() - 0.01);
        }
    }
}