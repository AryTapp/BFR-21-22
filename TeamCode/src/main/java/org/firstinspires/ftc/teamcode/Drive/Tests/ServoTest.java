package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;

//@Autonomous(name = "servoTest")
public class ServoTest extends FrogLinearOpMode {

    @Override
    public void initialize() {

    }

    @Override
    public void run() {
        RobotHardware robot = RobotHardware.getInstance();

        robot.Xrail.xRailServo.setPosition(0.01);
        telemetry.addData("basket servo position", robot.Xrail.xRailServo.getPosition());
        sleep(500);

        robot.Xrail.xRailServo.setPosition(.98);
        telemetry.addData("basket servo position", robot.Xrail.xRailServo.getPosition());

        telemetry.update();

        sleep(15000);

    }
}
