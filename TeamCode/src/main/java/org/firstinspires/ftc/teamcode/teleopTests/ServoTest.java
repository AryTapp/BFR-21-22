package org.firstinspires.ftc.teamcode.teleopTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;
@TeleOp
public class ServoTest extends FrogOpMode {
    @Override
    public void initialize() {

    }

    @Override
    public void repeat() {
        RobotHardware robot = RobotHardware.getInstance();
        robot.basket.inclineServo.setPosition(0);

        robot.basket.inclineServo.setPosition(0);
//
//        telemetry.addData("incline servo position", inclineServoPos);
//        telemetry.addData("swiper servo position", swiperServoPos);
//        telemetry.update();
    }
}
