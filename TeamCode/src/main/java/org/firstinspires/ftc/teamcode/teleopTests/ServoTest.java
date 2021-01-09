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
        robot.basket.inclineServo.setPosition(0.05);

        robot.basket.swiperServo.setPosition(0.55);
//
//        telemetry.addData("incline servo position", inclineServoPos);
//        telemetry.addData("swiper servo position", swiperServoPos);
//        telemetry.update()
        //0.05 and 0.55 are the starting positions for the incline and swiper servos
    }
}
