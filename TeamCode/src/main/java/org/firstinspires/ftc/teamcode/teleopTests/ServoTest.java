package org.firstinspires.ftc.teamcode.teleopTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;

import static android.os.SystemClock.sleep;


@TeleOp
public class ServoTest extends FrogOpMode {

    @Override
    public void initialize() {
        RobotHardware robot = RobotHardware.getInstance();

//        robot.basket.inclineServo.setDirection(Servo.Direction.REVERSE);
        robot.basket.inclineServo.setPosition(0.08);
        sleep(2000);
        robot.basket.inclineServo.setPosition(0.22);

    }

    @Override
    public void repeat() {

//        telemetry.addData("incline servo position", inclineServoPos);
//        telemetry.addData("swiper servo position", swiperServoPos);
//        telemetry.update()
        //0.05 and 0.55 are the starting positions for the incline and swiper servos
    }
}
