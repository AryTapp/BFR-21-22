package org.firstinspires.ftc.teamcode.teleopTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;


@TeleOp
public class BasketServoTest extends FrogOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    private static double basketServoPos = 0;

    @Override
    public void initialize() {
        robot.basket.inclineServo.setPosition(0.08);
    }

    @Override
    public void repeat() {
        if(gamepad1.left_stick_y > 0){
            basketServoPos = basketServoPos + 0.001;
            if(basketServoPos > 1){
                basketServoPos = 1;
            }
        }
        else if(gamepad1.left_stick_y < 0){
            basketServoPos = basketServoPos - 0.001;
            if(basketServoPos < 0){
                basketServoPos = 0;
            }
        }
        robot.basket.inclineServo.setPosition(basketServoPos);
        telemetry.addData("basketServoPos", robot.basket.inclineServo.getPosition());
        telemetry.update();
        basketServoPos = robot.basket.inclineServo.getPosition();
    }
}
