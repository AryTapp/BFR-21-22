package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoalArm implements Subsystem {

    private static double lowerArmPos = 0.60;
    private static double raiseArmPos = 0.22;
    private static double initArmPos = 0.12;

    private static double grabPos = 0.90;
    private static double releasePos = 0.64;
    private static double halfOpenPos = 0.83;

    public Servo liftArmServo;
    private String liftArmName = "liftArmServo";

    public Servo gripperServo;
    private String gripperName = "gripperServo";

    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        liftArmServo = map.servo.get(liftArmName);
        gripperServo = map.servo.get(gripperName);
    }

    public void lowerArm(){
        liftArmServo.setPosition(lowerArmPos);
    }
    public void raiseArm(){
        liftArmServo.setPosition(raiseArmPos);
    }
    public void initArm(){
        liftArmServo.setPosition(initArmPos);
    }

    public void grab(){
        gripperServo.setPosition(grabPos);
    }
    public void release(){
        gripperServo.setPosition(releasePos);
    }
    public void halfOpen(){
        gripperServo.setPosition(halfOpenPos);
    }

    public void initWobble(){
        initArm();
        halfOpen();
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
    }
}
