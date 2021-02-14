package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoalArm implements Subsystem {

    private static double lowerArmPos = 0.60;
    private static double raiseArmPos = 0.22;
    private static double initArmPos = 0.00;

    private static double grabPos = 0.64;
    private static double releasePos = 0.90;

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
    public void armBack(){
        liftArmServo.setPosition(initArmPos);
    }

    public void grab(){
        gripperServo.setPosition(grabPos);
    }
    public void release(){
        gripperServo.setPosition(releasePos);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
    }
}
