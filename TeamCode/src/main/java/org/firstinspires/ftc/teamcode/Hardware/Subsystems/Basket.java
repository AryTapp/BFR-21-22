package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Basket implements Subsystem{
    public Servo inclineServo;
    private String inclineName = "inclineServo";

    public Servo swiperServo;
    private String swiperName = "swiperServo";

    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        inclineServo = map.servo.get(inclineName);
        swiperServo = map.servo.get(swiperName);

    }
    public void swipe(){

    }
    @Override
    public void sendTelemetry(Telemetry telemetry) {
    }
}
