//package org.firstinspires.ftc.teamcode.Hardware.Subsystems;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class RingBlockers implements Subsystem{
//
//    public Servo leftBlockServo;
//    private String leftBlockName = "leftBlock";
//
//    public Servo rightBlockServo;
//    private String rightBlockName = "rightBlock";
//
//    @Override
//    public void initialize(HardwareMap map, Telemetry telemetry) {
//        leftBlockServo = map.servo.get(leftBlockName);
//        rightBlockServo = map.servo.get(rightBlockName);
//    }
//
//    public void raiseBlockers(){
//        leftBlockServo.setPosition(0);
//        rightBlockServo.setPosition(.35);
//    }
//    public void lowerBlockers(){
//        leftBlockServo.setPosition(.35);
//        rightBlockServo.setPosition(0);
//    }
//
//
//    @Override
//    public void sendTelemetry(Telemetry telemetry) {
//
//    }
//}
//
//
