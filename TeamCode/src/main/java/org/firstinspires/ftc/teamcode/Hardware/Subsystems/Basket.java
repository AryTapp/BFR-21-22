//package org.firstinspires.ftc.teamcode.Hardware.Subsystems;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
// public class Basket implements Subsystem{
//
//    private static double swiperStart = 0.55;
//    private static double swiperEnd = 0.28;
//
//    private static double lowerBasket = 0.01;
//    private static double raiseBasket = 0.18;
//
//
//    public Servo inclineServo;
//    private String inclineName = "inclineServo";
//
//    public Servo swiperServo;
//    private String swiperName = "swiperServo";
//
//    @Override
//    public void initialize(HardwareMap map, Telemetry telemetry) {
//      //  inclineServo = map.servo.get(inclineName);
//        swiperServo = map.servo.get(swiperName);
//    }
//    public void swipe(){
//        swiperServo.setPosition(swiperEnd);
//    }
//    public void resetSwiper(){
//        swiperServo.setPosition(swiperStart);
//    }
//
//    public void raiseBasket(){
//        inclineServo.setPosition(raiseBasket);
//    }
//    public void lowerBasket(){
//        inclineServo.setPosition(lowerBasket);
//    }
//
//
//    @Override
//    public void sendTelemetry(Telemetry telemetry) {
//    }
//}
//
//
//
//
