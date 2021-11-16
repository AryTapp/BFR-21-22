package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.String;

public class xRail implements Subsystem{
    public DcMotor xRailMotor;
    public String xRailName = "xRailMotor";
    public Servo xRailServo;
    public String xRailServoName = "xRailServo";
    public double xRailPower = 1.0;
//    //Servo
    //private static double dropBasketLeftPos = 0.60;
    //private static double dropBasketRightPos = 0.22;
    //private static double basketInitPos = 0.16;

    //public Servo basketServo;
    //private String basketServoName = "basketServo";

    public void initialize(HardwareMap map, Telemetry telemetry) {
        xRailMotor = map.dcMotor.get(xRailName);
        xRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xRailMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        xRailServo = map.servo.get(xRailServoName);
//        telemetry.addData("Motor Counts", xRailMotor.getCurrentPosition());
//        telemetry.update();

    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {


    }

    public void moveRail(double power, double time){
        xRailMotor.setPower(power);

        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
//
//        telemetry.addData("Motor Counts", xRailMotor.getCurrentPosition());
//        telemetry.update();
        xRailMotor.setPower(0);
    }
    public void dropFreightLeft(){
        xRailServo.setPosition(0.07);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        xRailServo.setPosition(0.5);
    }

    public void dropFreightRight(){
        xRailServo.setPosition(0.93);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        xRailServo.setPosition(0.5);
    }

    private void sleep(int i) {
    }

    public void resetxRailMotor(){
        xRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        telemetry.addData("Motor Counts", xRailMotor.getCurrentPosition());
//        telemetry.update();
    }
    public void liftxRail(String floorString) {
        if (floorString.toLowerCase() == "top") {
            xRailMotor.setTargetPosition(500);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
        } else if (floorString.toLowerCase() == "middle") {
            xRailMotor.setTargetPosition(1000);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);

        } else if (floorString.toLowerCase() == "cap") {
            xRailMotor.setTargetPosition(2000);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);

        } else {
            //lower floor by default
            xRailMotor.setTargetPosition(500);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
        }
        sleep(100);
        xRailMotor.setPower(0);

    }
//    public void lowerxRail(double power) {
//        fourBarMotor.setPower(power);
//    }
//
//    //Servo Code
//    public void dropLeft(){
//        basketServo.setPosition(dropBasketLeftPos);
//    }
//    public void dropRight(){
//        basketServo.setPosition(dropBasketRightPos);
//    }
//    public void initBasket(){
//        basketServo.setPosition(basketInitPos);
//    }
}
