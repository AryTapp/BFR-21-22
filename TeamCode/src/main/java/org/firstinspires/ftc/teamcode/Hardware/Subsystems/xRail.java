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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class xRail implements Subsystem{
    public DcMotor xRailMotor;
    public String xRailName = "xRailMotor";
    public Servo xRailServo;
    public String xRailServoName = "xRailServo";
    public double xRailPower = 0.5;
    public Servo capServo;
    public String capServoName = "capServo";
    public DcMotor xRailMotor2;
    public String xRailMotor2Name = "xRailMotor2";

    //    Runnable goRight;

//    Runnable goLeft;

//    //Servo
    //private static double dropBasketLeftPos = 0.60;
    //private static double dropBasketRightPos = 0.22;
    //private static double basketInitPos = 0.16;

    //public Servo basketServo;
    //private String basketServoName = "basketServo";

    public void initialize(HardwareMap map, Telemetry telemetry) {
        xRailMotor = map.dcMotor.get(xRailName);
        xRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xRailMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        xRailServo = map.servo.get(xRailServoName);
        capServo = map.servo.get(capServoName);
        telemetry.addData("Motor Counts", xRailMotor.getCurrentPosition());
        telemetry.update();
        capServo.setPosition(0.005);
        xRailMotor2 = map.dcMotor.get(xRailMotor2Name);
        xRailMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
    }

    public void moveRail(double power, double time){
        xRailMotor.setPower(power);
        xRailMotor2.setPower(-power);

        sleep((int)time);
        xRailMotor.setPower(0);
        xRailMotor2.setPower(0);

    }
    public void dropFreightAuto(double servoPos){
        //goLeft.run();
        xRailServo.setPosition(servoPos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        xRailServo.setPosition(0.47);
    }
    public void dropFreight(double servoPos){
        //goLeft.run();
        xRailServo.setPosition(servoPos);
        sleep(2000);
//        try {
//            Thread.sleep(2000);
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }
//
//        xRailServo.setPosition(servoPos + 0.05);
//        sleep(100);
//
//        try {
//            Thread.sleep(20);
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }

        //xRailServo.setPosition(0.5);
    }
    public void capElement(double servoPos){
        double x = capServo.getPosition();
        capServo.setPosition(x+servoPos);
        sleep(10);
        if (capServo.getPosition()<=0.05){
            capServo.setPosition(0.06);
        }
    }
    public void capElementDown(double servoPos){
        double x = capServo.getPosition();
        capServo.setPosition(x-servoPos);
        sleep(10);
        if (capServo.getPosition()<=0.05){
            capServo.setPosition(0.06);
        }
    }
    public void setZeroBasket(){
        xRailServo.setPosition(0);
    }
    public void setZeroCap(){
        capServo.setPosition(0);
    }

    private void sleep(int i) {
    }


    public void liftxRail(int floorNumber) {
        if (floorNumber == 3) {
            //xrail has to move up
            xRailMotor.setTargetPosition(5350);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
            xRailMotor2.setPower(-xRailPower);
        } else if (floorNumber == 2) {
            //xrail has to move up
            xRailMotor.setTargetPosition(3650);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
            xRailMotor2.setPower(-xRailMotor.getPower());

        } else if (floorNumber == 1) {
            //xrail has to move up
            xRailMotor.setTargetPosition(2500);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
            xRailMotor2.setPower(-xRailMotor.getPower());

        } else if (floorNumber == 10) {
            //move tiny bit //xrail has to move up
            xRailMotor.setTargetPosition(10);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
           xRailMotor2.setPower(-xRailMotor.getPower());

        } else {
            //go home
            xRailMotor.setTargetPosition(5);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
            xRailMotor2.setPower(xRailMotor.getPower());
        }
        while ( xRailMotor.isBusy()){}
        xRailMotor.setPower(0);
        xRailMotor2.setPower(0);

    }
}