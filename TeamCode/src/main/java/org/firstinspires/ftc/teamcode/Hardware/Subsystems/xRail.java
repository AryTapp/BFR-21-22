package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.String;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class xRail implements Subsystem{
    public DcMotor xRailMotor;
    public String xRailName = "xRailMotor";
    public Servo xRailServo;
    public String xRailServoName = "xRailServo";
    public double xRailPower = 0.9;
    public Servo capServo;
    //public Servo distanceSensorServo;
    public String capServoName = "capServo";
    //public String distanceSensorServoName = "colorSensorServo";

    //public DcMotor xRailMotor2;
    public String xRailMotor2Name = "xRailMotor2";
    public ColorSensor colorSensorBasket;
    public ColorSensor colorSensorDepo;
    public String colorSensorName = "depoSensor";
    public String basketSensorName = "basketSensor";

    public DistanceSensor depoDistance;
    public DistanceSensor basketDistance;

    public void initialize(HardwareMap map, Telemetry telemetry) {
        xRailMotor = map.dcMotor.get(xRailName);
        xRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xRailMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        xRailServo = map.servo.get(xRailServoName);
        capServo = map.servo.get(capServoName);
        //distanceSensorServo = map.servo.get(distanceSensorServoName);

        telemetry.addData("Motor Counts", xRailMotor.getCurrentPosition());
        telemetry.update();
        capServo.setPosition(0.98);
        //distanceSensorServo.setPosition(0.5);

        // xRailServo.setPosition(0.5);
        //xRailMotor2 = map.dcMotor.get(xRailMotor2Name);
        //xRailMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colorSensorBasket = map.colorSensor.get(basketSensorName);
        colorSensorDepo = map.colorSensor.get(colorSensorName);

        // get a reference to the distance sensor that shares the same name.
        depoDistance = map.get(DistanceSensor.class, colorSensorName);
        // get a reference to the distance sensor that shares the same name.
        basketDistance = map.get(DistanceSensor.class, basketSensorName);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {
    }

    public void moveRail(double power, double time){
        xRailMotor.setPower(power);
    }
    public void dropFreightAuto(double servoPos){
        //goLeft.run();
        xRailServo.setPosition(servoPos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    public void dropFreight(double servoPos){
        //goLeft.run();
        xRailServo.setPosition(servoPos);
        sleep(2000);
    }
    public void capElement(double servoPos){
        double x = capServo.getPosition();
        capServo.setPosition(x-servoPos);
        sleep(10);
        if (capServo.getPosition() >= 0.98){
            capServo.setPosition(0.97);
        }
    }
    public void capElementDown(double servoPos){
        double x = capServo.getPosition();
        capServo.setPosition(x+servoPos);
        sleep(10);
        if (capServo.getPosition() >= 0.98){
            capServo.setPosition(0.97);
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

    public void liftxRail(int floorNumber, int addCounts) {
        if (floorNumber == 3) {
            //xrail has to move up
            xRailMotor.setTargetPosition(1445+addCounts);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
        } else if (floorNumber == 2) {
            //xrail has to move up
            xRailMotor.setTargetPosition(865+addCounts);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
        } else if (floorNumber == 1) {
            //xrail has to move up
            xRailMotor.setTargetPosition(865+addCounts);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
            //xRailMotor2.setPower(-xRailMotor.getPower());
        } else if (floorNumber == 10) {
            //move tiny bit //xrail has to move up
            xRailMotor.setTargetPosition(10);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower);
           //xRailMotor2.setPower(-xRailMotor.getPower());
        } else {
            //go home
            xRailMotor.setTargetPosition(5);
            xRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xRailMotor.setPower(xRailPower*0.4);
        }
        while ( xRailMotor.isBusy()){}
        xRailMotor.setPower(0);
        //xRailMotor2.setPower(0);

    }
}