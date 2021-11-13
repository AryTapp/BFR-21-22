package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel implements Subsystem{

    public DcMotor carousalMotorLeft;
    public DcMotor carousalMotorRight;

    public String shooterNameR = "carousalMotorR";
    public String shooterNameL = "carousalMotorL";

    public static double shooterConstant = 13.75;
    public static double highGoalPowerConstant = .665;
    private double directionSign = 1;
    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        carousalMotorRight = map.dcMotor.get(shooterNameR);
        carousalMotorLeft = map.dcMotor.get(shooterNameL);

        carousalMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousalMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void rotateCarousalAuto(double time, double power){
        carousalMotorLeft.setPower(power);
        carousalMotorRight.setPower(power);
        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        carousalMotorRight.setPower(0);
        carousalMotorLeft.setPower(0);
    }

    public void rotateCarousalTeleOp(double power){
        carousalMotorLeft.setPower(power);
        carousalMotorRight.setPower(power);
    }
    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }
}
