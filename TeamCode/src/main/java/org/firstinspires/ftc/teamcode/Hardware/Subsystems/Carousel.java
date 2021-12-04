package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Carousel implements Subsystem{


    public DcMotor carousalMotorLeft;

    public String shooterNameL = "carousalMotorL";

    public static double shooterConstant = 13.75;
    public static double highGoalPowerConstant = .665;
    private double directionSign = 1;
    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        carousalMotorLeft = map.dcMotor.get(shooterNameL);

        carousalMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void rotateCarousalAuto(double time, double power){
        carousalMotorLeft.setPower(power);
        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        carousalMotorLeft.setPower(0);
    }

    public void rotateCarousalTeleOp(double power){
        carousalMotorLeft.setPower(power);
        carousalMotorLeft.setPower(0);
    }
    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }
}
