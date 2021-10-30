package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*public class Shooter implements Subsystem{
    public DcMotor shooterMotor;
    public String shooterName = "shooterMotor";
    public static double shooterConstant = 13.75;
    public static double highGoalPowerConstant = .665;
    private double directionSign = 1;
    @Override

    public void initialize(HardwareMap map, Telemetry telemetry) {
        shooterMotor = map.dcMotor.get(shooterName);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void shoot(double time, double power){
        shooterMotor.setPower(power * directionSign);
        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        shooterMotor.setPower(0);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }
}

 */
