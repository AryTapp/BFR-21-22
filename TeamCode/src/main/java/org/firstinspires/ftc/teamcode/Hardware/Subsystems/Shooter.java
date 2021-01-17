package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter implements Subsystem{
    public DcMotor shooterMotor;
    public String shooterName = "shooterMotor";


    @Override

    public void initialize(HardwareMap map, Telemetry telemetry) {
        shooterMotor = map.dcMotor.get(shooterName);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }
}
