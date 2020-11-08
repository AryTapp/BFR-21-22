package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {

    public abstract void initialize(HardwareMap map, Telemetry telemetry);

    public abstract void sendTelemetry(Telemetry telemetry);


}
