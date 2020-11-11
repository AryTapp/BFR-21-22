package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


public abstract class FrogOpMode extends OpMode {


    @Override
    public void init() {
        RobotHardware.getInstance().initialize(hardwareMap, telemetry);
        initialize();

    }

    @Override
    public void loop() {
        repeat();
        RobotHardware.getInstance().sendTelemetry(telemetry);

    }

    public abstract void initialize();

    public abstract void repeat();



}
