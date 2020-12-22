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

    // Interface method that subclasses that need to implement for things to do at the beginning
    // of the task.
    public abstract void initialize();

    // Interface method that subclasses that need to implement for the task.
    public abstract void repeat();
}
