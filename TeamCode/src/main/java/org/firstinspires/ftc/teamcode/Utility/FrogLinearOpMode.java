package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public abstract class FrogLinearOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){

        // Initialize the hardware and subsystems
        RobotHardware.getInstance().initialize(hardwareMap, telemetry);

        // Give the subclass an opportunity to do specific work before run.
        initialize();

        waitForStart();

        // The real code that the subclass needs to write to finish the task.
        run();
    }

    // Interface method that subclasses that need to implement for things to do at the beginning
    // of the task.
    public abstract void initialize();

    // Interface method that subclasses that need to implement for the task.
    public abstract void run();

    public void addTelemetry(String input){
        telemetry.addLine(input);
        telemetry.update();
    }

    public boolean isActive(){
        return opModeIsActive();
    }
}
