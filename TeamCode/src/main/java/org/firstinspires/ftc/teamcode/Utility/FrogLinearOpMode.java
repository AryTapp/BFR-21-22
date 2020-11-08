package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public abstract class FrogLinearOpMode extends LinearOpMode {




    @Override
    public void runOpMode(){


        RobotHardware.getInstance().initialize(hardwareMap, telemetry);
        initialize();

        while (!RobotHardware.getInstance().isInitializationComplete()){}
        waitForStart();
        while (!RobotHardware.getInstance().isInitializationComplete()){}
        run();


    }



    public abstract void initialize();

    public abstract void run();

    public void addTelemetry(String input){
        telemetry.addLine(input);
        telemetry.update();
    }
    public boolean isActive(){
        return opModeIsActive();
    }



}
