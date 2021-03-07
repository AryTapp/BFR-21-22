package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {


    /**
     * Motors declaration
     */
    public DcMotor intakeMotor;

    /**
     * Motor configuration names
     */
    private String intake = "intake";

    /**
     * Motor variables
     */
    private int directionSign = -1;


    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        intakeMotor = map.dcMotor.get(intake);


    }

    //overload for intake function
    public void intake(double time, double power){
        intakeMotor.setPower( power * directionSign);

        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        intakeMotor.setPower(0);
    }


    @Override
    public void sendTelemetry(Telemetry telemetry) {


    }
}
