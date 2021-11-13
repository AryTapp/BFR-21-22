package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {

    public DcMotor intakeMotor;
    private String intake = "intake";

    private int directionSign = -1;


    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        intakeMotor = map.dcMotor.get(intake);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //clockwise rotation
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //overload for intake function
    public void intake(double time, double power){
        intakeMotor.setPower(power * directionSign);

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

