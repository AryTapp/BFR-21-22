package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {


    /**
     * Motors declaration
     */
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    /**
     * Motor configuration names
     */
    private String liName = "li";
    private String riName = "ri";

    /**
     * Motor variables
     */
    private int directionSign = 1;


    @Override
    public void initialize(HardwareMap map, Telemetry telemetry) {
        leftIntake = map.dcMotor.get(liName);
        rightIntake = map.dcMotor.get(riName);


    }

    public void intakePower(double power){
        leftIntake.setPower( power * directionSign);
        rightIntake.setPower(power * directionSign);
    }

    public void intake(double time){
        leftIntake.setPower( 1 * directionSign);
        rightIntake.setPower(1 * directionSign);

        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        leftIntake.setPower( 0);
        rightIntake.setPower(0);

    }

    //overload for intake function
    public void intake(double time, double power){
        leftIntake.setPower( power * directionSign);
        rightIntake.setPower(power * directionSign);

        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        leftIntake.setPower( 0);
        rightIntake.setPower(0);

    }

    public void outtake(double time){
        leftIntake.setPower( -1 * directionSign);
        rightIntake.setPower(-1 * directionSign);

        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        leftIntake.setPower( 0);
        rightIntake.setPower(0);

    }

    //overload for intake function
    public void outtake(double time, double power){
        leftIntake.setPower(-power * directionSign);
        rightIntake.setPower(-power * directionSign);

        try {
            Thread.sleep((long)time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        leftIntake.setPower(0);
        rightIntake.setPower(0);

    }


    @Override
    public void sendTelemetry(Telemetry telemetry) {


    }
}
