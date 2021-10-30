package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FourBar implements Subsystem{
    public DcMotor fourBarMotor;
    public String FourBarName = "fourBarMotor";
    public void initialize(HardwareMap map, Telemetry telemetry) {
        fourBarMotor = map.dcMotor.get(FourBarName);
        fourBarMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }

    public void fourBar(double power){
        fourBarMotor.setPower(power);
        fourBarMotor.setPower(0);

    }
}
