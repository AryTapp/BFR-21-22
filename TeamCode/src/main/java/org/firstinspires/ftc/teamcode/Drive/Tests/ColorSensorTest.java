package org.firstinspires.ftc.teamcode.Drive.Tests;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.opencv.core.Mat;

import java.util.Locale;

@Autonomous(name = "distanceColorSensorTest")
public class ColorSensorTest extends FrogLinearOpMode {
    @Override
    public void initialize() {

    }

    @Override
    public void run() {
        RobotHardware robot = RobotHardware.getInstance();

        while (opModeIsActive()) {
            //robot.Xrail.distanceSensorServo.setPosition(0.1);
            double distance = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            telemetry.addData("depo Distance Sensor (inch)", robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("basket Distance Sensor (inch)", robot.Xrail.basketDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }
}