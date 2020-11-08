package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Subsystem;

public class Gyro implements Subsystem {



    /** sensor declarations */
    public BNO055IMU imu;


    /** sensor configuration names */

    public String imuName            = "imu";


    public Orientation angles;


    @Override
    public void initialize(HardwareMap map,Telemetry telemetry) {

        telemetry.addData("hello","s");
//
//        /* imu gyro initialization */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;

        telemetry.addData("hello","2");

        imu = map.get(BNO055IMU.class, imuName);
//
//
        telemetry.addData("hello","s2");

        imu.initialize(parameters);
//
//
        telemetry.addData("hello","s3");
//
////
////        //while (!robot.imu.isGyroCalibrated() && !isStopRequested()) {
////        while ((imu.getCalibrationStatus().toString() == "running")) {
//////            telemetry.addData(">", "Gyro is calibrating; DO NOT TOUCH!!!!!!!!!!!");
//////            telemetry.update();
//////            Thread.yield();
////
////
////        }
//
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }


    public double getAngle(){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;

    }


}
