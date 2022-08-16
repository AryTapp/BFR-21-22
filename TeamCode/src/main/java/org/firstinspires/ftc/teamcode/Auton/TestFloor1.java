package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;

//@Autonomous(name = "testFloor1")
public class TestFloor1 extends FrogLinearOpMode {
    static RobotHardware robot = null;

    public void initialize() {
        robot = RobotHardware.getInstance();
    }

    @Override
    public void run() {
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Xrail.xRailMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.Xrail.liftxRail(10, 0);
        robot.Xrail.liftxRail(2, 0);
        sleep(100);
        robot.Xrail.dropFreightAuto(0.745);
        sleep(6000);
        double distance = 4.0;
        for (int i = 0; i < 10; i++) {
            double d1 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d2 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d3 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d4 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d5 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d6 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d7 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d8 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d9 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            double d10 = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            distance = (d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8 + d9 + d10) / 10.0;
            telemetry.addData("depo Distance (inch)", robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("basket Distance (inch)", robot.Xrail.basketDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(1000);
            if (distance < 3.1) {
                break;
            }
        }
        robot.Xrail.liftxRail(2, 100);

        //robot.Xrail.liftxRail(3, 450);
        robot.Xrail.dropFreightAuto(0.87);

        robot.Xrail.dropFreightAuto(0.89);
            //go to the center
        sleep(3000);
        robot.Xrail.xRailServo.setPosition(0.44);
        robot.Xrail.liftxRail(0, 0);
    }
}