package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "testXrail")
public class testXRail extends FrogLinearOpMode {
    static RobotHardware robot = null;

    public void initialize() {
        robot = RobotHardware.getInstance();
    }
    @Override
    public void run() {
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Xrail.xRailMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.Xrail.xRailMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Xrail.xRailMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.Xrail.liftxRail(10);
        robot.Xrail.liftxRail(2);
        sleep(1000);
        robot.Xrail.dropFreightAuto(.07);

        robot.Xrail.liftxRail(0);
        sleep(1000);

        robot.Xrail.liftxRail(2);
        sleep(100);
        robot.Xrail.dropFreightAuto(.93);
        robot.Xrail.liftxRail(0);
        sleep(1000);
    }
}
