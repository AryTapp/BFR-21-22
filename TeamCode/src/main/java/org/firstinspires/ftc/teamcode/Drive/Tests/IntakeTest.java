package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.BFRMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;

@Config
@Autonomous
public class IntakeTest extends FrogLinearOpMode {

    @Override
    public void initialize() {

    }

    @Override
    public void run() {
        RobotHardware robot = RobotHardware.getInstance();
       // robot.intake.intake(10000, 50);

    }
}
