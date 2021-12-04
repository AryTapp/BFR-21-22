package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.opencv.core.Mat;

@Autonomous(name = "warehouseCarouselAuto")
public class WarehouseAuto extends FrogLinearOpMode {

    static RobotHardware robot = null;

    ImageResult imageResult = null;
    static double shooterPower = 0;
    int side = 1;
    // Key robot positions on the field

    Pose2d startingPos;
    Pose2d sideWallPos1,sideWallPos2,sideWallPosFinal;
    Pose2d sharedDepoPos2;
    Pose2d sharedDepoPos1;
    Pose2d sharedDepoPos0;

    Pose2d sharedDepoFloor1;
    Pose2d sharedDepoFloor2;
    Pose2d sharedDepoFloor3, startingPos2;
    Pose2d getSideWallPosTmp, sideWallPosTmp;
    Pose2d startingPos0;

    int floorNum = 2;
    //boolean secondWobbleMission = true;

    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();

        //startingPos2 = new Pose2d(12,-56.5* side,Math.PI/2* side);
        startingPos0 = new Pose2d(9,-65.6* side,0);
        sharedDepoFloor1 = new Pose2d(-9., (-41.7-1.25)* side, 0);
        sharedDepoFloor2 = new Pose2d(-9., (-41.7-1)* side, 0);
        sharedDepoFloor3 = new Pose2d(-9, -41.7* side, 0);

        sideWallPos1= new Pose2d(38,-65.6*side,0);
        sideWallPos2 = new Pose2d(38,-45*side,0);
        sharedDepoPos0 = new Pose2d(-9, -25.5* side, 0);


        sharedDepoPos2 = new Pose2d(8.5, -25.5* side, Math.PI/2* side);

    }
    @Override
    public void run() {
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Xrail.xRailMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.Xrail.xRailMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Xrail.xRailMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        Mat picture = robot.phone.getMat();
        telemetry.addData("Photo:","taken");
        telemetry.update();
        floorNum = new pictureAuton().getPosition(picture,side);
        telemetry.addData("Position: " , floorNum);
        telemetry.update();

        if (side==1) {
            robot.Xrail.capServo.setPosition(0.8);
        }

        robot.drive.setPoseEstimate(startingPos0);
        if (floorNum==1) {
            sharedDepoPos0 = sharedDepoFloor1;
        } else if (floorNum==2) {
            sharedDepoPos0 = sharedDepoFloor2;
        } else if (floorNum==3) {
            sharedDepoPos0 = sharedDepoFloor3;
        }
        Trajectory trajectory = robot.drive.trajectoryBuilder(startingPos0)
                .lineToConstantHeading(sharedDepoPos0.vec())
                .build();
        robot.drive.followTrajectory(trajectory);

        if (side==-1) {
            robot.Xrail.capServo.setPosition(0.8);
        }
        //rotate robot 20degrees
        robot.drive.turn(Math.PI/180.*23.*side);

        robot.Xrail.liftxRail(10);
        robot.Xrail.liftxRail(floorNum);
        sleep(10);

        if (side==-1) {
            robot.Xrail.dropFreightAuto(0.05);
            robot.Xrail.xRailServo.setPosition(.035);
            sleep(200);
        } else {
            robot.Xrail.dropFreightAuto(0.875);
            robot.Xrail.xRailServo.setPosition(.925);
            sleep(200);
        }
        robot.Xrail.xRailServo.setPosition(.46);

        ///////drive back the robot
        /////////park in depot near carousal
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(sharedDepoPos0)
                .lineToConstantHeading(startingPos0.vec())
                .build();
        robot.drive.followTrajectory(trajectory4);

        ////////travel alomg wall
        Trajectory trajectory5= robot.drive.trajectoryBuilder(startingPos0)
                .lineToConstantHeading(sideWallPos1.vec())
                .build();
        robot.drive.followTrajectory(trajectory5);

        Trajectory trajectory6 = robot.drive.trajectoryBuilder(sideWallPos1)
                .lineToConstantHeading(sideWallPos2.vec())
                .build();
        robot.drive.followTrajectory(trajectory6);

        robot.Xrail.liftxRail(0);
        sleep(1000);

    }
    void setSide (int i){
        side = i;
    }

}
