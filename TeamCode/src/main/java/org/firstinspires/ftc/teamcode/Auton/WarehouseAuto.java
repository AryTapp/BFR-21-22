package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.Locale;

@Autonomous(name = "warehouseCarouselAuto")
public class WarehouseAuto extends FrogLinearOpMode {

    static RobotHardware robot = null;

    ImageResult imageResult = null;
    static double shooterPower = 0;
    int side = 1;
    // Key robot positions on the field

    Pose2d startingPos;
    Pose2d tmpPos0;

    Pose2d sideWallPos1,sideWallPos2,sideWallPosFinal;
    Pose2d sharedDepoPos2;
    Pose2d sharedDepoPos1;
    Pose2d sharedDepoPos0, sharedDepoPos3;

    Pose2d sharedDepoFloor1;
    Pose2d sharedDepoFloor2;
    Pose2d sharedDepoFloor3, startingPos2;
    Pose2d getSideWallPosTmp, sideWallPosTmp;
    Pose2d startingPos0;

    int floorNum = 3;
    //boolean secondWobbleMission = true;

    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();

        //startingPos2 = new Pose2d(12,-56.5* side,Math.PI/2* side);
        startingPos0 = new Pose2d(9,-64.5* side,0);
        tmpPos0 = new Pose2d(6,-58.0*side,Math.PI/2*side);
        //sharedDepoPos0 = new Pose2d(-15.9, -39.5* side, 0);
        sharedDepoPos1 = new Pose2d(6.25, -28.0* side, Math.PI/2*side);
        sharedDepoPos2 = new Pose2d(6.25, -28.0* side, Math.PI/2*side);
        sharedDepoPos3 = new Pose2d(4., -28.0* side, Math.PI/2*side);

        sideWallPos1= new Pose2d(38,-65.6*side,0);
        sideWallPos2 = new Pose2d(38,-45*side,0);
    }
    @Override
    public void run() {
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Xrail.xRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Xrail.xRailMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Mat picture = robot.phone.getMat();
        telemetry.addData("Photo:", "taken");
        floorNum = new PictureAuton().getPosition(picture, side);
        telemetry.addData("Position: ", floorNum);
        telemetry.update();

        if (side==-1) {
            robot.Xrail.capServo.setPosition(0.35);
        }
        robot.drive.setPoseEstimate(startingPos0);
        //go forward 24 inches  so we can turn the robot 54

        Trajectory trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .strafeLeft(17*side)
                .build();

        robot.drive.followTrajectory(trajectory2);
        //turn the robot by 90 degrees
        robot.drive.turn(Math.PI/2.0*side);
        //avoid custom shipping element
        if (floorNum > 1) {
            //Strafe robot to left
            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeLeft(5*side).build();
        } else {
            //Strafe robot to right
            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeRight(3*side).build();
        }
        robot.drive.followTrajectory(trajectory2);
        //drive to depo
        if (floorNum ==1) {
            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToConstantHeading(sharedDepoPos1.vec())
                    .build();
        } else if (floorNum==2){
            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToConstantHeading(sharedDepoPos2.vec())
                    .build();
        } else {
            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToConstantHeading(sharedDepoPos3.vec())
                    .build();
        }
        robot.drive.followTrajectory(trajectory2);
        robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());

        robot.Xrail.capServo.setPosition(0.35);

        robot.Xrail.liftxRail(10,0);
        if (floorNum==1) {
            robot.Xrail.liftxRail(2,0);
        } else if (floorNum>1) {
            robot.Xrail.liftxRail(floorNum,0);
        }
        //make basket horizontal
        if (side==-1) {
            robot.Xrail.dropFreightAuto(0.125);
        } else {
            robot.Xrail.xRailServo.setPosition(0.745);
        }
        //Basket needs to stabilize for a second
        sleep(500);
        //read the distance servo value
        double[] myArray = new double[10];
        myArray[0] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[1]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[2]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[3] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[4]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[5] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[6]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[7]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[8]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        myArray[9]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
        Arrays.sort(myArray);
        double distance = (myArray[0]+myArray[1])/2.0;
        telemetry.addData("Distance (inch)",
                String.format(Locale.US, "%.02f", robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH)));
        telemetry.update();
        //rotate the front of the robot toward depo until distance sensor is on top of the lower shelf
        if (distance < 3.5) {
            if (floorNum == 1) {
                Trajectory trajectory1 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .strafeRight(3.0*side)
                        .build();

                robot.drive.followTrajectory(trajectory1);
                robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
                //robot.drive.turn(Math.PI*6./180.*side);
            }
            myArray[0] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[1] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[2] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[3] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[4] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[5] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[6] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[7] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[8] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[9] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            Arrays.sort(myArray);
            distance = (myArray[0] + myArray[1]) / 2.0;
        }
        int j = 0;
        for (int i=0;i<4;i++) {
            if (distance > 3.5) {
                j=i;
                robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
                Trajectory trajectory1 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .strafeLeft(1.25*side)
                        .build();
                robot.drive.followTrajectory(trajectory1);

                myArray[0] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[1]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[2]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[3] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[4]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[5] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[6]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[7]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[8]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                myArray[9]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
                Arrays.sort(myArray);
                distance = (myArray[0]+myArray[1])/2.0;
                telemetry.addData("depo Distance (inch)", robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("basket Distance (inch)", robot.Xrail.basketDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            } else {
                break;
            }
        }
        if (j==3) {
            robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
            Trajectory trajectory1 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeLeft(-2.0*side)
                    .build();
            robot.drive.followTrajectory(trajectory1);
        }

        telemetry.addData("depo Distance (inch)", robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("basket Distance (inch)", robot.Xrail.basketDistance.getDistance(DistanceUnit.INCH));
        telemetry.update();
        robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
        sleep(50);
        if (floorNum==1) {
             trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeRight(1.75*side)
                    .build();
            robot.drive.followTrajectory(trajectory2);
            robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
            myArray[0] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[1] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[2] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[3] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[4]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[5] = robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[6]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[7]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[8]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            myArray[9]= robot.Xrail.depoDistance.getDistance(DistanceUnit.INCH);
            Arrays.sort(myArray);
            distance = (myArray[0]+myArray[1])/2.0;
            if (distance < 3.5) {
                trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .strafeRight(1.0*side)
                        .build();
                robot.drive.followTrajectory(trajectory2);
                robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
            }

        } else if (floorNum==2) {
            robot.Xrail.liftxRail(2, 400);
             trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeLeft(0.75*side)
                    .build();
            robot.drive.followTrajectory(trajectory2);
            robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
            robot.drive.turn(Math.PI*5./180.*side);
        } else if (floorNum==3) {
            robot.Xrail.liftxRail(3, 700);
             trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .strafeLeft(1.0*side)
                    .build();
            robot.drive.followTrajectory(trajectory2);
            robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
            //robot.drive.turn(Math.PI*6./180.*side);
        }
        sleep(10);
        if (side==-1) {
            robot.Xrail.dropFreightAuto(0.0);
        } else {
            robot.Xrail.dropFreightAuto(0.87);
            //robot.Xrail.dropFreightAuto(0.89);
        }
        robot.Xrail.xRailServo.setPosition(0.43);
        robot.Xrail.liftxRail(0,0);

        //Parking
//        if (floorNum==1) {
//            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
//                    .strafeRight(7 * side)
//                    .build();
//
//            robot.drive.followTrajectory(trajectory2);
//            robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
//        }
//        Trajectory trajectory = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
//                .forward(-24*side)
//                .build();
//        robot.drive.followTrajectory(trajectory);
//        robot.drive.setPoseEstimate(robot.drive.getPoseEstimate());
//        robot.drive.turn(-Math.PI/2.0*side);

        Trajectory trajectory4 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(tmpPos0.vec())
                .build();
        robot.drive.followTrajectory(trajectory4);
        robot.drive.turn(-Math.PI/2.0*side);
        trajectory4 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(startingPos0.vec())
                .build();
        robot.drive.followTrajectory(trajectory4);
//        ////////travel along wall
        Trajectory trajectory5= robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(sideWallPos1.vec())
                .build();
        robot.drive.followTrajectory(trajectory5);

        Trajectory trajectory6 = robot.drive.trajectoryBuilder(sideWallPos1)
                .lineToConstantHeading(sideWallPos2.vec())
                .build();
        robot.drive.followTrajectory(trajectory6);

        robot.Xrail.capServo.setPosition(0.85);

    }
    void setSide (int i){
        side = i;
    }

}
