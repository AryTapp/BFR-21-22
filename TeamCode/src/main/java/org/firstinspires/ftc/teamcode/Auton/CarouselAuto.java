package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.Locale;

//@Autonomous(name = "blueCarouselAuto")
public class CarouselAuto extends FrogLinearOpMode {

    static RobotHardware robot = null;

    ImageResult imageResult = null;
    static double shooterPower = 0;
    int side = 1;
    Pose2d startingPos0;

    Pose2d carousalPos;
    Pose2d sharedDepoCarFloor1;
    Pose2d sharedDepoCarFloor2;
    Pose2d sharedDepoCarFloor3,tmpPos;

    Pose2d homeCarousel;
    int floorNum=2;

    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();

        // Key robot positions on the field
        //startingPos = new Pose2d(-40.25,-56.5* side,Math.PI/2* side);
        startingPos0 = new Pose2d(-39,-64.5* side,0);
        carousalPos = new Pose2d(-71+17.,-60. * side, 0);
        tmpPos = new Pose2d(-52.5, -19.* side, 0);

        sharedDepoCarFloor1 = new Pose2d(-30.25, -17.75* side, -Math.PI/2* side);
        sharedDepoCarFloor2 = new Pose2d(-30.25, -17.75* side, -Math.PI/2* side);
        sharedDepoCarFloor3 = new Pose2d(-28, -17.75* side, -Math.PI/2* side);

        homeCarousel = new Pose2d(-58.5, -36.75* side, -Math.PI/2.0* side);
    }
    @Override
    public void run() {
        Mat picture = robot.phone.getMat();
        telemetry.addData("Photo:","taken");
        floorNum = new PictureAuton().getPosition(picture,side);
        telemetry.addData("Position: " , floorNum);
        telemetry.addData("Side", side);
        telemetry.update();

        if (side==-1) {
            robot.Xrail.capServo.setPosition(0.35);
        }
        robot.drive.setPoseEstimate(startingPos0);
        //move to carousal
        Trajectory trajectory0 = robot.drive.trajectoryBuilder(startingPos0)
                .lineToConstantHeading(carousalPos.vec())
                .build();
        robot.drive.followTrajectory(trajectory0);
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .strafeRight(2.75*side)
                .build();
        robot.drive.followTrajectory(trajectory2);
        robot.drive.turn(Math.PI*5.0/180.0*side);
        //rotate carousal
        robot.carousel.rotateCarousalAuto(3000,-0.35*side);
        //go to temp position...
         trajectory2= robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(tmpPos.vec())
                .build();
        robot.drive.followTrajectory(trajectory2);
        robot.drive.turn(-Math.PI/2.*side);

        //go to depo...
        if (floorNum ==1) {
            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToConstantHeading(sharedDepoCarFloor1.vec())
                    .build();
        } else if (floorNum==2){
            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToConstantHeading(sharedDepoCarFloor2.vec())
                    .build();
        } else {
            trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToConstantHeading(sharedDepoCarFloor3.vec())
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
                    .strafeLeft(-1.0*side)
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
            robot.Xrail.liftxRail(3, 650);
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


///////park in depot near carousal
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(homeCarousel.vec())
                .build();
        robot.drive.followTrajectory(trajectory4);

///////rotate robot by 90
        robot.drive.turn(Math.PI*92/180*side);
        sleep(100);
        robot.Xrail.capServo.setPosition(0.95);
    }
    void setSide (int i){
        side = i;

    }
}
