package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.opencv.core.Mat;

//@Autonomous(name = "blueCarouselAuto")
public class CarouselAuto extends FrogLinearOpMode {

    static RobotHardware robot = null;

    ImageResult imageResult = null;
    static double shooterPower = 0;
    int side = 1;
    // Key robot positions on the field
    Pose2d startingPos1;
    Pose2d startingPos2;
    Pose2d startingPos;
    Pose2d startingPos0;

    Pose2d carousalPos;
    Pose2d sharedDepoCarFloor1;
    Pose2d sharedDepoCarFloor2;
    Pose2d sharedDepoCarFloor3;

    Pose2d SharedDepoPos1;
    Pose2d SharedDepoPosTmp;

//    Pose2d sharedDepoCarFloor3 = new Pose2d(-29.5, -25.5* side, Math.PI/2* side);

    Pose2d sharedDepoPos2;
    Pose2d homeCarousel;
    int floorNum=2;

    //boolean secondWobbleMission = true;

    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();

        // Key robot positions on the field
        //startingPos = new Pose2d(-40.25,-56.5* side,Math.PI/2* side);
        startingPos0 = new Pose2d(-39,-65.5* side,0);
       // startingPos1 = new Pose2d(-39,-60.75* side,0);

        carousalPos = new Pose2d(-54.1,-63.0 * side, 0);
        sharedDepoCarFloor1 = new Pose2d(-27-1.75, -26.5* side, Math.PI/2* side);
        sharedDepoCarFloor2 = new Pose2d(-27-.65, -26.5* side, Math.PI/2* side);
        sharedDepoCarFloor3 = new Pose2d(-26, -26.5* side, Math.PI/2* side);

        SharedDepoPos1 = new Pose2d(-28, -25.5* side, Math.PI/2* side);;

        SharedDepoPosTmp = new Pose2d(-54.125, -26.* side, Math.PI/2* side);
        homeCarousel = new Pose2d(-60.0, -35.75* side, Math.PI/2.0* side);

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
        floorNum = new pictureAuton().getPosition(picture,side);
        telemetry.addData("Position: " , floorNum);
        telemetry.addData("Side", side);
        telemetry.update();

        if (side==1) {
            robot.Xrail.capServo.setPosition(0.8);
        }
        robot.drive.setPoseEstimate(startingPos0);
        //move to carousal
        Trajectory trajectory0 = robot.drive.trajectoryBuilder(startingPos0)
                .lineToConstantHeading(carousalPos.vec())
                .build();

        robot.drive.followTrajectory(trajectory0);
        sleep(10);
        robot.drive.turn(Math.PI*2.5/180.*side);

        //rotate carousal
        robot.carousel.rotateCarousalAuto(3000,-0.35*side);
        robot.drive.turn(-Math.PI*2.5/180.*side);

         //Drive to shared depot
        if (floorNum==1) {
            SharedDepoPos1 = sharedDepoCarFloor1;
        } else if (floorNum==2) {
            SharedDepoPos1 = sharedDepoCarFloor2;
        } else if (floorNum==3) {
            SharedDepoPos1 = sharedDepoCarFloor3;
        }

        Trajectory trajectory2 = robot.drive.trajectoryBuilder(carousalPos)
                .lineToConstantHeading(SharedDepoPosTmp.vec())
                .build();
        robot.drive.followTrajectory(trajectory2);
        //Turn 90 degrees
        robot.drive.turn(Math.PI*91./180.*side);

        if (side==-1) {
            robot.Xrail.capServo.setPosition(0.8);
        }

        Pose2d rotatedSharedPos2 = new Pose2d(SharedDepoPosTmp.getX(), SharedDepoPosTmp.getY(),Math.PI/2*side);

        robot.drive.setPoseEstimate(rotatedSharedPos2);

        Trajectory trajectory3 = robot.drive.trajectoryBuilder(SharedDepoPosTmp)
                .lineToConstantHeading(SharedDepoPos1.vec())
                .build();

        robot.drive.followTrajectory(trajectory3);

        //robot.drive.turn(Math.PI/180*7*side);

        robot.Xrail.liftxRail(10);
        robot.Xrail.liftxRail(floorNum);
        sleep(10);

        if (side==1) {
            if (floorNum < 3) {
                robot.Xrail.dropFreightAuto(0.03);
                robot.Xrail.xRailServo.setPosition(.02);
            } else {
                //robot.drive.turn(Math.PI/180*5*side);
                robot.Xrail.dropFreightAuto(0.03);
                    robot.Xrail.xRailServo.setPosition(.02);
            }
            sleep(300);
        } else {
            if (floorNum < 3) {
                robot.Xrail.dropFreightAuto(0.9);
                robot.Xrail.xRailServo.setPosition(.925);
            } else {
                robot.Xrail.dropFreightAuto(0.9);
                robot.Xrail.xRailServo.setPosition(.925);
            }
            sleep(300);
        }
        robot.Xrail.xRailServo.setPosition(.46);
        //robot.drive.turn(-Math.PI/180*7*side);
        if (floorNum == 3) {
            //robot.drive.turn(-Math.PI / 180 * 5 * side);
        }


        ///////park in depot near carousal
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(SharedDepoPos1)
                .lineToConstantHeading(homeCarousel.vec())
                .build();
        robot.drive.followTrajectory(trajectory4);

        ///////rotate robot by -90
        robot.drive.turn(-Math.PI*92/180*side);

        robot.Xrail.liftxRail(0);
        sleep(100);
    }
    void setSide (int i){
        side = i;

    }
}
