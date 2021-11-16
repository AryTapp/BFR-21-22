package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.firstinspires.ftc.teamcode.Utility.UltimateGoalImageProcessor;
import org.opencv.core.Mat;

//@Autonomous(name = "blueCarouselAuto")
public class CarouselAuto extends FrogLinearOpMode {

    static RobotHardware robot = null;

    ImageResult imageResult = null;
    static double shooterPower = 0;
    int side = 1;
    // Key robot positions on the field
    Pose2d startingPos1 = new Pose2d(-41.5,-56.5* side,Math.PI/2* side);
    Pose2d startingPos2 = new Pose2d(8.5,-56.5* side,Math.PI/2* side);
    Pose2d startingPos = (side ==1) ? startingPos1 : startingPos2;

    Pose2d carousalPos = new Pose2d(-57, -56.5* side, Math.PI/2* side);
    Pose2d sharedDepoPos1 = new Pose2d(-29.5, -25.5* side, Math.PI/2* side);
    Pose2d sharedDepoPos2 = new Pose2d(8.5, -25.5* side, Math.PI/2* side);
    Pose2d homeCarousel = new Pose2d(-58.5, -34.75* side, Math.PI/2.0* side);

    //boolean secondWobbleMission = true;

    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();
    }
    @Override
    public void run() {

         //Drive to the shooting positon on a spline path while raising the basket and turn on shooting motor
         robot.drive.setPoseEstimate(startingPos);
//
//        Trajectory trajectory = robot.drive.trajectoryBuilder(startingPos)
//                .lineToConstantHeading(new Pose2d(0,0,Math.PI/2.0).vec())
//                .build();
//        robot.drive.followTrajectory(trajectory);


        Trajectory trajectory = robot.drive.trajectoryBuilder(startingPos)
                .lineToConstantHeading(carousalPos.vec())
                 .build();
        robot.drive.followTrajectory(trajectory);
        // Shoot the rings.
        robot.carousel.rotateCarousalAuto(5000,-0.4);

         //Drive to shared depot
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(carousalPos)
                .lineToConstantHeading(sharedDepoPos1.vec())
                .build();
        robot.drive.followTrajectory(trajectory3);

        robot.Xrail.liftxRail("top");
        if (side==1) {
           // robot.Xrail.dropFreightRight();
        } else {
            //robot.Xrail.dropFreightLeft();
        }
        sleep(100);
        //robot.Xrail.dropFreightRight();
        sleep(500);

        //park in depot near carousal
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(sharedDepoPos1)
                .lineToConstantHeading(homeCarousel.vec())
                .build();
        robot.drive.followTrajectory(trajectory4);

        //rotate robot by -90
        robot.drive.turn(-Math.PI/2);
        sleep(10000);

    }
    void setSide (int i){
        side = i;
    }
}
