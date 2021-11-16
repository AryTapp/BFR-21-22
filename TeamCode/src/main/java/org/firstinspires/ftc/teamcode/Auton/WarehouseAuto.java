package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.ImageResult;
import org.firstinspires.ftc.teamcode.Utility.UltimateGoalImageProcessor;
import org.opencv.core.Mat;

@Autonomous(name = "warehouseCarouselAuto")
public class WarehouseAuto extends FrogLinearOpMode {

    static RobotHardware robot = null;

    ImageResult imageResult = null;
    static double shooterPower = 0;
    int side = 1;
    // Key robot positions on the field

    Pose2d startingPos = new Pose2d(6.5,-56.5* side,Math.PI/2* side);
    Pose2d sideWallPos = new Pose2d(8.5,-65,0 );

    Pose2d sharedDepoPos2 = new Pose2d(8.5, -25.5* side, Math.PI/2* side);



    //boolean secondWobbleMission = true;

    @Override
    public void initialize() {
        robot = RobotHardware.getInstance();
    }
    @Override
    public void run() {
        Mat picture = robot.phone.getMat();

        UltimateGoalImageProcessor processor = UltimateGoalImageProcessor.getInstance();
        imageResult = processor.process(picture);
        telemetry.addData("shipping element Pos:", imageResult.numberOfRings);
        telemetry.update();

         //Drive to the shooting positon on a spline path while raising the basket and turn on shooting motor
         robot.drive.setPoseEstimate(startingPos);


        Trajectory trajectory = robot.drive.trajectoryBuilder(startingPos)
                .lineToConstantHeading(sharedDepoPos2.vec())
                 .build();
        robot.drive.followTrajectory(trajectory);

         //Drive to shared depot
        robot.drive.turn(-Math.PI/2);
        Pose2d rotatedSharedPos2 = new Pose2d(sharedDepoPos2.getX(), sharedDepoPos2.getY(),0);
        robot.drive.setPoseEstimate(rotatedSharedPos2);


        //robot.Xrail.liftxRail(0.6,"middle");
        sleep(100);
        //robot.Xrail.dropFreightRight();
        sleep(500);

        //park in depot near carousal
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(rotatedSharedPos2)
                .lineToConstantHeading(sideWallPos.vec())
                .build();
        robot.drive.followTrajectory(trajectory4);
        sleep(10000);

    }
    void setSide (int i){
        side = i;
    }
}
