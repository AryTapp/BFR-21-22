package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.firstinspires.ftc.teamcode.Utility.UltimateGoalImageProcessor;
import org.opencv.core.Mat;


//Image Inputs:


@Autonomous(name = "pictureAuton")
public class pictureAuton extends FrogLinearOpMode {
    @Override
    public void initialize() {

    }

    @Override
    public void run() {
        RobotHardware robot = RobotHardware.getInstance();
        telemetry.addData("Hardware:", "initialized");
        telemetry.update();
        Mat picture = robot.phone.getMat();
        telemetry.addData("Photo:","taken");
        telemetry.update();
        int pos = getPosition(picture);
        telemetry.addData("Position: " , pos);
        telemetry.update();
        while(opModeIsActive()){

        }

    }

    public int getPosition(Mat img){
        int pixelCount = 0;
        int rightCount = 0;
        int leftCount = 0;
        int errors = 0;
        int width = img.rows();
        int height = img.cols();
        for(int x = 0; x < width; x+= 5){
            for(int y = 0; y < height; y+=5){
                if(y > (3 * height/5)) {
                    double[] colors = img.get(x,y);
                    try {
                        if (colors[0] < 60 && colors[1] > 75 && colors[2] < 60) {
                            pixelCount++;
                            if (x > (width / 2)) {
                                rightCount++;
                            } else {
                                leftCount++;
                            }
                        }
                    } catch (Exception e){
//                        telemetry.addData("Error:" , e);
//                        telemetry.addData("x:", x);
//                        telemetry.addData("y:",y);
//                        telemetry.update();
                        errors++;
                    }
                }
            }
        }
        telemetry.addData("Pixel Count", pixelCount);
        telemetry.addData("Errors: ", errors);
        telemetry.addData("Width: ", width);
        telemetry.addData("Height: ", height);

        if(pixelCount < 200){
            return 1;
        } else if(rightCount > leftCount){
            return 2;
        } else {
            return 3;
        }
    }

}
