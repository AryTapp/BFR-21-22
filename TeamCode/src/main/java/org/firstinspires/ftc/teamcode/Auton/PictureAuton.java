package org.firstinspires.ftc.teamcode.Auton;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.FrogLinearOpMode;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;


//Image Inputs:


@Autonomous(name = "pictureAuton")
public class PictureAuton extends FrogLinearOpMode {
    @Override
    public void initialize() {

    }

    @Override
    public void run() {
        RobotHardware robot = RobotHardware.getInstance();
        telemetry.addData("Hardware:", "initialized");
        telemetry.update();
        Mat picture = robot.phone.getMat();
//        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
//        String filename = "Photo.jpg";
//        File file = new File(path, filename);
//        Imgcodecs.imwrite(file.toString(), picture);

        telemetry.addData("Photo:","taken");
        telemetry.update();
        int pos = getPosition(picture, 1);
        telemetry.addData("Position: " , pos);
        telemetry.update();
        while(opModeIsActive()){

        }
    }

    public int getPosition(Mat img, int side){
        int pixelCount = 0;
        int rightCount = 0;
        int leftCount = 0;
        int errors = 0;
        int width = img.rows();
        int height = img.cols();
        //Cropping an image
        Mat image_output= img.submat(0,width*2/3, 3*height/5, height*9/10);
        if (side==-1) {
            image_output= img.submat(width*2/10,width*8/10, 3*height/5, height*9/10);
        } else {
            image_output= img.submat(0,width*7/10, 3*height/5, height*9/10);
        }
//        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
//        String filename = "Photo_crop.jpg";
//        File file = new File(path, filename);
//        Imgcodecs.imwrite(file.toString(), image_output);
        width = image_output.rows();
        height  = image_output.cols();
        //BGR format
        //for(int x = 0; x < width; x++){
            for(int x = 0; x < width; x++){
                //for(int y = (3 * height/5); y < height; y++){
            for(int y =0; y < height; y++){
                    double[] colors = image_output.get(x,y);
                    try {
                        if ((colors[1] > 60) && (colors[2] < 50) && (colors[0]< 60)) {
                            pixelCount++;
                            if (x < (width/2)) {
                                rightCount++;
                            } else {
                                leftCount++;
                            }
                        }
                    } catch (Exception e) {
//                        telemetry.addData("Error:" , e);
//                        telemetry.addData("x:", x);
//                        telemetry.addData("y:",y);
//                        telemetry.update();
                        errors++;
                    }
            }
        }
        telemetry.addData("Pixel Count", pixelCount);
        telemetry.addData("Right Pixel Count", rightCount);
        telemetry.addData("left Pixel Count", leftCount);
        telemetry.addData("Errors: ", errors);
        telemetry.addData("Width: ", width);
        telemetry.addData("Height: ", height);
        if (side == 1) {
            if (pixelCount < 700) {
                return 3; //Right
            } else if (rightCount > leftCount) {
                return 2; //Middle
            } else {
                return 1; //Left
            }
        } else {
            if (pixelCount < 700) {
                return 3; //Left
            } else if (rightCount > leftCount) {
                return 1; //Right
            } else {
                return 2; //Middle
            }
        }
    }

}
