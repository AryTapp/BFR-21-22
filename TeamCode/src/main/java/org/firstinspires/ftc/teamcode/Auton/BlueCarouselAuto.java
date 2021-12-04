package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Mat;

@Autonomous(name = "blueCarouselAuto")
// Same thing as RedCarousel but just multiply output by -1
public class BlueCarouselAuto extends CarouselAuto{

    public BlueCarouselAuto() {

    }
    public void initialize(){
        setSide(-1);
        super.initialize();
    }

}
