package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Mat;

@Autonomous(name = "blueWarehouseAuto")
// Same thing as RedCarousel but just multiply output by -1
public class BlueWarehouseAuto extends WarehouseAuto{

    public BlueWarehouseAuto() {

    }

    public void initialize(){
        setSide(-1);
        super.initialize();
    }
}
