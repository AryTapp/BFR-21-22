package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "redWarehouseAuto")
// Same thing as RedCarousel but just multiply output by -1
public class RedWarehouseAuto extends WarehouseAuto{
    public RedWarehouseAuto() {
        setSide(1);
    }
}
