package org.firstinspires.ftc.teamcode.Hardware;

//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Basket;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.FourBar;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Gyro;
//import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Phone;
//import org.firstinspires.ftc.teamcode.Hardware.Subsystems.RingBlockers;
//import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Subsystem;
//import org.firstinspires.ftc.teamcode.Hardware.Subsystems.WobbleGoalArm;

import java.util.ArrayList;

import static android.os.SystemClock.sleep;


public class RobotHardware {
    private static RobotHardware instance = new RobotHardware();
    public static RobotHardware getInstance(){
        return instance;
    }

    static long shootingHoldTime = 300;
    static long resetTime = 600;

    public Gyro gyro = null;
    public BFRMecanumDrive drive = null;
    public Phone phone = null;
    public Intake intake = null;
    public FourBar fourBar = null;
    //  public Basket basket = null;
    //public Shooter shooter = null;
   // public WobbleGoalArm wobbleGoalArm = null;
    //public RingBlockers ringBlockers = null;

    public ArrayList<Subsystem> subsystems = new ArrayList<Subsystem>();

    public RobotHardware(){
        subsystems.clear();
    }

    public void initialize(HardwareMap map, Telemetry telemetry){

        subsystems.clear();

        drive = new BFRMecanumDrive(map);
        subsystems.add(drive);

     //   basket = new Basket();
//        subsystems.add(basket);

        intake = new Intake();
       subsystems.add(intake);

       fourBar = new FourBar();
       subsystems.add(fourBar);

        //shooter = new Shooter();
//        subsystems.add(shooter);

//       wobbleGoalArm = new WobbleGoalArm();
//        subsystems.add(wobbleGoalArm);

//        ringBlockers = new RingBlockers();
//        subsystems.add(ringBlockers);


        phone = new Phone();
        subsystems.add(phone);

        for(Subsystem subsystem: subsystems){
            subsystem.initialize(map, telemetry);
        }

    }
//    public void shootThreeRings(){
////        basket.swipe();
//
//        sleep(shootingHoldTime);
//
////        basket.resetSwiper();
//
//        sleep(resetTime);
//
//        basket.swipe();
//
//        sleep(shootingHoldTime);
//
//        basket.resetSwiper();
//
//        sleep(resetTime);
//
//        basket.swipe();
//
//        sleep(shootingHoldTime);
//
//        basket.resetSwiper();
//        basket.lowerBasket();
//    }
//
//    public void shootTwoRings(){
//        basket.swipe();
//
//        sleep(shootingHoldTime);
//
//        basket.resetSwiper();
//
//        sleep(resetTime);
//
//        basket.swipe();
//
//        sleep(shootingHoldTime);
//
//        basket.resetSwiper();
//
//        basket.lowerBasket();
//    }

    public void sendTelemetry(Telemetry telemetry){
        for(Subsystem subsystem: subsystems){
            subsystem.sendTelemetry(telemetry);
        }

    }


}