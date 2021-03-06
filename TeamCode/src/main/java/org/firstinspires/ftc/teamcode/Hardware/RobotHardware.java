package org.firstinspires.ftc.teamcode.Hardware;

//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.BFRPhone;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Basket;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Phone;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.WobbleGoalArm;

import java.util.ArrayList;



public class RobotHardware {
    private static RobotHardware instance = new RobotHardware();
    public static RobotHardware getInstance(){
        return instance;
    }


    public Gyro gyro = null;
    public BFRMecanumDrive drive = null;
    public Phone phone = null;
    public Intake intake = null;
    public Basket basket = null;
    public Shooter shooter = null;
    public WobbleGoalArm wobbleGoalArm = null;

    public ArrayList<Subsystem> subsystems = new ArrayList<Subsystem>();

    public RobotHardware(){
        subsystems.clear();
    }

    public void initialize(HardwareMap map, Telemetry telemetry){

        subsystems.clear();

        drive = new BFRMecanumDrive(map);
        subsystems.add(drive);

        basket = new Basket();
        subsystems.add(basket);

        intake = new Intake();
        subsystems.add(intake);
        
        shooter = new Shooter();
        subsystems.add(shooter);

        wobbleGoalArm = new WobbleGoalArm();
        subsystems.add(wobbleGoalArm);

        phone = new Phone();
        subsystems.add(phone);

        for(Subsystem subsystem: subsystems){
            subsystem.initialize(map, telemetry);
        }

    }



    public void sendTelemetry(Telemetry telemetry){
        for(Subsystem subsystem: subsystems){
            subsystem.sendTelemetry(telemetry);
        }

    }


}