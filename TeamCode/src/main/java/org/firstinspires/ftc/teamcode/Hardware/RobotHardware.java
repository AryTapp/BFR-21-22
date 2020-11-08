package org.firstinspires.ftc.teamcode.Hardware;

//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.BFRPhone;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Capstone;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Extender;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.MotorizedClaw;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.StringedFoundation;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.TapeMeasure;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.TwoPieceFoundation;

import java.util.ArrayList;

/**
 * Created by Sanjith.
 */


public class RobotHardware {
    private static RobotHardware instance = new RobotHardware();
    public static RobotHardware getInstance(){
        return instance;
    }


    public Drive drive = new Drive();
    public Gyro gyro = new Gyro();
    public Extender extender = new Extender();
    public MotorizedClaw motorizedClaw = new MotorizedClaw();
    public BFRPhone bfrPhone = new BFRPhone();
    public TwoPieceFoundation twoPieceFoundation = new TwoPieceFoundation();
    public FoundationMover foundationMover = new FoundationMover();
    public StringedFoundation stringedFoundation = new StringedFoundation();
    public Capstone capstone = new Capstone();
    public Intake intake = new Intake();
    public Claw claw = new Claw();
    public TapeMeasure tapeMeasure = new TapeMeasure();
    private boolean initializationComplete = false;

    public ArrayList<Subsystem> subsystems = new ArrayList<Subsystem>();

    public RobotHardware(){
        subsystems.clear();
        subsystems.add(bfrPhone);
        subsystems.add(gyro);
        subsystems.add(drive);
        subsystems.add(extender);
        subsystems.add(claw);
        subsystems.add(stringedFoundation);
        subsystems.add(capstone);
        subsystems.add(intake);
        subsystems.add(tapeMeasure);

    }

     public void initialize(HardwareMap map, Telemetry telemetry){
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        for(Subsystem subsystem: subsystems){
            subsystem.initialize(map, telemetry);

        }
        initializationComplete = true;
     }

     public boolean isInitializationComplete(){
        return initializationComplete;
     }

    public void sendTelemetry(Telemetry telemetry){
        for(Subsystem subsystem: subsystems){
            subsystem.sendTelemetry(telemetry);
        }

    }


}