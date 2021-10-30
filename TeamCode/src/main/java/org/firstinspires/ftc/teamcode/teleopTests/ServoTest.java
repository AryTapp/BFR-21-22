//package org.firstinspires.ftc.teamcode.teleopTests;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.Utility.FrogOpMode;
//
//import static android.os.SystemClock.sleep;
//
//
//@TeleOp
//public class ServoTest extends FrogOpMode {
//    RobotHardware robot = RobotHardware.getInstance();
//    private static double liftArmServoPos = 0;
//    private static double gripperServoPos = 0;
//
//    @Override
//    public void initialize() {
//        robot.wobbleGoalArm.liftArmServo.setPosition(0.5);
//    }
//
//    @Override
//    public void repeat() {
//        if(gamepad1.left_stick_y > 0){
//            liftArmServoPos = liftArmServoPos + 0.01;
//            if(liftArmServoPos > 1){
//                liftArmServoPos = 1;
//            }
//        }
//        else if(gamepad1.left_stick_y < 0){
//            liftArmServoPos = liftArmServoPos - 0.01;
//            if(liftArmServoPos < 0){
//                liftArmServoPos = 0;
//            }
//        }
//        robot.wobbleGoalArm.liftArmServo.setPosition(liftArmServoPos);
//
//        if(gamepad1.right_stick_y > 0){
//            gripperServoPos = gripperServoPos + 0.01;
//            if(gripperServoPos > 1) {
//                gripperServoPos = 1;
//            }
//        }
//        else if(gamepad1.right_stick_y < 0) {
//            gripperServoPos = gripperServoPos - 0.01;
//            if(gripperServoPos < 0) {
//                gripperServoPos = 0;
//            }
//        }
//        robot.wobbleGoalArm.gripperServo.setPosition(gripperServoPos);
//
//        telemetry.addData("liftArmServoPos", liftArmServoPos);
//        telemetry.addData("gripperServoPos", gripperServoPos);
//        telemetry.update();
//    }
//}
