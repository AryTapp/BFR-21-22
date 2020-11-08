package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Main.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Main.Hardware.Subsystems.Gyro;
import org.opencv.core.Mat;


public class Drive extends Subsystem {


    static final double TURNING_STALL_POWER = 0.2;

    static final double ARM_DIAMETER_INCHES = 24;
    static final double COUNTS_PER_ARM_MOTOR_REV = 1680; //1680;
    static final double ARM_GEAR_REDUCTION = 0.5;
    static final double COUNTS_PER_ARM_INCH = (COUNTS_PER_ARM_MOTOR_REV * ARM_GEAR_REDUCTION) /
            (ARM_DIAMETER_INCHES * 3.1415);
    static final double ARM_WIDTH = 24;
    static final double FULL_TURN_CIRCUMFERENCE = 3.1415 * ARM_WIDTH * 2;
    static final double FULL_TURN_COUNTS = FULL_TURN_CIRCUMFERENCE * COUNTS_PER_ARM_INCH;

    public final double GYRO_K_FACTOR_FORWARD = 0.0225;
    public final double GYRO_K_FACTOR_FORWARD_MECHANUM = 0.0225; //0.08
    public final double GYRO_K_FACTOR_BACK = 0.02;

    static final double IIR_SMOOTHER = 10;   // Smoothing factor in the IIR Smoothing calc
    static final double CORRECTION_FACTOR = 8000;   // Scales down the correction of speed
    //static final double POWER_SPEED_CONVERSION = 170;
    public static final double POWER_SPEED_CONVERSION = 150;
    public double startAngleHanging = 0.0625;
    public double startAngleAfterHanging = -2.125;


    //speed control variables
    double firstTime, secondTime;
    double firstPosRight, secondPosRight, firstPosLeft, secondPosLeft;
    double rightPower, leftPower;
    double currentRightSpeed, rightSpeedError, currentLeftSpeed, leftSpeedError;
    double avgRightSpeed, avgLeftSpeed;
    double targetSpeedRight, targetSpeedLeft;

    public boolean left;
    public double turnAngle;

    public boolean isStalled = false;


    public ElapsedTime runtime = new ElapsedTime();

    /**
     * drive and turn function constants
     */
    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Rev Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);


    /**
     * motors declaration
     */
    private DcMotor rightMotor;
    private DcMotor rightMotorR;
    private DcMotor leftMotor;
    private DcMotor leftMotorR;


    /**
     * Motor configuration names
     */
    private String lrName = "lr";
    private String lfName = "lf";
    private String rrName = "rr";
    private String rfName = "rf";


    /**
     * minimum drive powers
     */
    public final double MINIMUM_DRIVE_PWR = 0.22;

    @Override
    public void initialize(HardwareMap map,Telemetry telemetry) {

        rightMotor = map.dcMotor.get(rfName);
        rightMotorR = map.dcMotor.get(rrName);
        leftMotor = map.dcMotor.get(lfName);
        leftMotorR = map.dcMotor.get(lrName);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotorR.setDirection(DcMotor.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void sendTelemetry(Telemetry telemetry) {

    }


    public void setPower(double left, double right) {
        rightMotor.setPower(right);
        rightMotorR.setPower(right);
        leftMotor.setPower(left);
        leftMotorR.setPower(left);

    }

    /**
     * TODO: hard codisng a negation is probably a bad practice
     *
     * @return
     */
    public int getLeftCounts() {
        return Math.abs((leftMotorR.getCurrentPosition())); //+ leftMotorR.getCurrentPosition())/2);

    }

    /**
     * TODO: hard coding a negation is probably a bad practice
     *
     * @return
     */
    public int getRightCounts() {
        return Math.abs((rightMotorR.getCurrentPosition()));// + rightMotorR.getCurrentPosition())/2);

    }

    public double getLeftInches() {
        return getLeftCounts() / COUNTS_PER_INCH;

    }

    public double getRightInches() {
        return getRightCounts() / COUNTS_PER_INCH;

    }

    public void setMotorModes(DcMotor.RunMode mode) {

        leftMotor.setMode(mode);
        leftMotorR.setMode(mode);
        rightMotor.setMode(mode);
        rightMotorR.setMode(mode);

    }

    public void forward(double distance, double heading, double maximumPower, double stallTime, boolean detectStall, boolean isStop) {


        //distanceError = 0.0;

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPwrBhvrs(DcMotor.ZeroPowerBehavior.BRAKE);

        //make sure maximumPower is always positive and less than 1
        maximumPower = Math.abs(maximumPower);

        double targetPos = Math.abs(distance * COUNTS_PER_INCH);

        double d0 = 0.0;
        double d1 = 3 * COUNTS_PER_INCH;
        double d2 = targetPos * 0.77;
        double d3 = targetPos;

        double p0 = 0.2;
        double p1 = maximumPower;
        double p2 = maximumPower * 0.65;
        double p3 = 0.0;

        double c0 = p0;

        double k1 = (d2 * d1 * d1 * d1) - (d1 * d2 * d2 * d2); //(d2 * d1^3) - (d1 * d2^3)
        double k2 = (d2 * d2 * d1 * d1 * d1) - (d1 * d1 * d2 * d2 * d2); //(d2^2 * d1^3) - (d1^2 * d2^3)
        double k3 = (d1 * d1 * d1) * (p2 - c0) - (d2 * d2 * d2) * (p1 - c0);
        // d1^3 * (p2 - c0) - d2^3 * (p1 - c0)
        double k4 = (d3 * d1 * d1 * d1) - (d1 * d3 * d3 * d3); //(d3 * d1^3) - (d1 * d3^3)
        double k5 = (d3 * d3 * d1 * d1 * d1) - (d1 * d1 * d3 * d3 * d3); //(d3^2 * d1^3) - (d1^2 * d3^3)
        double k6 = (d1 * d1 * d1) * (p3 - c0) - (d3 * d3 * d3) * (p1 - c0);
        // d1^3 * (p3 - c0) - d3^3 * (p1 - c0)
        double c1 = ((k2 * k6) - (k3 * k5)) / ((k2 * k4) - (k5 * k1));
        double c2 = ((k3 - (k1 * c1)) / k2);
        double c3 = (p2 - c0 - (c1 * d2) - (c2 * d2 * d2)) / (d2 * d2 * d2);


        //initial motor positions
        double initPosrr = rightMotorR.getCurrentPosition();
        double initPoslr = leftMotorR.getCurrentPosition();

        //slope for proportional power decrease0
        double slope = (maximumPower - MINIMUM_DRIVE_PWR) / (Math.abs(targetPos));

        double distanceSign = Math.signum(distance);

        //current motor positions

        double currentRobotPosition = 0;

        double startTime = runtime.milliseconds();
        double targetTime = startTime + stallTime;
        double curTime = runtime.milliseconds();

        secondPosRight = rightMotorR.getCurrentPosition();
        secondPosLeft = leftMotorR.getCurrentPosition();
        secondTime = runtime.milliseconds();
        while (Math.abs(currentRobotPosition) < Math.abs(targetPos) && curTime < targetTime
                && !isStop) {

            curTime = runtime.milliseconds();

            //current motor positions (constantly updating)
            double curPosrr = rightMotorR.getCurrentPosition() - initPosrr;
            double curPoslr = leftMotorR.getCurrentPosition() - initPoslr;
            currentRobotPosition = (curPoslr + curPosrr) / 2;

            double distanceTraveled = Math.abs(currentRobotPosition);

            //power profile calc
            double power = c0 + c1 * (distanceTraveled) + c2 *
                    (distanceTraveled * distanceTraveled) +
                    c3 * (distanceTraveled * distanceTraveled * distanceTraveled);

            if (power > 1.0)
                power = 1.0;

            if (power >= maximumPower)
                power = maximumPower;

            if (Math.abs(distance) < 9)
                power = 0.25;

            firstPosRight = secondPosRight;
            firstPosLeft = secondPosLeft;

            firstTime = secondTime;

            secondPosRight = rightMotorR.getCurrentPosition();
            secondPosLeft = leftMotorR.getCurrentPosition();

            secondTime = runtime.milliseconds();

            /* Right Front Motor Calculation */
            currentRightSpeed = (secondPosRight - firstPosRight) / (secondTime - firstTime);
            currentRightSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedRight = Math.abs(power * POWER_SPEED_CONVERSION);

            rightSpeedError = Math.abs(targetSpeedRight) - Math.abs(currentRightSpeed);
            rightPower = Math.abs(power) + rightSpeedError / CORRECTION_FACTOR;

            /* Left Motor Calculation */
            currentLeftSpeed = (secondPosLeft - firstPosLeft) / (secondTime - firstTime);
            currentLeftSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedLeft = Math.abs(power * POWER_SPEED_CONVERSION);

            leftSpeedError = Math.abs(targetSpeedLeft) - Math.abs(currentLeftSpeed);
            leftPower = Math.abs(power) + leftSpeedError / CORRECTION_FACTOR;

            double currentHeading = RobotHardware.getInstance().gyro.getAngle();
//if error is negative, then robot is on the right, so you must turn left.
            //same for positive error
            /** @TODO change second heading in compute gyro drive correction to current heading*/
            double gyroCorrectionPwr = GYRO_K_FACTOR_FORWARD
                    * computeGyroDriveCorrectionError(heading, currentHeading);

            leftMotor.setPower(((leftPower + (gyroCorrectionPwr * distanceSign))
                    * distanceSign));
            leftMotorR.setPower(((leftPower + (gyroCorrectionPwr * distanceSign))
                    * distanceSign));
            rightMotor.setPower((rightPower - (gyroCorrectionPwr * distanceSign))
                    * distanceSign);
            rightMotorR.setPower((rightPower - (gyroCorrectionPwr * distanceSign))
                    * distanceSign);

            if (detectStall) {
                if (curTime - startTime > 500) {
                    if (Math.abs(currentLeftSpeed) < 0.1 || Math.abs(currentRightSpeed) < 0.1)
                        break;
                }
            }

//            telemetry.addData("Target Pos: ", targetPos);
//            telemetry.addData("Current Robot Position: ", currentRobotPosition);
//            //telemetry.addData( "Rf " , curPosrf );
//            telemetry.addData("Rr", curPosrr);
//            //telemetry.addData("Lf " , curPoslf );
//            telemetry.addData("Lr", curPoslr);
//            telemetry.addData("Current Power", power);
//            telemetry.addData("Slope", slope);
//            telemetry.addData("heading", heading);
//            telemetry.addData(">", "Current Right Speed = " + currentRightSpeed);
//            telemetry.addData(">", "Current Right Speed = " + currentLeftSpeed);
//            //telemetry.addData("heading Error", headingError);
//            telemetry.update();
//
//            idle();

        }

//        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        rightMotorR.setPower(0);
        leftMotorR.setPower(0);

    }

    public void backward(double distance, double heading, double maximumPower, double stallTime, boolean detectStall, boolean isStop) {


        //distance += distanceError;

        //distanceError = 0.0;

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPwrBhvrs(DcMotor.ZeroPowerBehavior.BRAKE);

        //make sure maximumPower is always positive and less than 1
        maximumPower = Math.abs(maximumPower);

        double targetPos = Math.abs((distance * COUNTS_PER_INCH)/* * robot.DRIVE_DISTANCE_FACTOR*/);

        double d0 = 0.0;
        double d1 = 3 * COUNTS_PER_INCH;
        double d2 = targetPos * 0.77;
        double d3 = targetPos;

        double p0 = 0.2;
        double p1 = maximumPower;
        double p2 = maximumPower * 0.65;
        double p3 = 0.0;

        double c0 = p0;

        double k1 = (d2 * d1 * d1 * d1) - (d1 * d2 * d2 * d2); //(d2 * d1^3) - (d1 * d2^3)
        double k2 = (d2 * d2 * d1 * d1 * d1) - (d1 * d1 * d2 * d2 * d2); //(d2^2 * d1^3) - (d1^2 * d2^3)
        double k3 = (d1 * d1 * d1) * (p2 - c0) - (d2 * d2 * d2) * (p1 - c0);
        // d1^3 * (p2 - c0) - d2^3 * (p1 - c0)
        double k4 = (d3 * d1 * d1 * d1) - (d1 * d3 * d3 * d3); //(d3 * d1^3) - (d1 * d3^3)
        double k5 = (d3 * d3 * d1 * d1 * d1) - (d1 * d1 * d3 * d3 * d3); //(d3^2 * d1^3) - (d1^2 * d3^3)
        double k6 = (d1 * d1 * d1) * (p3 - c0) - (d3 * d3 * d3) * (p1 - c0);
        // d1^3 * (p3 - c0) - d3^3 * (p1 - c0)
        double c1 = ((k2 * k6) - (k3 * k5)) / ((k2 * k4) - (k5 * k1));
        double c2 = ((k3 - (k1 * c1)) / k2);
        double c3 = (p2 - c0 - (c1 * d2) - (c2 * d2 * d2)) / (d2 * d2 * d2);


        //initial motor positions
        double initPosrr = rightMotorR.getCurrentPosition();
        double initPoslr = leftMotorR.getCurrentPosition();

        //slope for proportional power decrease0
        double slope = (maximumPower - MINIMUM_DRIVE_PWR) / (Math.abs(targetPos));
        //current motor positions

        double currentRobotPosition = 0;
        double distanceSign = Math.signum(distance);

        double startTime = runtime.milliseconds();
        double targetTime = startTime + stallTime;
        double curTime = runtime.milliseconds();

        secondPosRight = rightMotorR.getCurrentPosition();
        secondPosLeft = leftMotorR.getCurrentPosition();
        secondTime = runtime.milliseconds();
        while (Math.abs(currentRobotPosition) < Math.abs(targetPos) && curTime < targetTime
                && !isStop) {

            curTime = runtime.milliseconds();


            //current motor positions (constantly updating)
            double curPosrr = rightMotorR.getCurrentPosition() - initPosrr;
            double curPoslr = leftMotorR.getCurrentPosition() - initPoslr;
            currentRobotPosition = (curPoslr + curPosrr) / 2;

            double distanceTraveled = Math.abs(currentRobotPosition);

            //power declarations

            double power = c0 + c1 * (distanceTraveled) + c2 * (distanceTraveled * distanceTraveled) +
                    c3 * (distanceTraveled * distanceTraveled * distanceTraveled);

            if (power >= maximumPower)
                power = maximumPower;

            if (Math.abs(distance) < 9)
                power = 0.25;

            firstPosRight = secondPosRight;
            firstPosLeft = secondPosLeft;

            firstTime = secondTime;

            secondPosRight = rightMotorR.getCurrentPosition();
            secondPosLeft = leftMotorR.getCurrentPosition();

            secondTime = runtime.milliseconds();

            /* Right Front Motor Calculation */
            currentRightSpeed = (secondPosRight - firstPosRight) / (secondTime - firstTime);
            currentRightSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            //avgRightSpeed = (currentRightSpeed / IIR_SMOOTHER) + (avgRightSpeed * (IIR_SMOOTHER - 1) / IIR_SMOOTHER);

            targetSpeedRight = Math.abs(power * POWER_SPEED_CONVERSION);

            rightSpeedError = Math.abs(targetSpeedRight) - Math.abs(currentRightSpeed);
            rightPower = Math.abs(power) + rightSpeedError / CORRECTION_FACTOR;

            /* Left Motor Calculation */
            currentLeftSpeed = (secondPosLeft - firstPosLeft) / (secondTime - firstTime);
            currentLeftSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            //avgLeftSpeed = (currentLeftSpeed / IIR_SMOOTHER) + (avgLeftSpeed * (IIR_SMOOTHER - 1) / IIR_SMOOTHER);

            targetSpeedLeft = Math.abs(power * POWER_SPEED_CONVERSION);

            leftSpeedError = Math.abs(targetSpeedLeft) - Math.abs(currentLeftSpeed);
            leftPower = Math.abs(power) + leftSpeedError / CORRECTION_FACTOR;

            double currentHeading = RobotHardware.getInstance().gyro.getAngle();

            //if error is negative, then robot is on the right,
            // so you must turn left. same for positive error

            /** @TODO change second heading in compute gyro drive correction to current heading*/
            double gyroCorrectionPwr = GYRO_K_FACTOR_FORWARD * computeGyroDriveCorrectionError(heading, currentHeading);

            leftMotor.setPower(((leftPower + (gyroCorrectionPwr * distanceSign)) * distanceSign));
            leftMotorR.setPower(((leftPower + (gyroCorrectionPwr * distanceSign)) * distanceSign));
            rightMotor.setPower((rightPower - (gyroCorrectionPwr * distanceSign)) * distanceSign);
            rightMotorR.setPower((rightPower - (gyroCorrectionPwr * distanceSign)) * distanceSign);

            if (detectStall) {
                if (curTime - startTime > 500) {
                    if (Math.abs(currentLeftSpeed) < 1. || Math.abs(currentRightSpeed) < 1.)
                        break;
                }
            }

//            telemetry.addData("Target Pos: ", targetPos);
//            telemetry.addData("Current Robot Position: ", currentRobotPosition);
//            //telemetry.addData( "Rf " , curPosrf );
//            telemetry.addData("Rr", curPosrr);
//            //telemetry.addData("Lf " , curPoslf );
//            telemetry.addData("Lr", curPoslr);
//            telemetry.addData("Current Power", power);
//            telemetry.addData("Slope", slope);
//            telemetry.addData("heading", heading);
//            telemetry.addData(">", "Current Right Speed = " + currentRightSpeed);
//            telemetry.addData(">", "Current Right Speed = " + currentLeftSpeed);
//            //telemetry.addData("heading Error", headingError);
//            telemetry.update();
//
//            idle();

        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);
        rightMotorR.setPower(0);
        leftMotorR.setPower(0);

    }

    public void turn(double angle, double maximumPower, double maxTime, boolean opModeActive, boolean isStop) {
        if(angle < 0){
            turnLeft(Math.abs(angle), maximumPower, maxTime, opModeActive, isStop);
        }else{
            turnRight(angle, maximumPower, maxTime, opModeActive, isStop);
        }
    }


    public void turnRight(double angle, double maximumPower, double maxTime, boolean opModeActive, boolean isStop) {

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double initAngle = RobotHardware.getInstance().gyro.getAngle();

        //make sure maximumPower is always positive and less than 1
        Range.clip(maximumPower, 0.0, 1.0);

        //angle measurements
        double curAngle = RobotHardware.getInstance().gyro.getAngle();

        double d0 = 0.0;
        double d1 = Math.abs(angle * 0.3);
        double d2 = Math.abs(angle * 0.54);
        double d3 = Math.abs(angle);

        double p0 = 0.4 * maximumPower;
        double p1 = maximumPower * 0.82;
        double p2 = maximumPower * 0.61;
        double p3 = TURNING_STALL_POWER;

        double c0 = p0;

        double k1 = (d2 * d1 * d1 * d1) - (d1 * d2 * d2 * d2); //(d2 * d1^3) - (d1 * d2^3)
        double k2 = (d2 * d2 * d1 * d1 * d1) - (d1 * d1 * d2 * d2 * d2); //(d2^2 * d1^3) - (d1^2 * d2^3)
        double k3 = (d1 * d1 * d1) * (p2 - c0) - (d2 * d2 * d2) * (p1 - c0);
        // d1^3 * (p2 - c0) - d2^3 * (p1 - c0)
        double k4 = (d3 * d1 * d1 * d1) - (d1 * d3 * d3 * d3); //(d3 * sd1^3) - (d1 * d3^3)
        double k5 = (d3 * d3 * d1 * d1 * d1) - (d1 * d1 * d3 * d3 * d3); //(d3^2 * d1^3) - (d1^2 * d3^3)
        double k6 = (d1 * d1 * d1) * (p3 - c0) - (d3 * d3 * d3) * (p1 - c0);
        // d1^3 * (p3 - c0) - d3^3 * (p1 - c0)
        double c1 = ((k2 * k6) - (k3 * k5)) / ((k2 * k4) - (k5 * k1));
        double c2 = ((k3 - (k1 * c1)) / k2);
        double c3 = (p2 - c0 - (c1 * d2) - (c2 * d2 * d2)) / (d2 * d2 * d2);

        double referenceAngle = 0.0;

        //slope for proportional power decrease

        double startTime = runtime.milliseconds();
        double targetTime = startTime + maxTime;
        double curTime = runtime.milliseconds();


        double angleSign = 1;

        secondPosRight = rightMotorR.getCurrentPosition();
        secondPosLeft = leftMotorR.getCurrentPosition();
        secondTime = runtime.milliseconds();
        while (referenceAngle < Math.abs(angle) && opModeActive && curTime < targetTime
                && !isStop) {

            curTime = runtime.milliseconds();


            curAngle = RobotHardware.getInstance().gyro.getAngle();

            if (curAngle * initAngle >= 0)
                referenceAngle = Math.abs(curAngle - initAngle);
            else {
                if (curAngle > 0)
                    referenceAngle = (180 - curAngle) + (initAngle + 180);
                else
                    referenceAngle = initAngle - curAngle;
            }


            double power = c0 + c1 * (referenceAngle) + c2 * (referenceAngle * referenceAngle) +
                    c3 * (referenceAngle * referenceAngle * referenceAngle);

            if (power >= maximumPower)
                power = maximumPower;

            firstPosRight = secondPosRight;
            firstPosLeft = secondPosLeft;

            firstTime = secondTime;

            secondPosRight = rightMotorR.getCurrentPosition();
            secondPosLeft = leftMotorR.getCurrentPosition();

            secondTime = runtime.milliseconds();

            /* Right Front Motor Calculation */
            currentRightSpeed = (secondPosRight - firstPosRight) / (secondTime - firstTime);
            currentRightSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedRight = Math.abs(power * POWER_SPEED_CONVERSION);

            rightSpeedError = Math.abs(targetSpeedRight) - Math.abs(currentRightSpeed);
            rightPower = Math.abs(power) + rightSpeedError / CORRECTION_FACTOR;

            /* Left Motor Calculation */
            currentLeftSpeed = (secondPosLeft - firstPosLeft) / (secondTime - firstTime);
            currentLeftSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedLeft = Math.abs(power * POWER_SPEED_CONVERSION);

            leftSpeedError = Math.abs(targetSpeedLeft) - Math.abs(currentLeftSpeed);
            leftPower = Math.abs(power) + leftSpeedError / CORRECTION_FACTOR;

            leftMotor.setPower(leftPower * angleSign);
            leftMotorR.setPower(leftPower * angleSign);
            rightMotor.setPower(-rightPower * angleSign);
            rightMotorR.setPower(-rightPower * angleSign);

            if (curTime - startTime > 500) {
                if (Math.abs(currentLeftSpeed) < 10 || Math.abs(currentRightSpeed) < 10)
                    break;
            }

//            telemetry.addData("Target Angle: ", angle);
//            telemetry.addData("Current Angle: ", curAngle);
//            telemetry.addData("Current Power", power);
//            //telemetry.addData("Slope", slope);j
//            telemetry.addData(">", "C0 = " + c0);
//            telemetry.addData(">", "C1 = " + c1);
//            telemetry.addData(">", "C2 = " + c2);
//            telemetry.addData(">", "C3 = " + c3);
//            telemetry.update();
//
//            idle();

        }

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        leftMotorR.setPower(0.0);
        rightMotorR.setPower(0.0);
    }

    public void turnLeft(double angle, double maximumPower, double maxTime, boolean opModeActive, boolean isStop) {

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double initAngle = RobotHardware.getInstance().gyro.getAngle();

        //make sure maximumPower is always positive and less than 1
        Range.clip(maximumPower, 0.0, 1.0);

        //angle measurements
        double curAngle = RobotHardware.getInstance().gyro.getAngle();

        double d0 = 0.0;
        double d1 = Math.abs(angle * 0.3);
        double d2 = Math.abs(angle * 0.54);
        double d3 = Math.abs(angle);

        double p0 = 0.4 * maximumPower;//maximumPower;
        double p1 = maximumPower * 0.82;
        double p2 = maximumPower * 0.61;
        double p3 = TURNING_STALL_POWER;

        double c0 = p0;

        double k1 = (d2 * d1 * d1 * d1) - (d1 * d2 * d2 * d2); //(d2 * d1^3) - (d1 * d2^3)
        double k2 = (d2 * d2 * d1 * d1 * d1) - (d1 * d1 * d2 * d2 * d2); //(d2^2 * d1^3) - (d1^2 * d2^3)
        double k3 = (d1 * d1 * d1) * (p2 - c0) - (d2 * d2 * d2) * (p1 - c0);
        // d1^3 * (p2 - c0) - d2^3 * (p1 - c0)
        double k4 = (d3 * d1 * d1 * d1) - (d1 * d3 * d3 * d3); //(d3 * d1^3) - (d1 * d3^3)
        double k5 = (d3 * d3 * d1 * d1 * d1) - (d1 * d1 * d3 * d3 * d3); //(d3^2 * d1^3) - (d1^2 * d3^3)
        double k6 = (d1 * d1 * d1) * (p3 - c0) - (d3 * d3 * d3) * (p1 - c0);
        // d1^3 * (p3 - c0) - d3^3 * (p1 - c0)
        double c1 = ((k2 * k6) - (k3 * k5)) / ((k2 * k4) - (k5 * k1));
        double c2 = ((k3 - (k1 * c1)) / k2);
        double c3 = (p2 - c0 - (c1 * d2) - (c2 * d2 * d2)) / (d2 * d2 * d2);

        double referenceAngle = 0.0;

        double startTime = runtime.milliseconds();
        double targetTime = startTime + maxTime;
        double curTime = runtime.milliseconds();

        double angleSign = -1;

        secondPosRight = rightMotorR.getCurrentPosition();
        secondPosLeft = leftMotorR.getCurrentPosition();
        secondTime = runtime.milliseconds();
        while (referenceAngle < Math.abs(angle) && opModeActive && curTime < targetTime
                && !isStop) {

            curTime = runtime.milliseconds();


            curAngle = RobotHardware.getInstance().gyro.getAngle();

            if (curAngle * initAngle >= 0)
                referenceAngle = Math.abs(curAngle - initAngle);
            else {
                if (curAngle > 0)
                    referenceAngle = curAngle - initAngle;
                else
                    referenceAngle = (180 - initAngle) + (curAngle + 180);
            }

            double power = c0 + c1 * (referenceAngle) + c2 * (referenceAngle * referenceAngle) +
                    c3 * (referenceAngle * referenceAngle * referenceAngle);

            if (power >= maximumPower)
                power = maximumPower;

            firstPosRight = secondPosRight;
            firstPosLeft = secondPosLeft;

            firstTime = secondTime;

            secondPosRight = rightMotorR.getCurrentPosition();
            secondPosLeft = leftMotorR.getCurrentPosition();

            secondTime = runtime.milliseconds();

            /* Right Front Motor Calculation */
            currentRightSpeed = (secondPosRight - firstPosRight) / (secondTime - firstTime);
            currentRightSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedRight = Math.abs(power * POWER_SPEED_CONVERSION);

            rightSpeedError = Math.abs(targetSpeedRight) - Math.abs(currentRightSpeed);
            rightPower = Math.abs(power) + rightSpeedError / CORRECTION_FACTOR;

            /* Left Motor Calculation */
            currentLeftSpeed = (secondPosLeft - firstPosLeft) / (secondTime - firstTime);
            currentLeftSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedLeft = Math.abs(power * POWER_SPEED_CONVERSION);

            leftSpeedError = Math.abs(targetSpeedLeft) - Math.abs(currentLeftSpeed);
            leftPower = Math.abs(power) + leftSpeedError / CORRECTION_FACTOR;

            leftMotor.setPower(leftPower * angleSign);
            leftMotorR.setPower(leftPower * angleSign);
            rightMotor.setPower(-rightPower * angleSign);
            rightMotorR.setPower(-rightPower * angleSign);

            if (curTime - startTime > 500) {
                if (Math.abs(currentLeftSpeed) < 10 || Math.abs(currentRightSpeed) < 10)
                    break;
            }

//            telemetry.addData("Target Angle: ", angle);
//            telemetry.addData("Current Angle: ", curAngle);
//            telemetry.addData("Current Power", power);
//            telemetry.addData("Reference Angle", referenceAngle);
//            //telemetry.addData("Slope", slope);
//            telemetry.addData(">", "C0 = " + c0);
//            telemetry.addData(">", "C1 = " + c1);
//            telemetry.addData(">", "C2 = " + c2);
//            telemetry.addData(">", "C3 = " + c3);
//            telemetry.update();


        }

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        leftMotorR.setPower(0.0);
        rightMotorR.setPower(0.0);

    }

    public void forwardShifted(double distance, double heading, double maximumPower, double stallTime, boolean detectStall, boolean isStop) {


        //distanceError = 0.0;

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPwrBhvrs(DcMotor.ZeroPowerBehavior.BRAKE);

        //make sure maximumPower is always positive and less than 1
        maximumPower = Math.abs(maximumPower);

        double targetPos = Math.abs(distance * COUNTS_PER_INCH);

        double d0 = 0.0;
        double d1 = 3 * COUNTS_PER_INCH;
        double d2 = targetPos * 0.45;
        double d3 = targetPos;

        double p0 = 0.2;
        double p1 = maximumPower;
        double p2 = maximumPower * 0.65;
        double p3 = 0.0;

        double c0 = p0;

        double k1 = (d2 * d1 * d1 * d1) - (d1 * d2 * d2 * d2); //(d2 * d1^3) - (d1 * d2^3)
        double k2 = (d2 * d2 * d1 * d1 * d1) - (d1 * d1 * d2 * d2 * d2); //(d2^2 * d1^3) - (d1^2 * d2^3)
        double k3 = (d1 * d1 * d1) * (p2 - c0) - (d2 * d2 * d2) * (p1 - c0);
        // d1^3 * (p2 - c0) - d2^3 * (p1 - c0)
        double k4 = (d3 * d1 * d1 * d1) - (d1 * d3 * d3 * d3); //(d3 * d1^3) - (d1 * d3^3)
        double k5 = (d3 * d3 * d1 * d1 * d1) - (d1 * d1 * d3 * d3 * d3); //(d3^2 * d1^3) - (d1^2 * d3^3)
        double k6 = (d1 * d1 * d1) * (p3 - c0) - (d3 * d3 * d3) * (p1 - c0);
        // d1^3 * (p3 - c0) - d3^3 * (p1 - c0)
        double c1 = ((k2 * k6) - (k3 * k5)) / ((k2 * k4) - (k5 * k1));
        double c2 = ((k3 - (k1 * c1)) / k2);
        double c3 = (p2 - c0 - (c1 * d2) - (c2 * d2 * d2)) / (d2 * d2 * d2);


        //initial motor positions
        double initPosrr = rightMotorR.getCurrentPosition();
        double initPoslr = leftMotorR.getCurrentPosition();

        //slope for proportional power decrease0
        double slope = (maximumPower - MINIMUM_DRIVE_PWR) / (Math.abs(targetPos));

        double distanceSign = Math.signum(distance);

        //current motor positions

        double currentRobotPosition = 0;

        double startTime = runtime.milliseconds();
        double targetTime = startTime + stallTime;
        double curTime = runtime.milliseconds();

        secondPosRight = rightMotorR.getCurrentPosition();
        secondPosLeft = leftMotorR.getCurrentPosition();
        secondTime = runtime.milliseconds();
        while (Math.abs(currentRobotPosition) < Math.abs(targetPos) && curTime < targetTime
                && !isStop) {

            curTime = runtime.milliseconds();

            //current motor positions (constantly updating)
            double curPosrr = rightMotorR.getCurrentPosition() - initPosrr;
            double curPoslr = leftMotorR.getCurrentPosition() - initPoslr;
            currentRobotPosition = (curPoslr + curPosrr) / 2;

            double distanceTraveled = Math.abs(currentRobotPosition);

            //power profile calc
            double power = c0 + c1 * (distanceTraveled) + c2 *
                    (distanceTraveled * distanceTraveled) +
                    c3 * (distanceTraveled * distanceTraveled * distanceTraveled);

            if (power > 1.0)
                power = 1.0;

            if (power >= maximumPower)
                power = maximumPower;

            if (Math.abs(distance) < 9)
                power = 0.25;

            firstPosRight = secondPosRight;
            firstPosLeft = secondPosLeft;

            firstTime = secondTime;

            secondPosRight = rightMotorR.getCurrentPosition();
            secondPosLeft = leftMotorR.getCurrentPosition();

            secondTime = runtime.milliseconds();

            /* Right Front Motor Calculation */
            currentRightSpeed = (secondPosRight - firstPosRight) / (secondTime - firstTime);
            currentRightSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedRight = Math.abs(power * POWER_SPEED_CONVERSION);

            rightSpeedError = Math.abs(targetSpeedRight) - Math.abs(currentRightSpeed);
            rightPower = Math.abs(power) + rightSpeedError / CORRECTION_FACTOR;

            /* Left Motor Calculation */
            currentLeftSpeed = (secondPosLeft - firstPosLeft) / (secondTime - firstTime);
            currentLeftSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedLeft = Math.abs(power * POWER_SPEED_CONVERSION);

            leftSpeedError = Math.abs(targetSpeedLeft) - Math.abs(currentLeftSpeed);
            leftPower = Math.abs(power) + leftSpeedError / CORRECTION_FACTOR;

            double currentHeading = RobotHardware.getInstance().gyro.getAngle();
//if error is negative, then robot is on the right, so you must turn left.
            //same for positive error
            /** @TODO change second heading in compute gyro drive correction to current heading*/
            double gyroCorrectionPwr = GYRO_K_FACTOR_FORWARD
                    * computeGyroDriveCorrectionError(heading, currentHeading);

            leftMotor.setPower(((leftPower + (gyroCorrectionPwr * distanceSign))
                    * distanceSign));
            leftMotorR.setPower(((leftPower + (gyroCorrectionPwr * distanceSign))
                    * distanceSign));
            rightMotor.setPower((rightPower - (gyroCorrectionPwr * distanceSign))
                    * distanceSign);
            rightMotorR.setPower((rightPower - (gyroCorrectionPwr * distanceSign))
                    * distanceSign);

            if (detectStall) {
                if (curTime - startTime > 500) {
                    if (Math.abs(currentLeftSpeed) < 0.1 || Math.abs(currentRightSpeed) < 0.1)
                        break;
                }
            }

//            telemetry.addData("Target Pos: ", targetPos);
//            telemetry.addData("Current Robot Position: ", currentRobotPosition);
//            //telemetry.addData( "Rf " , curPosrf );
//            telemetry.addData("Rr", curPosrr);
//            //telemetry.addData("Lf " , curPoslf );
//            telemetry.addData("Lr", curPoslr);
//            telemetry.addData("Current Power", power);
//            telemetry.addData("Slope", slope);
//            telemetry.addData("heading", heading);
//            telemetry.addData(">", "Current Right Speed = " + currentRightSpeed);
//            telemetry.addData(">", "Current Right Speed = " + currentLeftSpeed);
//            //telemetry.addData("heading Error", headingError);
//            telemetry.update();
//
//            idle();

        }

//        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        rightMotorR.setPower(0);
        leftMotorR.setPower(0);

    }

    public void barc(double distance, double startHeading, double endHeading, double whenTurn, double maximumPower, double stallTime, boolean detectStall, boolean isStop) {


        //distance += distanceError;

        //distanceError = 0.0;

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPwrBhvrs(DcMotor.ZeroPowerBehavior.BRAKE);

        //make sure maximumPower is always positive and less than 1
        maximumPower = Math.abs(maximumPower);

        double targetPos = Math.abs((distance * COUNTS_PER_INCH)/* * robot.DRIVE_DISTANCE_FACTOR*/);

        double d0 = 0.0;
        double d1 = 3 * COUNTS_PER_INCH;
        double d2 = targetPos * 0.77;
        double d3 = targetPos;

        double p0 = 0.2;
        double p1 = maximumPower;
        double p2 = maximumPower * 0.65;
        double p3 = 0.0;

        double c0 = p0;

        double k1 = (d2 * d1 * d1 * d1) - (d1 * d2 * d2 * d2); //(d2 * d1^3) - (d1 * d2^3)
        double k2 = (d2 * d2 * d1 * d1 * d1) - (d1 * d1 * d2 * d2 * d2); //(d2^2 * d1^3) - (d1^2 * d2^3)
        double k3 = (d1 * d1 * d1) * (p2 - c0) - (d2 * d2 * d2) * (p1 - c0);
        // d1^3 * (p2 - c0) - d2^3 * (p1 - c0)
        double k4 = (d3 * d1 * d1 * d1) - (d1 * d3 * d3 * d3); //(d3 * d1^3) - (d1 * d3^3)
        double k5 = (d3 * d3 * d1 * d1 * d1) - (d1 * d1 * d3 * d3 * d3); //(d3^2 * d1^3) - (d1^2 * d3^3)
        double k6 = (d1 * d1 * d1) * (p3 - c0) - (d3 * d3 * d3) * (p1 - c0);
        // d1^3 * (p3 - c0) - d3^3 * (p1 - c0)
        double c1 = ((k2 * k6) - (k3 * k5)) / ((k2 * k4) - (k5 * k1));
        double c2 = ((k3 - (k1 * c1)) / k2);
        double c3 = (p2 - c0 - (c1 * d2) - (c2 * d2 * d2)) / (d2 * d2 * d2);

        //slope for heading change
        double turnStart = whenTurn * targetPos;
        double headingSlope = endHeading/(targetPos - turnStart);

        double heading;


        //initial motor positions
        double initPosrr = rightMotorR.getCurrentPosition();
        double initPoslr = leftMotorR.getCurrentPosition();

        //slope for proportional power decrease0
        double slope = (maximumPower - MINIMUM_DRIVE_PWR) / (Math.abs(targetPos));
        //current motor positions

        double currentRobotPosition = 0;
        double distanceSign = Math.signum(distance);

        double startTime = runtime.milliseconds();
        double targetTime = startTime + stallTime;
        double curTime = runtime.milliseconds();

        secondPosRight = rightMotorR.getCurrentPosition();
        secondPosLeft = leftMotorR.getCurrentPosition();
        secondTime = runtime.milliseconds();
        while (Math.abs(currentRobotPosition) < Math.abs(targetPos) && curTime < targetTime
                && !isStop) {

            curTime = runtime.milliseconds();


            //current motor positions (constantly updating)
            double curPosrr = rightMotorR.getCurrentPosition() - initPosrr;
            double curPoslr = leftMotorR.getCurrentPosition() - initPoslr;
            currentRobotPosition = (curPoslr + curPosrr) / 2;

            double distanceTraveled = Math.abs(currentRobotPosition);

            //power declarations

            double power = c0 + c1 * (distanceTraveled) + c2 * (distanceTraveled * distanceTraveled) +
                    c3 * (distanceTraveled * distanceTraveled * distanceTraveled);

            if (power >= maximumPower)
                power = maximumPower;

            if (Math.abs(distance) < 9)
                power = 0.25;

            if((headingSlope * distanceTraveled) > startHeading){
                heading = startHeading;
            }else{
                heading = headingSlope * distanceTraveled;
            }

            firstPosRight = secondPosRight;
            firstPosLeft = secondPosLeft;

            firstTime = secondTime;

            secondPosRight = rightMotorR.getCurrentPosition();
            secondPosLeft = leftMotorR.getCurrentPosition();

            secondTime = runtime.milliseconds();

            /* Right Front Motor Calculation */
            currentRightSpeed = (secondPosRight - firstPosRight) / (secondTime - firstTime);
            currentRightSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            //avgRightSpeed = (currentRightSpeed / IIR_SMOOTHER) + (avgRightSpeed * (IIR_SMOOTHER - 1) / IIR_SMOOTHER);

            targetSpeedRight = Math.abs(power * POWER_SPEED_CONVERSION);

            rightSpeedError = Math.abs(targetSpeedRight) - Math.abs(currentRightSpeed);
            rightPower = Math.abs(power) + rightSpeedError / CORRECTION_FACTOR;

            /* Left Motor Calculation */
            currentLeftSpeed = (secondPosLeft - firstPosLeft) / (secondTime - firstTime);
            currentLeftSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            //avgLeftSpeed = (currentLeftSpeed / IIR_SMOOTHER) + (avgLeftSpeed * (IIR_SMOOTHER - 1) / IIR_SMOOTHER);

            targetSpeedLeft = Math.abs(power * POWER_SPEED_CONVERSION);

            leftSpeedError = Math.abs(targetSpeedLeft) - Math.abs(currentLeftSpeed);
            leftPower = Math.abs(power) + leftSpeedError / CORRECTION_FACTOR;

            double currentHeading = RobotHardware.getInstance().gyro.getAngle();

            //if error is negative, then robot is on the right,
            // so you must turn left. same for positive error

            /** @TODO change second heading in compute gyro drive correction to current heading*/
            double gyroCorrectionPwr = GYRO_K_FACTOR_FORWARD * computeGyroDriveCorrectionError(heading, currentHeading);

            leftMotor.setPower(((leftPower + (gyroCorrectionPwr * distanceSign)) * distanceSign));
            leftMotorR.setPower(((leftPower + (gyroCorrectionPwr * distanceSign)) * distanceSign));
            rightMotor.setPower((rightPower - (gyroCorrectionPwr * distanceSign)) * distanceSign);
            rightMotorR.setPower((rightPower - (gyroCorrectionPwr * distanceSign)) * distanceSign);

            if (detectStall) {
                if (curTime - startTime > 500) {
                    if (Math.abs(currentLeftSpeed) < 1. || Math.abs(currentRightSpeed) < 1.)
                        break;
                }
            }

//            telemetry.addData("Target Pos: ", targetPos);
//            telemetry.addData("Current Robot Position: ", currentRobotPosition);
//            //telemetry.addData( "Rf " , curPosrf );
//            telemetry.addData("Rr", curPosrr);
//            //telemetry.addData("Lf " , curPoslf );
//            telemetry.addData("Lr", curPoslr);
//            telemetry.addData("Current Power", power);
//            telemetry.addData("Slope", slope);
//            telemetry.addData("heading", heading);
//            telemetry.addData(">", "Current Right Speed = " + currentRightSpeed);
//            telemetry.addData(">", "Current Right Speed = " + currentLeftSpeed);
//            //telemetry.addData("heading Error", headingError);
//            telemetry.update();
//
//            idle();

        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);
        rightMotorR.setPower(0);
        leftMotorR.setPower(0);

    }


    public void setMotorZeroPwrBhvrs(DcMotor.ZeroPowerBehavior zeroPwrBhvr) {
        rightMotor.setZeroPowerBehavior(zeroPwrBhvr);
        rightMotorR.setZeroPowerBehavior(zeroPwrBhvr);
        leftMotor.setZeroPowerBehavior(zeroPwrBhvr);
        leftMotorR.setZeroPowerBehavior(zeroPwrBhvr);
    }

    double computeGyroDriveCorrectionError(double inputHeading, double currentHeading) {

        double error;
        if (inputHeading * currentHeading >= 0)
            error = currentHeading - inputHeading;
        else {
            if (Math.abs(inputHeading) > 90) {
                if (inputHeading < 0)
                    error = -((180 - currentHeading) + (180 + inputHeading));
                else
                    error = (180 + currentHeading) + (180 - inputHeading);
            } else
                error = currentHeading - inputHeading;
        }

        return error;
    }

    public void arcadeDrive(double moveValue, double rotateValue) {
        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = Range.clip(moveValue, -1, 1);
        rotateValue = Range.clip(rotateValue, -1, 1);

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }
        setPower(leftMotorSpeed, rightMotorSpeed);
    }

    public void POV(double drive, double turn) {

    // Combine drive and turn for blended motion.
    double left  =drive +turn;
    double right =drive -turn;

    // Normalize the values so neither exceed +/- 1.0
    double max =Math.max(Math.abs(left),Math.abs(right));
            if(max >1.0)
            {
                left /= max;
                right /= max;
        }

    // Output the safe vales to the motor drives.
            setPower(left, right);
    }



    public void forwardWithRun(double distance, double heading, double maximumPower, double stallTime, boolean detectStall, boolean isStop, Runnable runnable, double postionToRun) {


        //distanceError = 0.0;

        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPwrBhvrs(DcMotor.ZeroPowerBehavior.BRAKE);

        //make sure maximumPower is always positive and less than 1
        maximumPower = Math.abs(maximumPower);

        double targetPos = Math.abs(distance * COUNTS_PER_INCH);

        double d0 = 0.0;
        double d1 = 3 * COUNTS_PER_INCH;
        double d2 = targetPos * 0.77;
        double d3 = targetPos;

        double p0 = 0.2;
        double p1 = maximumPower;
        double p2 = maximumPower * 0.65;
        double p3 = 0.0;

        double c0 = p0;

        double k1 = (d2 * d1 * d1 * d1) - (d1 * d2 * d2 * d2); //(d2 * d1^3) - (d1 * d2^3)
        double k2 = (d2 * d2 * d1 * d1 * d1) - (d1 * d1 * d2 * d2 * d2); //(d2^2 * d1^3) - (d1^2 * d2^3)
        double k3 = (d1 * d1 * d1) * (p2 - c0) - (d2 * d2 * d2) * (p1 - c0);
        // d1^3 * (p2 - c0) - d2^3 * (p1 - c0)
        double k4 = (d3 * d1 * d1 * d1) - (d1 * d3 * d3 * d3); //(d3 * d1^3) - (d1 * d3^3)
        double k5 = (d3 * d3 * d1 * d1 * d1) - (d1 * d1 * d3 * d3 * d3); //(d3^2 * d1^3) - (d1^2 * d3^3)
        double k6 = (d1 * d1 * d1) * (p3 - c0) - (d3 * d3 * d3) * (p1 - c0);
        // d1^3 * (p3 - c0) - d3^3 * (p1 - c0)
        double c1 = ((k2 * k6) - (k3 * k5)) / ((k2 * k4) - (k5 * k1));
        double c2 = ((k3 - (k1 * c1)) / k2);
        double c3 = (p2 - c0 - (c1 * d2) - (c2 * d2 * d2)) / (d2 * d2 * d2);


        //initial motor positions
        double initPosrr = rightMotorR.getCurrentPosition();
        double initPoslr = leftMotorR.getCurrentPosition();

        //slope for proportional power decrease0
        double slope = (maximumPower - MINIMUM_DRIVE_PWR) / (Math.abs(targetPos));

        double distanceSign = Math.signum(distance);

        //current motor positions

        double currentRobotPosition = 0;

        double startTime = runtime.milliseconds();
        double targetTime = startTime + stallTime;
        double curTime = runtime.milliseconds();

        secondPosRight = rightMotorR.getCurrentPosition();
        secondPosLeft = leftMotorR.getCurrentPosition();
        secondTime = runtime.milliseconds();
        while (Math.abs(currentRobotPosition) < Math.abs(targetPos) && curTime < targetTime
                && !isStop) {

            if(Math.abs(currentRobotPosition/targetPos) == postionToRun){

            }

            curTime = runtime.milliseconds();

            //current motor positions (constantly updating)
            double curPosrr = rightMotorR.getCurrentPosition() - initPosrr;
            double curPoslr = leftMotorR.getCurrentPosition() - initPoslr;
            currentRobotPosition = (curPoslr + curPosrr) / 2;

            double distanceTraveled = Math.abs(currentRobotPosition);

            //power profile calc
            double power = c0 + c1 * (distanceTraveled) + c2 *
                    (distanceTraveled * distanceTraveled) +
                    c3 * (distanceTraveled * distanceTraveled * distanceTraveled);

            if (power > 1.0)
                power = 1.0;

            if (power >= maximumPower)
                power = maximumPower;

            if (Math.abs(distance) < 9)
                power = 0.25;

            firstPosRight = secondPosRight;
            firstPosLeft = secondPosLeft;

            firstTime = secondTime;

            secondPosRight = rightMotorR.getCurrentPosition();
            secondPosLeft = leftMotorR.getCurrentPosition();

            secondTime = runtime.milliseconds();

            /* Right Front Motor Calculation */
            currentRightSpeed = (secondPosRight - firstPosRight) / (secondTime - firstTime);
            currentRightSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedRight = Math.abs(power * POWER_SPEED_CONVERSION);

            rightSpeedError = Math.abs(targetSpeedRight) - Math.abs(currentRightSpeed);
            rightPower = Math.abs(power) + rightSpeedError / CORRECTION_FACTOR;

            /* Left Motor Calculation */
            currentLeftSpeed = (secondPosLeft - firstPosLeft) / (secondTime - firstTime);
            currentLeftSpeed *= 60 * 1000 / COUNTS_PER_MOTOR_REV;

            targetSpeedLeft = Math.abs(power * POWER_SPEED_CONVERSION);

            leftSpeedError = Math.abs(targetSpeedLeft) - Math.abs(currentLeftSpeed);
            leftPower = Math.abs(power) + leftSpeedError / CORRECTION_FACTOR;

            double currentHeading = RobotHardware.getInstance().gyro.getAngle();
//if error is negative, then robot is on the right, so you must turn left.
            //same for positive error
            /** @TODO change second heading in compute gyro drive correction to current heading*/
            double gyroCorrectionPwr = GYRO_K_FACTOR_FORWARD
                    * computeGyroDriveCorrectionError(heading, currentHeading);

            leftMotor.setPower(((leftPower + (gyroCorrectionPwr * distanceSign))
                    * distanceSign));
            leftMotorR.setPower(((leftPower + (gyroCorrectionPwr * distanceSign))
                    * distanceSign));
            rightMotor.setPower((rightPower - (gyroCorrectionPwr * distanceSign))
                    * distanceSign);
            rightMotorR.setPower((rightPower - (gyroCorrectionPwr * distanceSign))
                    * distanceSign);

            if (detectStall) {
                if (curTime - startTime > 500) {
                    if (Math.abs(currentLeftSpeed) < 0.1 || Math.abs(currentRightSpeed) < 0.1)
                        break;
                }
            }

//            telemetry.addData("Target Pos: ", targetPos);
//            telemetry.addData("Current Robot Position: ", currentRobotPosition);
//            //telemetry.addData( "Rf " , curPosrf );
//            telemetry.addData("Rr", curPosrr);
//            //telemetry.addData("Lf " , curPoslf );
//            telemetry.addData("Lr", curPoslr);
//            telemetry.addData("Current Power", power);
//            telemetry.addData("Slope", slope);
//            telemetry.addData("heading", heading);
//            telemetry.addData(">", "Current Right Speed = " + currentRightSpeed);
//            telemetry.addData(">", "Current Right Speed = " + currentLeftSpeed);
//            //telemetry.addData("heading Error", headingError);
//            telemetry.update();
//
//            idle();

        }

//        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        rightMotorR.setPower(0);
        leftMotorR.setPower(0);

    }




}
