package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.util.Constants;

public class Arm {

    public final DcMotorEx motor1;
    public final DcMotorEx motor2;

    long startTime;
    long lastTime;


    public Arm(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR1_MAP_NAME);
        motor2 = hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR2_MAP_NAME);

//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        for(DcMotorEx motor : new DcMotorEx[]{motor1, motor2}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        batterVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        batterComp = (12.8 / batterVoltageSensor.getVoltage());

    }

    public double getPivotTargetPos() {
        return pivotTargetPos;
    }

    public void setPivotTargetPos(double pivotTargetPos) {
        this.pivotTargetPos = pivotTargetPos;
//        this.pivotTargetPos = pivotTargetPos > Constants.Arm.MAX ?
    }

    public double getExtensionTargetPos() {
        return extensionTargetPos;
    }

    public void setExtensionTargetPos(double extensionTargetPos) {
        this.extensionTargetPos = extensionTargetPos;
    }

    public void init() {
        startTime = System.nanoTime();
        lastTime = startTime;
    }

    public VoltageSensor batterVoltageSensor;


    public static double pivotTargetPos = 0;
    public static double extensionTargetPos = 0;

    public static PIDCoefficients averageCoef = new PIDCoefficients(0.008, 0, 0);
    public static PIDCoefficients differenceCoef = new PIDCoefficients(0.00013, 9e-16, 140000);



    public double differenceError = 0;
    public double averageError;
    public double lastdifferenceError = 0;
    public double lastaverageError = 0;
    public double totaldifferenceError = 0;
    public double totalaverageError = 0;

    public double batterComp = 0;

    public static double G = 1.4;
    public static double MOTOR_COEF = 1;

    double average = 0, difference = 0;

    public double getExtensionCurrentPos() {
        return average;
    }

    public double getPivotCurrentPos() {
        return difference;
    }


    public void update() {

        difference = (motor1.getCurrentPosition() - motor2.getCurrentPosition())/2.0;

        differenceError = pivotTargetPos - difference;
        if (differenceError * lastdifferenceError <= 0) totaldifferenceError = 0;
        else totaldifferenceError += differenceError;
        double differenceI = (totaldifferenceError * (System.nanoTime() - startTime)) * differenceCoef.kI;
        double differenceD = ((differenceError - lastdifferenceError) / (System.nanoTime() - lastTime)) * differenceCoef.kD;
        double differenceP = differenceError * differenceCoef.kP;

        double differencePower = differenceP + differenceI + differenceD;

        average = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0;

        averageError = extensionTargetPos - average;
        if (averageError * lastaverageError <= 0) totalaverageError = 0;
        else totalaverageError += averageError;
        double averageI = (totalaverageError * (System.nanoTime() - startTime)) * averageCoef.kI;
        double averageD = ((averageError - lastaverageError) / (System.nanoTime() - lastTime)) * averageCoef.kD;
        double averageP = averageError * averageCoef.kP;

        double averagePower = averageP + averageI + averageD;
        double gravityPower = Math.cos(Math.toRadians(Arm.ticksToDegrees(difference))) * .15 * batterComp * G;
        double power1 = differencePower + averagePower + gravityPower;
        double power2 = -differencePower + averagePower - gravityPower;

        motor1.setPower(power1 * MOTOR_COEF);
        motor2.setPower(power2 * MOTOR_COEF);

        lastTime = System.nanoTime();
        lastdifferenceError = differenceError;

    }

    public static double backdropYtoX(double y) {
        return y / Math.sqrt(3);
    }

    public static double ticksToMillimeters(double ticks) {
        return ticks / 8.94468118871;
    }

    public static int millimetersToTicks(double millimeters) {
        return (int) (8.94468118871 * millimeters);
    }

    public static double ticksToDegrees(double ticks) {
        return ticks / 4.4807486631;
    }

    public static double degreesToTicks(double degrees) {
        return (int) (degrees * 4.4807486631);
    }


}
