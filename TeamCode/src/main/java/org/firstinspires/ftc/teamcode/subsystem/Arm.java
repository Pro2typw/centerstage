package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;

public class Arm {

    public final DcMotorEx motor1;
    public final DcMotorEx motor2;


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

    private VoltageSensor batterVoltageSensor;

    private double pivotTargetPos = 0;
    private double extensionTargetPos = 0;

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



    private double differenceError = 0;
    private double averageError;
    private double lastdifferenceError = 0;
    private double lastaverageError = 0;
    private double totaldifferenceError = 0;
    private double totalaverageError = 0;

    private double batterComp = 0;

    private static double G = 0.6;

    private long startTime;
    private long lastTime;

    private double currentAveragePosition = 0;
    private double currentDifferencePosition = 0;

    public double getCurrentAveragePosition() {
        return currentAveragePosition;
    }

    public double getCurrentDifferencePosition() {
        return currentDifferencePosition;
    }

    public void init() {
        startTime = System.nanoTime();
        lastTime = startTime;
    }

    public void update() {
        currentDifferencePosition = (motor1.getCurrentPosition() - motor2.getCurrentPosition())/2.0;

        differenceError = pivotTargetPos - currentDifferencePosition;
        if (differenceError * lastdifferenceError <= 0) totaldifferenceError = 0;
        else totaldifferenceError += differenceError;
        double differenceI = (totaldifferenceError * (System.nanoTime() - startTime)) * Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kI;
        double differenceD = ((differenceError - lastdifferenceError) / (System.nanoTime() - lastTime)) * Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kD;
        double differenceP = differenceError * Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kP;

        double differencePower = differenceP + differenceI + differenceD;

        currentAveragePosition = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0;

        averageError = extensionTargetPos - currentAveragePosition;
        if (averageError * lastaverageError <= 0) totalaverageError = 0;
        else totalaverageError += averageError;
        double averageI = (totalaverageError * (System.nanoTime() - startTime)) * Constants.Arm.AVERAGE_PID_COEFFICIENTS.kI;
        double averageD = ((averageError - lastaverageError) / (System.nanoTime() - lastTime)) * Constants.Arm.AVERAGE_PID_COEFFICIENTS.kD;
        double averageP = averageError * Constants.Arm.AVERAGE_PID_COEFFICIENTS.kP;

        double averagePower = averageP + averageI + averageD;
        double gravityPower = Math.cos(Math.toRadians(Arm.ticksToDegrees(currentDifferencePosition))) * .15 * batterComp * G;
        double power1 = differencePower + averagePower + gravityPower;
        double power2 = -differencePower + averagePower - gravityPower;

        motor1.setPower(power1);
        motor2.setPower(power2);


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
