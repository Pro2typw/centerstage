package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.Arm.G_STATIC;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.WPIMathUtil;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.justbetter.sensor.Encoder;

public class Arm {

    public final DcMotorEx motor1;
    public final DcMotorEx motor2;
//    public final Encoder extensionEncoder;

//    public double currentExtensionEncoderPosition;

    public Arm(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR1_MAP_NAME);
        motor2 = hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR2_MAP_NAME);

//        extensionEncoder = hardwareMap.get(Encoder.class, Constants.Arm.EXTENSION_ENCODER_MAP_NAME);
//        extensionEncoder.reset();
//        currentExtensionEncoderPosition = extensionEncoder.getCurrentPosition();

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
        this.pivotTargetPos = WPIMathUtil.clamp(pivotTargetPos, 0, Constants.Arm.MAX_PIVOT);
    }

    public double getExtensionTargetPos() {
        return extensionTargetPos;
    }

    public void setExtensionTargetPos(double extensionTargetPos) {
        this.extensionTargetPos = WPIMathUtil.clamp(extensionTargetPos, 0, Constants.Arm.MAX_EXTENSION);
    }

    long startTime;
    long lastTime;
    public void init() {
        startTime = System.nanoTime();
        lastTime = startTime;
    }

    public static double G_PIVOT = 2.05652575;
    public double getExtensionCurrentPos() {
        return average;
    }

    public double getPivotCurrentPos() {
        return difference;
    }

//    public double getCurrentExtensionEncoderPosition() {
//        return currentExtensionEncoderPosition;
//    }

    public VoltageSensor batterVoltageSensor;


    public static double pivotTargetPos = 0;
    public static double extensionTargetPos = 0;

    public double differenceError = 0;
    public double averageError;
    public double lastdifferenceError = 0;
    public double lastaverageError = 0;
    public double totaldifferenceError = 0;
    public double totalaverageError = 0;

    public double batterComp = 0;

//    public static double G = 1.4;
    public static PIDCoefficients AVERAGE_PID_COEFFICIENTS = new PIDCoefficients(0.01, 2.5e-11, 10000);
    public static PIDCoefficients DIFFERENCE_PID_COEFFICIENTS = new PIDCoefficients(0.0036, 2.5e-12, 170000);
    public static double G_PIVOT2 = .23;
    public static double G_EXTENSION = .01;
    public static double G_STATIC = 0.000007;

    double difference = 0;
    double average = 0;
    public void update() {

        long time = System.nanoTime() - startTime;
//            wrist.setPosition(Constants.Wrist.INTAKE_POS);
        difference = (motor1.getCurrentPosition() - motor2.getCurrentPosition())/2.0;

        differenceError = pivotTargetPos - difference;
        if (differenceError * lastdifferenceError <= 0 || Math.abs(differenceError - lastdifferenceError) > 5) totaldifferenceError = 0;
        else totaldifferenceError += differenceError;

        double differenceI = (totaldifferenceError * (time - lastTime)) * DIFFERENCE_PID_COEFFICIENTS.kI;
        double differenceD = ((differenceError - lastdifferenceError) / (time - lastTime)) * DIFFERENCE_PID_COEFFICIENTS.kD;
        double differenceP = differenceError * DIFFERENCE_PID_COEFFICIENTS.kP;

        double differencePower = Math.abs(differenceError) < 0 ? 0 : differenceP + differenceI + differenceD;

        average = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0;

        averageError = extensionTargetPos - average;
        if (Math.abs(averageError) < 10)
            averageError = 0;
        if (averageError * lastaverageError <= 0 || Math.abs(averageError - lastaverageError) > 5) totalaverageError = 0;
        else totalaverageError += averageError;
        double averageI = (totalaverageError * (time - lastTime)) * AVERAGE_PID_COEFFICIENTS.kI;
        double averageD = ((averageError - lastaverageError) / (time - lastTime)) * AVERAGE_PID_COEFFICIENTS.kD;
        double averageP = averageError * AVERAGE_PID_COEFFICIENTS.kP;

        G_PIVOT = 1 + (G_STATIC * Math.pow(average, 2));

        double averagePower = averageP + averageI + averageD;
        averagePower += G_EXTENSION * Math.sin(Math.toRadians(Arm.ticksToDegrees(difference)));

        double gravityPower = difference > 0 ? Math.cos(Math.toRadians(Arm.ticksToDegrees(difference))) * G_PIVOT2 * batterComp * G_PIVOT: 0;

        double power1, power2;
//        if (Math.abs(differencePower) < .2) {
            power1 = differencePower + averagePower + gravityPower;
            power2 = -differencePower + averagePower- gravityPower;
//        } else {
//            power1 = differencePower + gravityPower;
//            power2 = -differencePower - gravityPower;
//        }

        motor1.setPower(power1);
        motor2.setPower(power2);

        lastTime = time;
        lastdifferenceError = differenceError;
        lastaverageError = averageError;

    }

    public static double backdropYtoX(double y) {
        return y / Math.sqrt(3);
    }

    public static double ticksToMillimeters(double ticks) {
        return ticks / 8.94468118871;
    } // todo its wrong

    public static int millimetersToTicks(double millimeters) { // todo its wrong
        return (int) (8.94468118871 * millimeters);
    }

    public static double ticksToDegrees(double ticks) {
        return ticks / 4.4807486631;
    }

    public static double degreesToTicks(double degrees) {
        return (int) (degrees * 4.4807486631);
    }

    public static double encoderTicksToMillimeters(double ticks) {
        return (Math.PI * 2.0 * (23.333333333 / 2.0) * ticks) / 8192.0; // .008948221 mm per tick
    }

}
