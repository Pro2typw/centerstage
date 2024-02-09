package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.subsystem.util.PIDController;
import org.firstinspires.ftc.teamcode.util.WPIMathUtil;


public class Arm {

    public final DcMotorEx motor1;
    public final DcMotorEx motor2;

    VoltageSensor batterVoltageSensor;
    private double batterComp;

    private double pivotTargetPos = 0;
    private double extensionTargetPos = 0;
    private double pivotCurrentPos = 0;
    private double extensionCurrentPos = 0;


    PIDController pivotController, extensionController;


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

        if(Constants.Arm.VOLTAGE_COMPENSATION_CONSTANT == 0) batterComp = 1;
        else batterComp = (Constants.Arm.VOLTAGE_COMPENSATION_CONSTANT / batterVoltageSensor.getVoltage());

        pivotController = new PIDController(Constants.Arm.DIFFERENCE_PID_COEFFICIENTS);
        extensionController = new PIDController(Constants.Arm.AVERAGE_PID_COEFFICIENTS);

    }

    public void init() {
        pivotController.init();
        extensionController.init();
    }

    public void update() {
        pivotCurrentPos = (motor1.getCurrentPosition() - motor2.getCurrentPosition()) / 2.0;
        extensionCurrentPos = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0;

        double differencePower = pivotController.calculate(pivotCurrentPos);
        double averagePower = extensionController.calculate(extensionCurrentPos);
        double gravityPower = Math.cos(Math.toRadians(Arm.ticksToDegrees(pivotCurrentPos))) * .15 * batterComp;

        double power1 = differencePower + averagePower + gravityPower;
        double power2 = -differencePower + averagePower - gravityPower;

        motor1.setPower(power1);
        motor2.setPower(power2);
    }

    public void newUpdate() {
        pivotCurrentPos = (motor1.getCurrentPosition() - motor2.getCurrentPosition()) / 2.0;
        extensionCurrentPos = (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0;

        double differencePower = pivotController.calculate(pivotCurrentPos);
        double averagePower = extensionController.calculate(extensionCurrentPos);
        double gravityPower = Math.cos(Math.toRadians(Arm.ticksToDegrees(pivotCurrentPos))) * .15 * batterComp;

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

    public double getPivotCurrentPos() {
        return pivotCurrentPos;
    }

    public double getExtensionCurrentPos() {
        return extensionCurrentPos;
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
