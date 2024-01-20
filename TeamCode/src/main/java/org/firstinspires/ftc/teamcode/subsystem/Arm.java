package org.firstinspires.ftc.teamcode.subsystem;
// n words
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.justbetter.actuator.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.subsystem.util.GravityFeedforward;
import org.jetbrains.annotations.NotNull;

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
    }

    public void update() {

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

    // 550 ticks 4.04 ticks per degree
    public enum ArmPositions{
        FLAT,
        BACKBOARD,
        COMINGDOWN
    }


}
