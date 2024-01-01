package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

import org.jetbrains.annotations.NotNull;

public class Wrist {
    private final ServoEx servo;
    private double position;

    public Wrist(@NotNull HardwareMap hardwareMap, @NotNull double position) {
        servo = hardwareMap.get(ServoEx.class, "");

        this.position = position;
    }

    public double getPosition() {
        return position;
    }

    public void setPosition(double position) {
        this.position = position;
        servo.setPosition(position);
    }

    // todo conversions (maybe)
    public double angleToRadian(double angRad) {

        return angRad;
    }
}