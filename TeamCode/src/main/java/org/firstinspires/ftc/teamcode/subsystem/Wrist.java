package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.justbetter.actuator.CachingServo;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.jetbrains.annotations.NotNull;

public class Wrist {
    private final CachingServo servo;
    private double position;

    public Wrist(@NotNull HardwareMap hardwareMap) {
        servo = new CachingServo(hardwareMap.get(Servo.class, Constants.Wrist.WRIST_MAP_NAME));
    }

    public double getPosition() {
        return position;
    }

    public void setPosition(double position) {
        this.position = position;
        servo.setPosition(position);
    }

    // todo conversions (maybe)
    public double angleToPos(double angRad) {
        return angRad;
    }
}