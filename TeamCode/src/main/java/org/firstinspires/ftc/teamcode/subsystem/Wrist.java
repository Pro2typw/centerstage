package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.justbetter.actuator.CachingServo;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.jetbrains.annotations.NotNull;

public class Wrist {
    private final Servo servo;
    private double position;

    public enum WristState {
        REST_POS,
        INTAKE_POS,
        LOW_DEPO_POS,
        HIGH_DEPO_POS
    }

    public Wrist(@NotNull HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, Constants.Wrist.WRIST_MAP_NAME);
    }

    public double getPosition() {
        return position;
    }

    public void setState(WristState state) {
        switch (state) {
            case REST_POS:
                setPosition(Constants.Wrist.REST_POS);
                break;
            case INTAKE_POS:
                setPosition(Constants.Wrist.INTAKE_POS);
                break;
            case LOW_DEPO_POS:
                setPosition(Constants.Wrist.LOW_DEPO_POS);
                break;
            case HIGH_DEPO_POS:
                setPosition(Constants.Wrist.HIGH_DEPO_POS);
                break;

        }
    }

    public void setPosition(double position) {
        this.position = position;
        servo.setPosition(position);
    }
}