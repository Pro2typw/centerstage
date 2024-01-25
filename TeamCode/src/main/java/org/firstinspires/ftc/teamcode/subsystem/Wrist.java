package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.justbetter.actuator.CachingServo;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.jetbrains.annotations.NotNull;

public class Wrist {

    private final CachingServo leftServo, rightServo;
    private CachingServo[] servos;
    private double leftPosition, rightPosition;

    public Wrist(HardwareMap hardwareMap) {
        this(hardwareMap, 0); // change starting position to teleop starting position
    }

    public Wrist(HardwareMap hardwareMap, double position) {
        leftServo = new CachingServo(hardwareMap.get(Servo.class, Constants.Wrist.LEFT_WRIST_MAP_NAME));
        rightServo = new CachingServo(hardwareMap.get(Servo.class, Constants.Wrist.RIGHT_WRIST_MAP_NAME));

        servos = new CachingServo[] {leftServo, rightServo};

        leftPosition = position;
        rightPosition = position;
    }


    public void setPosition(double position) {
        leftPosition = position;
        rightPosition = position;

        for(CachingServo servo : servos) {
            servo.setPosition(position);
        }
    }

    public static double angleToPosition(double deg) {
        return deg / 180;
    }

    public static double positionToAngle(double pos) {
        return pos * 180;
    }


}