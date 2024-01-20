package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.util.Constants;

public class Launch {

    public Servo servo;

    public Launch(@NonNull HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, Constants.Plane.DRONE_MAP_NAME);
        servo.setPosition(Constants.Plane.INIT_POS);
    }
    public void launch() {
        servo.setPosition(Constants.Plane.LAUNCH_POS);
    }
}