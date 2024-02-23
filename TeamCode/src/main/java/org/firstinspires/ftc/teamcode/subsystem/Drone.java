package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {

    public Servo servo;
    private boolean launched;

    public Drone(@NonNull HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, Constants.Drone.DRONE_MAP_NAME);
        servo.setPosition(Constants.Drone.INIT_POS);
        launched = false;
    }
    public void launch() {
        if(launched) servo.setPosition(Constants.Drone.LAUNCH_POS);
        else servo.setPosition(Constants.Drone.INIT_POS);

        launched = !launched;
    }

    public boolean isLaunched() {
        return launched;
    }
}
