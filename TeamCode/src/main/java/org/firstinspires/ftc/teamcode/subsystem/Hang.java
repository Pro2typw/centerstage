package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.util.MultiMotor;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.jetbrains.annotations.NotNull;

public class Hang {

    public static enum HangState {
        UP,
        DOWN
    }

    DcMotorEx hang1, hang2;
    MultiMotor hang;
    HangState state;
    public Hang(@NotNull HardwareMap hardwareMap, @NotNull HangState state) {
        hang1 = hardwareMap.get(DcMotorEx.class, Constants.Hang.HANG1_MAP_NAME);
        hang2 = hardwareMap.get(DcMotorEx.class, Constants.Hang.HANG2_MAP_NAME);

        hang = new MultiMotor(hang1, hang2);

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setState(state);

    }

    public void setState(@NotNull HangState state) {
        switch (state) {
            case UP:
                this.state = state;
                hang.setTargetPosition(Constants.Hang.UP_POSITION);
                break;
            case DOWN:
                this.state = state;
                hang.setTargetPosition(Constants.Hang.DOWN_POSITION);
                break;
        }
    }

    public HangState getState() {
        return state;
    }
}
