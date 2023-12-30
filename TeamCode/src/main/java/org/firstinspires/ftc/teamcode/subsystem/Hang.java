package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.subsystem.util.MultiMotor;
import org.jetbrains.annotations.NotNull;

public class Hang {
    public static enum HangState {
        UP,
        DOWN
    }

    private final MultiMotor hang;
    private HangState state;

    public Hang(@NotNull HardwareMap hardwareMap, @NotNull HangState state) {
        DcMotorEx leftHang = hardwareMap.get(DcMotorEx.class, Constants.Hang.LEFT_HANG_MAP_NAME);
        DcMotorEx rightHang = hardwareMap.get(DcMotorEx.class, Constants.Hang.RIGHT_HANG_MAP_NAME);

        leftHang.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, Constants.Hang.LEFT_HANG_PID_COEFFICIENTS);
        rightHang.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, Constants.Hang.RIGHT_HANG_PID_COEFFICIENTS);


        hang = new MultiMotor(leftHang, rightHang);

        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setState(state);
    }

    public void setState(@NotNull HangState state) {
        switch (state) {
            case UP:
                this.state = state;
                hang.setTargetPosition(Constants.Hang.UP_POSITION);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case DOWN:
                this.state = state;
                hang.setTargetPosition(Constants.Hang.DOWN_POSITION);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
    }

    public HangState getState() {
        return state;
    }
}
