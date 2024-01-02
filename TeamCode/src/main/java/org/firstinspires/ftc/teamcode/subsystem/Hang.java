package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.justbetter.actuator.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.subsystem.util.CachingMultiMotor;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.jetbrains.annotations.NotNull;

public class Hang {
    public static enum HangState {
        UP,
        DOWN
    }

    private final CachingMultiMotor hang;
    private HangState state;

    public Hang(@NotNull HardwareMap hardwareMap, @NotNull HangState state) {
        CachingDcMotorEX leftHang = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Constants.Hang.LEFT_HANG_MAP_NAME));
        CachingDcMotorEX rightHang = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Constants.Hang.RIGHT_HANG_MAP_NAME));
        
        hang = new CachingMultiMotor(leftHang, rightHang);

        leftHang.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, Constants.Hang.LEFT_HANG_PID_COEFFICIENTS);
        rightHang.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, Constants.Hang.RIGHT_HANG_PID_COEFFICIENTS);
        
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
