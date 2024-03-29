package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.justbetter.actuator.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.subsystem.util.CachingMultiMotor;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.subsystem.util.MultiMotor;
import org.jetbrains.annotations.NotNull;

public class Hang {
    public static enum HangState {
        INIT,
        UP,
        DOWN
    }

    private final MultiMotor hang;
    private HangState state;

    public Hang(@NotNull HardwareMap hardwareMap, @NotNull HangState state) {
        DcMotorEx leftHang = hardwareMap.get(DcMotorEx.class, Constants.Hang.LEFT_HANG_MAP_NAME);
        DcMotorEx rightHang = hardwareMap.get(DcMotorEx.class, Constants.Hang.RIGHT_HANG_MAP_NAME);
        
        hang = new MultiMotor(leftHang, rightHang);
        leftHang.setDirection(DcMotorSimple.Direction.REVERSE);

        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public Hang(@NotNull HardwareMap hardwareMap) {
        this(hardwareMap, HangState.INIT);
    }

    public void setState(@NotNull HangState state) {
        this.state = state;
        switch (state) {

            case INIT:
                setTargetPosition(Constants.Hang.INIT_POSITION, 1);
                break;
            case UP:
                setTargetPosition(Constants.Hang.UP_POSITION, 1);
                break;
            case DOWN:
                setTargetPosition(Constants.Hang.DOWN_POSITION, .5);
        }
    }

    public void setTargetPosition(int pos, double pow) {
        hang.setTargetPosition(pos);
        hang.setPower(pow);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getTargetPosition() {
        return hang.getTargetPosition();
    }

    public HangState getState() {
        return state;
    }

    public int getCurrentPosition() {
        return hang.getCurrentPosition();
    }

    public void cycleNextHangState() {
        switch (state) {
            case DOWN:
            case INIT:
                setState(HangState.UP);
                break;
            case UP:
                setState(HangState.DOWN);
                break;

        }
    }

    public void setRunMode(DcMotor.RunMode mode){
        hang.setMode(mode);
    }

    public void setPower(double pow) {
        hang.setPower(pow);
    }

}
