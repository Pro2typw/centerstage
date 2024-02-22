package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MultiMotor;
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
//        leftHang.setDirection(DcMotorSimple.Direction.REVERSE);

        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public Hang(@NotNull HardwareMap hardwareMap) {
        this(hardwareMap, HangState.INIT);
    }

    public void setRunMode(DcMotor.RunMode mode){
        hang.setMode(mode);
    }

    public void setPower(double pow) {
        hang.setPower(pow);
    }

    public double[] getPowers() {
        return hang.getPowers();
    }

}
