package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Extension {
    private DcMotorEx extension;
    private Encoder encoder;

    private int targetPosition;
    private MotionProfile motionProfile;
    private PIDFController controller;

    public Extension(HardwareMap hardwareMap, int targetPosition) {
        extension = hardwareMap.get(DcMotorEx.class, Constants.Extension.EXTENSION_MAP_NAME);
        encoder = hardwareMap.get(Encoder.class, Constants.Extension.ENCODER_MAP_NAME);

        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(targetPosition, 0, 0),
                1, 2, 3, true
                );

        this.targetPosition = targetPosition;
        controller = new PIDFController(Constants.Extension.PID_CCOEFFICIENTS);
        controller.setTargetPosition(targetPosition);
        controller.setOutputBounds(0, 1);
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        controller.setTargetPosition(targetPosition);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void movePosition(double ticks) {
        targetPosition += ticks;

        if(targetPosition < Constants.Extension.MIN_POSITION) targetPosition = Constants.Extension.MIN_POSITION;
        else if(targetPosition > Constants.Extension.MAX_POSITION) targetPosition = Constants.Extension.MAX_POSITION;

        controller.setTargetPosition(targetPosition);
    }

    public void update(ElapsedTime time) {
        extension.setPower(controller.update(encoder.getCurrentPosition()));
    }
}
