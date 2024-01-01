package org.firstinspires.ftc.teamcode.subsystem.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.LoopRateTracker;

import java.util.List;


@Disabled
@Config
@TeleOp(group = "utility")
public class MaxAcceleration extends LinearOpMode {

    String MOTOR_NAME = "";
    String ENCODER_NAME = ""; // leave blank if using drive encoders


    boolean USING_ENCODER = ENCODER_NAME.length() == 0;

    int min, max;

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : modules) module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        Encoder encoder = null;
        if(USING_ENCODER) encoder = hardwareMap.get(Encoder.class, ENCODER_NAME);

        min = max = USING_ENCODER ? encoder.getPosition() : motor.getCurrentPosition();

        LoopRateTracker loopRateTracker = new LoopRateTracker();

        while(opModeInInit()) {
            for(LynxModule module : modules) module.clearBulkCache();

            int curr = USING_ENCODER ? encoder.getPosition() : motor.getCurrentPosition();

            if(curr < min) min = curr;
            else if(curr > max) max = curr;

            telemetry.addLine("Press START after setting minimum and maximum possible positions.");
            telemetry.addData("Min Position", min);
            telemetry.addData("Max Position", max);
            telemetry.addData("Current Position", curr);
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {

            int cur = USING_ENCODER ? encoder.getPosition() : motor.getCurrentPosition();

            loopRateTracker.update();

            for(LynxModule module : modules) module.clearBulkCache();
        }
    }
}
