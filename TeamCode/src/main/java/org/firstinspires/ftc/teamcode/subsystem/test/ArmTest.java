package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Arm;

@Config
@TeleOp(group = "test")
public class ArmTest extends LinearOpMode {
    public static int targetPivotPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Arm arm = new Arm(hardwareMap);


        waitForStart();

        arm.init();
        while (opModeIsActive()) {
            arm.setPivotTargetPos(targetPivotPos);

            arm.update();
        }
    }
}
