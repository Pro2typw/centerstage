package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

public class BlueBackdrop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE, Hang.HangState.DOWN);



        do {
            robot.update();
        } while (opModeInInit());

        waitForStart();
        robot.init();

        while (opModeIsActive()) {
            robot.update();
        }
    }
}
