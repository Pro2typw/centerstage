package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;

import java.util.concurrent.TimeUnit;

public class CenterGuessBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Claw claw = new Claw(hardwareMap, Claw.ClawState.CLOSE);
        wrist.setState(Wrist.WristState.REST_POS);
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            if(timer.time(TimeUnit.SECONDS) <=1) {
                wrist.setState(Wrist.WristState.INTAKE_POS);
                drive.setMotorPowers(1, 1, 1, 1);
            }
            else if(timer.time(TimeUnit.SECONDS) <= 2) {
                claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
            }
            else if(timer.time(TimeUnit.SECONDS) <= 3) {
                drive.setMotorPowers(-1, 1, -1, 1);
            }
            else {
                drive.setMotorPowers(0, 0, 0, 0);
            }
        }
    }
}
