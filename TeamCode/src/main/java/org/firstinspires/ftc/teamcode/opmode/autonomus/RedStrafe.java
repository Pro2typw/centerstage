package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

import java.util.concurrent.TimeUnit;

@Autonomous(group = "game")
public class RedStrafe extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            if(timer.time(TimeUnit.SECONDS) <= 2) {
                drive.setMotorPowers(1, -1, 1, -1);
            }
            else {
                drive.setMotorPowers(0, 0, 0, 0);
            }
        }
    }
}
