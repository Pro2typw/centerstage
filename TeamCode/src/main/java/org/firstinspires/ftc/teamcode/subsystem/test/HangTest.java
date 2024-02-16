package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

@TeleOp(group = "test")
public class HangTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hang hang = new Hang(hardwareMap);
        Hang.HangState state = Hang.HangState.INIT;
        JustPressed gp = new JustPressed(gamepad1);
        MultipleTelemetry telemetry1 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean manual = false;

        waitForStart();

        while (opModeIsActive()) {

            if(gp.right_bumper()) {
                manual = !manual;
                if(manual) hang.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(manual) {
                hang.setPower(gp.left_stick_y() * 1);
            }
            else {
                if(gp.a()) hang.setState(Hang.HangState.INIT);
                if(gp.b()) hang.setState(Hang.HangState.UP);
                if(gp.x()) hang.setState(Hang.HangState.DOWN);

            }

            telemetry1.addData("Manual", manual);
            telemetry1.addData("Position", hang.getCurrentPosition());
            telemetry1.update();
            gp.update();

        }
    }
}
