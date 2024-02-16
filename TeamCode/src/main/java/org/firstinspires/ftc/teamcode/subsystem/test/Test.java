package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystem.util.Constants;

@Config()
@TeleOp(group = "test", name = "idk")
public class Test extends LinearOpMode {

    public static double LEFT_POWER = 0;
    public static double RIGHT_POWER = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR1_MAP_NAME);
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR2_MAP_NAME);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        for(DcMotorEx motor : new DcMotorEx[]{motor1, motor2}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }



        waitForStart();
        while (opModeIsActive()) {
            motor1.setPower(LEFT_POWER);
            motor2.setPower(RIGHT_POWER);
            telemetry.addData("left", motor1.getCurrentPosition());
            telemetry.addData("right", motor2.getCurrentPosition());
            telemetry.update();
        }

    }
}
