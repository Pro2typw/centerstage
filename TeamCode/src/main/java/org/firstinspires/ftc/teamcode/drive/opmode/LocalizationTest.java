package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.photoncore.Photon;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.concurrent.TimeUnit;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@Disabled
@TeleOp(group = "drive")
@Photon
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        LoopRateTracker loopRateTracker = new LoopRateTracker(TimeUnit.MILLISECONDS);
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        waitForStart();

        while (!isStopRequested()) {
            loopRateTracker.update();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -minPower(gamepad1.left_stick_y, .10),
                            -minPower(gamepad1.left_stick_x, .22),
                            -minPower(gamepad1.right_stick_x, .16)
                    )
            );
            // drive
//            drive.setPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, x -> .75 * Math.cbrt(x));


            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Loop Rate (ms)", loopRateTracker.getLoopTime(TimeUnit.MILLISECONDS));
            telemetry.addData("LEFT STICK X", gamepad1.left_stick_x);
            telemetry.addData("LEFT STICK Y", gamepad1.left_stick_y);
            telemetry.addData("RIGHT STICK X", gamepad1.right_stick_x);
            telemetry.update();

//            for (LynxModule lynxModule : hardwareMap.getAll(LynxModule.class)) {
//                lynxModule.clearBulkCache();
//            }
        }
    }

    public static double minPower(double input, double minValue) {
        return input == 0 ? 0 : Math.signum(input) * Range.scale(Math.abs(input), 0, 1, minValue, 1);
//        return input == 0 ? 0 : Math.signum(input) * Math.max(Math.abs(input), minValue);
    }
}
