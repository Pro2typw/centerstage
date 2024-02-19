package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.BlueBackdropPropDetection;
import org.firstinspires.ftc.teamcode.vision.pipeline.RedBackdropPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(group = "test")
public class PropDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BlueBackdropPropDetection processor = new BlueBackdropPropDetection();

        VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), processor);

        while (opModeInInit()) {
            telemetry.addData("MODE", portal.getCameraState());
        }
        waitForStart();

        telemetry.addLine("DUMASS, DON'T START THE OPMODE...");
        telemetry.addLine("RESTART OPMODE");
        telemetry.update();
    }
}
