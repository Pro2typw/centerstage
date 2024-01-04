package org.firstinspires.ftc.teamcode.subsystem.vision.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.Camera;
import org.firstinspires.ftc.teamcode.subsystem.vision.pipeline.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.util.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(group = "test")
public class PropDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PropDetectionPipeline propDetectionPipeline = new PropDetectionPipeline(AllianceColor.BLUE);
        Camera camera = new Camera(hardwareMap, propDetectionPipeline);
        JustPressed gp = new JustPressed(gamepad1);

        while (camera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            tele.addLine(camera.getCameraState().toString());
            tele.update();
            sleep(20);
        }
        while (opModeInInit()) {
            camera.setProcessorEnabled(propDetectionPipeline, true);

            if(gp.a()) {
                propDetectionPipeline = new PropDetectionPipeline(AllianceColor.BLUE);
                camera = new Camera(hardwareMap, propDetectionPipeline);
                while (camera.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    tele.addLine(camera.getCameraState().toString());
                    tele.update();
                    sleep(20);
                }
                camera.setProcessorEnabled(propDetectionPipeline, true);
            }

            double[] boxpercents = propDetectionPipeline.getBoxAreas();
            tele.addData("Location", propDetectionPipeline.getPropPosition());
            tele.addData("Left Square %", boxpercents[0]);
            tele.addData("Center Square %", boxpercents[1]);
            tele.addData("Right Square %", boxpercents[2]);

            tele.update();
            gp.update();
        }

        waitForStart();
    }
}
