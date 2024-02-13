package org.firstinspires.ftc.teamcode.subsystem.vision.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.Camera;
import org.firstinspires.ftc.teamcode.subsystem.vision.pipeline.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(group = "test")
public class PropDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        PropDetectionPipeline propDetectionPipeline = new PropDetectionPipeline(AllianceColor.RED);
        Camera camera = new Camera(hardwareMap, propDetectionPipeline);
        JustPressed gp = new JustPressed(gamepad1);

        while (camera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine(camera.getCameraState().toString());
            telemetry.addData("Prop Position", propDetectionPipeline.getPropPosition());
            double[] averages = propDetectionPipeline.getAveragedBoxes();
            telemetry.addLine(averages[0] + " " + averages[1] + " " + averages[2]);
            telemetry.update();
        }
        while (opModeInInit()) {
            camera.setProcessorEnabled(propDetectionPipeline, true);

//            double[] percents = propDetectionPipeline.getBoxAreas();
//            telemetry.addData("Location", propDetectionPipeline.getPropPosition());
//            telemetry.addData("Left Square %", percents[0]);
//            telemetry.addData("Center Square %", percents[1]);
//            telemetry.addData("Right Square %", percents[2]);

            telemetry.update();
            gp.update();
        }

        waitForStart();
            telemetry.addLine("Restart program and don't go past the init mode");
            telemetry.update();

    }
}
