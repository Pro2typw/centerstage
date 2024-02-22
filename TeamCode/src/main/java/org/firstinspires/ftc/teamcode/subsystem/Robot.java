package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class Robot {

    public enum PixelState {
        INTAKE,
        TRANSIT,
        DEPOSIT,
        ADJUST_DEPOSIT
    }

    public Arm arm;
    public Claw claw;
    public MecanumDrive drive;
    public Hang hang;
    public Wrist wrist;
    public Drone drone;
    public VisionProcessor propDetectionPipeline;
    public List<LynxModule> lynxModules;
    public Telemetry telemetry;


    public Robot(@NotNull HardwareMap hardwareMap, @NotNull Telemetry telemetry, @NotNull Claw.ClawState clawState) {
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap, clawState);
        drive = new MecanumDrive(hardwareMap);
        drone = new Drone(hardwareMap);
        hang = new Hang(hardwareMap);
        wrist = new Wrist(hardwareMap);

        arm.setPivotTargetPos(0);
        arm.setExtensionTargetPos(0);


        lynxModules = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.telemetry = telemetry;

    }
    public Robot(@NotNull HardwareMap hardwareMap, @NotNull Telemetry telemetry) {
        this(hardwareMap, telemetry, Claw.ClawState.CLOSE);
    }

    /**
     * call this first in the loop
     */
    public void clearCache() {
        for (LynxModule module : lynxModules) {
            module.clearBulkCache();
        }
    }

    /**
     * call this right before the while(opmodeisactive) loop
     */
    public void init() {
        arm.init();
    }

    /**
     * call this in the while(opmodeisaction) loop
     */
    public void update() {
        drive.updatePoseEstimate();
        arm.update();
    }

}
