package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.util.Constants.IMU.HEADING_PID_COEFFICIENTS;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.subsystem.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.pipeline.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
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
    public Camera camera;
    public Drone drone;
    public AprilTagDetectionPipeline apriltagDetectionPipeline;
    public PropDetectionPipeline propDetectionPipeline;
    public List<LynxModule> lynxModules;
    public Telemetry telemetry;

    private IMU imu;
    private Orientation currentOrientation;
    private PIDFController headingPID;
    private boolean isResetToIMU;

    public Robot(@NotNull HardwareMap hardwareMap, @NotNull Telemetry telemetry, @NotNull Claw.ClawState clawState) {
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap, clawState);
        drive = new MecanumDrive(hardwareMap);
        hang = new Hang(hardwareMap);
        wrist = new Wrist(hardwareMap);
        drone = new Drone(hardwareMap);

        arm.setPivotTargetPos(0);
        arm.setExtensionTargetPos(0);


        lynxModules = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.telemetry = telemetry;

//        imu = new IMU(hardwareMap);
        headingPID = new PIDFController(HEADING_PID_COEFFICIENTS);
        headingPID.setOutputBounds(0, 1);
        headingPID.setTargetPosition(0);
        isResetToIMU = false;
        armState = ArmState.INIT;
    }

    public Robot(@NotNull HardwareMap hardwareMap, @NotNull Telemetry telemetry) {
        this(hardwareMap, telemetry, Claw.ClawState.CLOSE);
    }

    public Robot(@NotNull HardwareMap hardwareMap, @NotNull Telemetry telemetry, @NotNull Claw.ClawState clawState, AllianceColor color) {
        this(hardwareMap, telemetry, clawState);
        apriltagDetectionPipeline = new AprilTagDetectionPipeline();
        propDetectionPipeline = new PropDetectionPipeline(color);
        camera = new Camera(hardwareMap, apriltagDetectionPipeline.getAprilTagProcessor(), propDetectionPipeline);
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
        if(isResetToIMU) {
            currentOrientation = imu.getCurrentAngularOrientation();
            double driveTurnPower = headingPID.update(currentOrientation.firstAngle);

            drive.turnWithPower(driveTurnPower);
        }

        drive.update();
        arm.update();
    }


    public enum ArmState {
        INIT,
        INTAKE,
        TRANSITION,
        DEPOSIT

    }

    ArmState armState;
    public void setArmState(ArmState state) {

    }

    public ArmState getArmState() {
        return armState;
    }

}