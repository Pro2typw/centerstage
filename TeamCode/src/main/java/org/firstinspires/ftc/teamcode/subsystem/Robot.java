package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.util.Constants.IMU.HEADING_PID_COEFFICIENTS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class Robot {
    public Claw claw;
    public MecanumDrive drive;
    public Hang hang;
    public List<LynxModule> lynxModules;
    private Telemetry telemetry;

    private IMU imu;
    private Orientation currentOrientation;
    private PIDFController headingPID;
    private boolean isResetToIMU;

    public Robot(@NotNull HardwareMap hardwareMap, @NotNull Telemetry telemetry, @NotNull Claw.ClawState clawState, @NotNull Hang.HangState hangState, boolean dashboard) {
        claw = new Claw(hardwareMap, clawState);
        drive = new MecanumDrive(hardwareMap);
        hang = new Hang(hardwareMap, hangState);

        lynxModules = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if(dashboard) this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        else this.telemetry = telemetry;

        imu = new IMU(hardwareMap);
        headingPID = new PIDFController(HEADING_PID_COEFFICIENTS);
        headingPID.setOutputBounds(0, 1);
        headingPID.setTargetPosition(0);
        isResetToIMU = false;
    }

    public void toggleResetToIMU() {
        isResetToIMU = !isResetToIMU;
    }

    public void clearCache() {
        for (LynxModule module : lynxModules) {
            module.clearBulkCache();
        }
    }

    public void update() {
        if(isResetToIMU) {
            currentOrientation = imu.getCurrentAngularOrientation();
            double driveTurnPower = headingPID.update(currentOrientation.firstAngle); // todo config me

            drive.turnWithPower(driveTurnPower);
        }
    }

    public void getTelemetry() {
        telemetry.addData("Left Claw", claw.getClawState(Claw.ClawSide.LEFT));
        telemetry.addData("Right Claw", claw.getClawState(Claw.ClawSide.RIGHT));
        telemetry.addData("Hang", hang.getState());
        telemetry.addData("Heading", currentOrientation.firstAngle);

        //...
    }

    public void updateTelemetry() {
        telemetry.update();
    }
}
