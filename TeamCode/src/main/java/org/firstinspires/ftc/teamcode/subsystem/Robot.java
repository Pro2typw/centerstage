package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.Constants.IMU.HEADING_PID_COEFFICIENTS;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.Timer;
import java.util.function.Function;


/**
 * Must call {@code update()} for everything to work
 */
public class Robot {
    public Claw claw;
    public MecanumDrive drive;
    public Hang hang;

    private IMU imu;


    private Orientation orientation;
    private PIDFController headingPID;
    private boolean isResetToIMU;

    public Robot(HardwareMap hardwareMap, Claw.ClawState clawState, Hang.HangState hangState) {
        claw = new Claw(hardwareMap, clawState);
        drive = new MecanumDrive(hardwareMap);
        hang = new Hang(hardwareMap, hangState);
        
        imu = new IMU(hardwareMap);
        headingPID = new PIDFController(HEADING_PID_COEFFICIENTS);
        headingPID.setOutputBounds(0, 1);
        headingPID.setTargetPosition(0);
        isResetToIMU = false;
    }

    public void toggleResetToIMU() {
        isResetToIMU = !isResetToIMU;
    }
    
    public void update() {
        if(isResetToIMU) {
            drive.turnWithPower(headingPID.update(imu.getCurrentAngularOrientation().firstAngle));
        }
    }
}
