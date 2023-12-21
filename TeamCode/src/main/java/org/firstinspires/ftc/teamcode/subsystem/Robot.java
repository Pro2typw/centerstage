package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.Constants.IMU.HEADING_PID_COEFFICIENTS;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;


/**
 * Must call {@code update()} for everything to work
 */
public class Robot {
    public Claw claw;
    public IMU imu;
    public MecanumDrive drive;
    
    private Orientation orientation;
    private PIDFController headingPID;
    private boolean isResetToIMU;
    
    public Robot(HardwareMap hardwareMap, Claw.ClawState clawState) {
        claw = new Claw(hardwareMap);
        claw.setClawState(Claw.ClawSide.BOTH, clawState);
        
        imu = new IMU(hardwareMap);
        headingPID = new PIDFController(HEADING_PID_COEFFICIENTS);
        headingPID.setOutputBounds(0, 1);
        isResetToIMU = false;
        
        drive = new MecanumDrive(hardwareMap);
    }
    
    public void resetToIMU() {
        isResetToIMU = true;
        headingPID.setTargetPosition(0);
        drive.turnWithPower(headingPID.update(imu.getCurrentAngularOrientation().firstAngle));
    }
    
    public void update() {
        if(isResetToIMU) {
            drive.turnWithPower(headingPID.update(imu.getCurrentAngularOrientation().firstAngle));
        }
    }
}
