package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.Constants.IMU.HEADING_PID_COEFFICIENTS;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

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
        isResetToIMU = false;
    }

    public void setPowers(double x, double y, double rx, Function<Double, Double> func) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lf = func.apply((y + x + rx) / denominator);
        double lb = func.apply((y - x + rx) / denominator);
        double rb = func.apply((y + x - rx) / denominator);
        double rf = func.apply((y - x - rx) / denominator);

        drive.setMotorPowers(lf, lb, rb, rf);
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
