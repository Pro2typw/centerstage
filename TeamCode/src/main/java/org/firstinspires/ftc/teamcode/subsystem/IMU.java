package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IMU {
    
    private final BNO055IMU imu;
    private Orientation zeroHeading;
    
    public IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, Constants.IMU.IMU_MAP_NAME);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        
        zeroHeading = imu.getAngularOrientation();
    }
    
    public void resetAngularOrientation() {
        zeroHeading = imu.getAngularOrientation();
    }
    
    public Orientation getCurrentAngularOrientation() {
        Orientation currOrientation = imu.getAngularOrientation();
        return new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES,
                currOrientation.firstAngle - zeroHeading.firstAngle,
                currOrientation.secondAngle - zeroHeading.secondAngle,
                currOrientation.thirdAngle - zeroHeading.thirdAngle,
                currOrientation.acquisitionTime);
    }
    
}
