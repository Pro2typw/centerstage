package org.firstinspires.ftc.teamcode.bettersensors.Sensors;


import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.bettersensors.HardwareMapProvider;

public class MotorEncoder extends com.wolfpackmachina.bettersensors.Sensor<Double> {

    String deviceName;
    MotorEx encoder;

    public MotorEncoder(String hardwareID){
        super(hardwareID, 0);
    }

    public MotorEncoder(String hardwareID, int pingFrequency){
        super(hardwareID, pingFrequency);
    }

    @Override
    protected void sensorInit(String hardwareID) {
        deviceName = HardwareMapProvider.hardwareMap.get(DcMotor.class, hardwareID).getDeviceName() + "-encoder";
        encoder = new MotorEx(HardwareMapProvider.hardwareMap, hardwareID);
        encoder.resetEncoder();
    }

    @Override
    protected Double pingSensor() {
        return encoder.getDistance();
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    /**
     * Sets distance per encoder tick
     * @param distancePerPulse
     */
    protected void setDistancePerPulse(double distancePerPulse){
        encoder.setDistancePerPulse(distancePerPulse);
    }

    public double getDistance(){
        return readingCache;
    }

}
