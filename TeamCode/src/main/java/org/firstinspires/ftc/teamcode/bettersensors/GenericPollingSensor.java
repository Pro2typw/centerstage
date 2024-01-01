package org.firstinspires.ftc.teamcode.bettersensors;


import org.firstinspires.ftc.teamcode.bettersensors.Utils.GenericDataSupplier;

public class GenericPollingSensor <T> extends com.wolfpackmachina.bettersensors.Sensor<T> {

    GenericDataSupplier<T> dataSupplier;

    public GenericPollingSensor(int pingFrequency, GenericDataSupplier<T> dataSupplier){
        super("", pingFrequency);
    }

    @Override
    protected void sensorInit(String hardwareID) {
    }

    @Override
    protected T pingSensor() {
        return dataSupplier.getData();
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "unknown";
    }
}
