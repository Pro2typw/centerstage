package org.firstinspires.ftc.teamcode.subsystem.util;


public class    GravityFeedforward {

    double gain;

    /**
     *
     * @param gain proportional gain used to scale the gravity feedforward signal
     */
    public GravityFeedforward(double gain) {
        this.gain = gain;
    }

    /**
     *
     * @param length length of the lever or arm
     * @param angRad angle from the horizontal (in radians) at which the lever or arm is positioned
     * @return gravity-based feedforward signal to compensate for torque
     */
    public double calculate(double length, double angRad) {
        return length * 9.8 * Math.sin(angRad) * gain;
    }
}
