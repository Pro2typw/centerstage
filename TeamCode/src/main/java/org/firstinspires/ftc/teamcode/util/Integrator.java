package org.firstinspires.ftc.teamcode.util;

/**
 * Integral approximation using trapezoidal integration.
 */
public class Integrator {
    private double value;
    private double lastRate;

    public Integrator() {
        this(0);
    }
    public Integrator(double initialValue) {
        reset(value);
    }

    public void reset(double value) {
        this.value = value;
        this.lastRate = 0;
    }

    public double get() {
        return value;
    }

    public void update(double rate, double dt) {
        value += (rate + lastRate) * dt / 2;
        lastRate = rate;
    }
}
