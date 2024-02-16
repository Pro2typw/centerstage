package org.firstinspires.ftc.teamcode.util;

import java.util.concurrent.TimeUnit;

public class LoopRateTracker {
    private TimeUnit resolution;
    private long lastTime = System.nanoTime();
    private long delta = 0;

    public LoopRateTracker() {
        this(TimeUnit.MILLISECONDS);
    }
    public LoopRateTracker(TimeUnit resolution) {
        this.resolution = resolution;
    }

    public void setResolution(TimeUnit resolution) {
        this.resolution = resolution;
    }

    public long getLoopTime() {
        return getLoopTime(resolution);
    }
    public long getLoopTime(TimeUnit resolution) {
        return resolution.convert(delta, TimeUnit.NANOSECONDS);
    }

    public long getRefreshRate() {
        return 1000000000 / delta;
    }

    public LoopRateTracker update() {
        long time = System.nanoTime();
        delta = time - lastTime;
        lastTime = time;
        return this;
    }
}
