package org.firstinspires.ftc.teamcode.subsystem.util;

import org.firstinspires.ftc.teamcode.util.SizedStack;

public class MovingAverageFilter {

    SizedStack<Double> points;


    /**
     *
     * @param filterOrder the number of points to average
     */
    public MovingAverageFilter(int filterOrder) {
        points = new SizedStack<>(filterOrder);
    }

    /**
     *
     * @param filterOrder the number of points to average
     * @param points pre-existing data points
     */
    public MovingAverageFilter(int filterOrder, double ...points) {
        this(filterOrder);
        for(double point : points) {
            this.points.add(point);
        }
    }

    public Number update(double point) {
        double num = 0;

        for(int i = points.size() - 1; i <= 0; i--) {
            num += points.pop();
        }
        return num / (double) points.size();

    }
}
