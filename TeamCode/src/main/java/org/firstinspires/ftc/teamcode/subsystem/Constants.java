package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;


public class Constants {
    @Config(value = "[]Camera Constants")
    public static class Camera {
        public final static String CAMERA_MAP_NAME = "Webcam 1";

        public final static double X_OFFSET = 0;
        public final static double Y_OFFSET = 0;
    }

    @Config(value = "[]Claw Constants")
    public static class Claw {
        public final static String RIGHT_CLAW_MAP_NAME = "right claw"; // named from facing the back of the claw (not thw way it opens but the other way
        public final static String LEFT_CLAW_MAP_NAME = "left claw"; // named from facing the back of the claw (not thw way it opens but the other way

        public final static String RIGHT_CLAW_DISTANCE_SENSOR_MAP = "right claw distance sensor";
        public final static String LEFT_CLAW_DISTANCE_SENSOR_MAP = "left claw distance sensor";


        public static double RIGHT_CLAW_OPEN_POSITION = .45;
        public static double RIGHT_CLAW_CLOSE_POSITION = .7;
        public static double LEFT_CLAW_OPEN_POSITION = .5;
        public static double LEFT_CLAW_CLOSE_POSITION = .225;
    }

    @Config(value = "[]Arm Constants")
    public static class Arm {
        public final static String MOTOR1_MAP_NAME = "arm left";
        public final static String MOTOR2_MAP_NAME = " arm right";
//        public final static String EXTENSION_ENCODER_MAP_NAME = "fr";


        public static double G_PIVOT = 2.05652575;
        public static double G_PIVOT2 = .23;
        public static double G_EXTENSION = .01;
        public static double G_STATIC = 0.000007;
        public static PIDCoefficients AVERAGE_PID_COEFFICIENTS = new PIDCoefficients(0.01, 2.5e-11, 10000);
        public static PIDCoefficients DIFFERENCE_PID_COEFFICIENTS = new PIDCoefficients(0.0036, 2.5e-12, 170000);

        public static double VOLTAGE_COMPENSATION_CONSTANT = -1;

        public static final int MAX_EXTENSION = 500; // ticks
        public static final int MAX_PIVOT = 635; // ticks


    }

    @Config
    public static class Drone {
        public static final String LAUNCHER_MAP_NAME = "launcherServo";

        public static final double LAUNCHER_LAUNCH_POWER = .65;
    }

    @Config(value = "[]Hang Constants")
    public static class Hang {
        public final static String LEFT_HANG_MAP_NAME = "left hang";
        public final static String RIGHT_HANG_MAP_NAME = "right hang";

        public static int INIT_POSITION = 0;
        public static int UP_POSITION = 0;
        public static int DOWN_POSITION = 0;

    }

    @Config(value = "[]Wrist Constants")
    public static class Wrist {
        public final static String LEFT_WRIST_MAP_NAME = "left wrist";
        public final static String RIGHT_WRIST_MAP_NAME = "right wrist";


        public static double INIT_POS = .657;
        public static double INTAKE_POS = .263;
        public static double DEPO_POS_580 = .263;
        public static double DEPO_POS_635 = .363;
    }

    @Config
    public static class Plane {
        public final static String DRONE_MAP_NAME = "drone";
        public static double INIT_POS = .2;
        public static double LAUNCH_POS = .68;
    }

}