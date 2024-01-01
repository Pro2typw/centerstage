package org.firstinspires.ftc.teamcode.subsystem.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Constants {

    //todo config constants
    @Config(value = "[]VisionConstants")
    public static class Vision {
        public static int BLUE_CENTER_RECTANGLE_TOP_LEFT_X = 0;
        public static int BLUE_CENTER_RECTANGLE_TOP_LEFT_Y = 0;
        public static int BLUE_CENTER_RECTANGLE_BOTTOM_RIGHT_X = 300;
        public static int BLUE_CENTER_RECTANGLE_BOTTOM_RIGHT_Y = 300;

        public static int BLUE_LEFT_RECTANGLE_TOP_LEFT_X = 0;
        public static int BLUE_LEFT_RECTANGLE_TOP_LEFT_Y = 0;
        public static int BLUE_LEFT_RECTANGLE_BOTTOM_RIGHT_X = 0;
        public static int BLUE_LEFT_RECTANGLE_BOTTOM_RIGHT_Y = 0;

        public static int BLUE_RIGHT_RECTANGLE_TOP_LEFT_X = 0;
        public static int BLUE_RIGHT_RECTANGLE_TOP_LEFT_Y = 0;
        public static int BLUE_RIGHT_RECTANGLE_BOTTOM_RIGHT_X = 0;
        public static int BLUE_RIGHT_RECTANGLE_BOTTOM_RIGHT_Y = 0;


        public static int RED_CENTER_RECTANGLE_TOP_LEFT_X = 100;
        public static int RED_CENTER_RECTANGLE_TOP_LEFT_Y = 100;
        public static int RED_CENTER_RECTANGLE_BOTTOM_RIGHT_X = 0;
        public static int RED_CENTER_RECTANGLE_BOTTOM_RIGHT_Y = 0;

        public static int RED_LEFT_RECTANGLE_TOP_LEFT_X = 0;
        public static int RED_LEFT_RECTANGLE_TOP_LEFT_Y = 0;
        public static int RED_LEFT_RECTANGLE_BOTTOM_RIGHT_X = 0;
        public static int RED_LEFT_RECTANGLE_BOTTOM_RIGHT_Y = 0;

        public static int RED_RIGHT_RECTANGLE_TOP_LEFT_X = 0;
        public static int RED_RIGHT_RECTANGLE_TOP_LEFT_Y = 0;
        public static int RED_RIGHT_RECTANGLE_BOTTOM_RIGHT_X = 0;
        public static int RED_RIGHT_RECTANGLE_BOTTOM_RIGHT_Y = 0;
    }

    @Config(value = "[]Camera")
    public static class Camera {
        public final static String CAMERA_MAP_NAME = "Webcam 1";
    }

    //todo config
    @Config(value = "[]ClawConstants")
    public static class Claw {
        public final static String RIGHT_CLAW_MAP_NAME = "";
        public final static String LEFT_CLAW_MAP_NAME = "";
        
        public static int RIGHT_CLAW_OPEN_POSITION = 0;
        public static int RIGHT_CLAW_CLOSE_POSITION = 0;
        public static int LEFT_CLAW_OPEN_POSITION = 0;
        public static int LEFT_CLAW_CLOSE_POSITION = 0;
    }

    // todo config ig the entire file but ye
    @Config
    public static class Arm {
        public final static String MOTOR1_MAP_NAME = "";
        public final static String MOTOR2_MAP_NAME = "";

        public static PIDCoefficients DIFFERENCE_PID_COEFFICIENTS = new PIDCoefficients(0, 0, 0);
        public static PIDCoefficients AVERAGE_PID_COEFFICIENTS = new PIDCoefficients(0, 0, 0);
        public static double GRAVITY_FEEDFORWARD_GAIN = 0;

        public static int targetPivotPosition = 0;
        public static int targetExtensionPosition = 0;


    }

    // todo
    @Config(value = "[]Hang")
    public static class Hang {
        public final static String LEFT_HANG_MAP_NAME = "hang1";
        public final static String RIGHT_HANG_MAP_NAME = "hang2";

        public static int UP_POSITION = 0;
        public static int DOWN_POSITION = 0;

        public static PIDFCoefficients LEFT_HANG_PID_COEFFICIENTS = new PIDFCoefficients(0, 0, 0, 0);
        public static PIDFCoefficients RIGHT_HANG_PID_COEFFICIENTS = new PIDFCoefficients(0, 0, 0, 0);

    }

    @Config(value = "[]IMU")
    public static class IMU {
        public final static String IMU_MAP_NAME = "imu";
        
        public static PIDCoefficients HEADING_PID_COEFFICIENTS = new PIDCoefficients(0, 0, 0);  //todo config
    }
    
}
