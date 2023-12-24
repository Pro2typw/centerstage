package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;


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
    
    //todo config
    @Config(value = "[]ClawConstants")
    public static class Claw {
        public final static String RIGHT_CLAW_MAP_NAME = ""; //todo config
        public final static String LEFT_CLAW_MAP_NAME = ""; //todo config
        
        public static int RIGHT_CLAW_OPEN_POSITION = 0; //todo config
        public static int RIGHT_CLAW_CLOSE_POSITION = 0; //todo config
        public static int LEFT_CLAW_OPEN_POSITION = 0; //todo config
        public static int LEFT_CLAW_CLOSE_POSITION = 0; //todo config
    }
    
    @Config(value = "[]IMU")
    public static class IMU {
        public final static String IMU_MAP_NAME = "imu";
        
        public static PIDCoefficients HEADING_PID_COEFFICIENTS = new PIDCoefficients(0, 0, 0);  //todo config
    }

    @Config(value = "[]Hang")
    public static class Hang {
        public final static String HANG1_MAP_NAME = "hang1";
        public final static String HANG2_MAP_NAME = "hang2";

        public static int UP_POSITION = 0;
        public static int DOWN_POSITION = 0;

    }
    
    
}
