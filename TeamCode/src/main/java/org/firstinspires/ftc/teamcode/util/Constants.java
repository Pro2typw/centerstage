package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;


public class Constants {

    //todo config constants
    @Config(value = "[]VisionConfig")
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
    public static class Claw {
        public static String RIGHT_CLAW_MAP_NAME = ""; //todo config
        public static String LEFT_CLAW_MAP_NAME = ""; //todo config
        
        public static int RIGHT_CLAW_OPEN_POSITION = 0;
        public static int RIGHT_CLAW_CLOSE_POSITION = 0;
        public static int LEFT_CLAW_OPEN_POSITION = 0;
        public static int LEFT_CLAW_CLOSE_POSITION = 0;
        
        
    }
}
