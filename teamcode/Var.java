package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Var {
    public enum DetectionTypes {
        DAY_green,
        NIGHT_green,
        DAY_blue,
        NIGHT_blue,
        DAY_yellow,
        DAY_White,
        DAY_red
    }

    public static double Day_Hhigh = 90, Day_Shigh = 255, Day_Vhigh = 255, Day_Hlow = 50, Day_Slow = 20, Day_Vlow = 0;
    public static double Night_Hhigh = 19, Night_Shigh = 255, Night_Vhigh = 255, Night_Hlow = 12, Night_Slow = 100, Night_Vlow = 50;
    public static double Day_Hhigh_albastru = 138, Day_Shigh_albastru = 255, Day_Vhigh_albastru = 255, Day_Hlow_albastru = 78, Day_Slow_albastru = 150, Day_Vlow_albastru = 0;
    public static double Night_Hhigh_albastru = 19, Night_Shigh_albastru = 255, Night_Vhigh_albastru = 255, Night_Hlow_albastru = 12, Night_Slow_albastru = 100, Night_Vlow_albastru = 50;
    public static double Day_Hhigh_red = 180, Day_Shigh_red = 255, Day_Vhigh_red = 255, Day_Hlow_red = 170, Day_Slow_red = 150, Day_Vlow_red = 100;
    public static double Day_Hhigh_yellow = 30, Day_Shigh_yellow = 255, Day_Vhigh_yellow = 255, Day_Hlow_yellow = 20, Day_Slow_yellow = 100, Day_Vlow_yellow = 100;

    public static double Day_Hhigh_white = 255, Day_Shigh_white = 100, Day_Vhigh_white = 255, Day_Hlow_white = 0, Day_Slow_white = 0, Day_Vlow_white = 177;

    public static double kp = 0.000002, ki = 0, kd = 0.0002;
    public static int CV_kernel_pult_size = 5, Webcam_w = 640, Webcam_h = 480, CV_rect_x1 = 0, CV_rect_y1 = 0, CV_rect_x2 = 640, CV_rect_y2 = 240;
    public static int dreptunghi_y_sus = 1280;
    public static DetectionTypes CV_detectionType = DetectionTypes.DAY_red;
    public static Scalar scalar = new Scalar(0,0,0);
}