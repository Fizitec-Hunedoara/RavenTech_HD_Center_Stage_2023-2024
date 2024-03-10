package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var.CV_kernel_pult_size;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_x1;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_x2;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_y1;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_y2;
import static org.firstinspires.ftc.teamcode.Var.Day_Hhigh;
import static org.firstinspires.ftc.teamcode.Var.Day_Hhigh_albastru;
import static org.firstinspires.ftc.teamcode.Var.Day_Hhigh_red;
import static org.firstinspires.ftc.teamcode.Var.Day_Hhigh_white;
import static org.firstinspires.ftc.teamcode.Var.Day_Hhigh_yellow;
import static org.firstinspires.ftc.teamcode.Var.Day_Hlow;
import static org.firstinspires.ftc.teamcode.Var.Day_Hlow_albastru;
import static org.firstinspires.ftc.teamcode.Var.Day_Hlow_red;
import static org.firstinspires.ftc.teamcode.Var.Day_Hlow_white;
import static org.firstinspires.ftc.teamcode.Var.Day_Hlow_yellow;
import static org.firstinspires.ftc.teamcode.Var.Day_Shigh;
import static org.firstinspires.ftc.teamcode.Var.Day_Shigh_albastru;
import static org.firstinspires.ftc.teamcode.Var.Day_Shigh_red;
import static org.firstinspires.ftc.teamcode.Var.Day_Shigh_white;
import static org.firstinspires.ftc.teamcode.Var.Day_Shigh_yellow;
import static org.firstinspires.ftc.teamcode.Var.Day_Slow;
import static org.firstinspires.ftc.teamcode.Var.Day_Slow_albastru;
import static org.firstinspires.ftc.teamcode.Var.Day_Slow_red;
import static org.firstinspires.ftc.teamcode.Var.Day_Slow_white;
import static org.firstinspires.ftc.teamcode.Var.Day_Slow_yellow;
import static org.firstinspires.ftc.teamcode.Var.Day_Vhigh;
import static org.firstinspires.ftc.teamcode.Var.Day_Vhigh_albastru;
import static org.firstinspires.ftc.teamcode.Var.Day_Vhigh_red;
import static org.firstinspires.ftc.teamcode.Var.Day_Vhigh_white;
import static org.firstinspires.ftc.teamcode.Var.Day_Vhigh_yellow;
import static org.firstinspires.ftc.teamcode.Var.Day_Vlow;
import static org.firstinspires.ftc.teamcode.Var.Day_Vlow_albastru;
import static org.firstinspires.ftc.teamcode.Var.Day_Vlow_red;
import static org.firstinspires.ftc.teamcode.Var.Day_Vlow_white;
import static org.firstinspires.ftc.teamcode.Var.Day_Vlow_yellow;
import static org.firstinspires.ftc.teamcode.Var.Night_Hhigh;
import static org.firstinspires.ftc.teamcode.Var.Night_Hlow;
import static org.firstinspires.ftc.teamcode.Var.Night_Shigh;
import static org.firstinspires.ftc.teamcode.Var.Night_Slow;
import static org.firstinspires.ftc.teamcode.Var.Night_Vhigh;
import static org.firstinspires.ftc.teamcode.Var.Night_Vlow;

import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

public class PachetelNouHistOpenCV extends OpenCvPipeline {
    //stabileste forma detectorului
    private final int elementType = Imgproc.CV_SHAPE_RECT;
    //asta e un dreptunghi(Rect = dreptunghi pentru webcam)
    private Rect dreptunghi;
    LinearOpMode opMode;
    public PachetelNouHistOpenCV(LinearOpMode opMode){

        this.opMode = opMode;
    }
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    //mat = foaie de desen pentru webcam
    public Mat processFrame(Mat input) {
        try {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            //face un patrat de latura kernel_pult_size si cu ancora in centru
            Mat element = Imgproc.getStructuringElement(elementType, new Size(2 * CV_kernel_pult_size + 1, 2 * CV_kernel_pult_size + 1),
                    new Point(CV_kernel_pult_size, CV_kernel_pult_size));
            //creeaza o copie a imaginii de pe webcam
            Mat original = input.clone();

            //Scalari de HSV(totusi H e jumate din valorile de pe color picker)
            Scalar scalarLowerHSV, scalarUpperHSV;

            //daca e day da valorile de day, daca e night da valorile de night

            if(CV_detectionType == Var.DetectionTypes.DAY_green) {
                scalarLowerHSV = new Scalar(Day_Hlow, Day_Slow, Day_Vlow);
                scalarUpperHSV = new Scalar(Day_Hhigh, Day_Shigh, Day_Vhigh);
            }
            else if(CV_detectionType == Var.DetectionTypes.NIGHT_green){
                scalarLowerHSV = new Scalar(Night_Hlow, Night_Slow, Night_Vlow);
                scalarUpperHSV = new Scalar(Night_Hhigh, Night_Shigh, Night_Vhigh);
            }
            else if(CV_detectionType == Var.DetectionTypes.DAY_blue){
                scalarLowerHSV = new Scalar(Day_Hlow_albastru, Day_Slow_albastru, Day_Vlow_albastru);
                scalarUpperHSV = new Scalar(Day_Hhigh_albastru, Day_Shigh_albastru, Day_Vhigh_albastru);
            }
            else if(CV_detectionType == Var.DetectionTypes.DAY_White){
                scalarLowerHSV = new Scalar(Day_Hlow_white, Day_Slow_white, Day_Vlow_white);
                scalarUpperHSV = new Scalar(Day_Hhigh_white, Day_Shigh_white, Day_Vhigh_white);
            }
            else if (CV_detectionType == Var.DetectionTypes.DAY_red){
                scalarLowerHSV = new Scalar(Day_Hlow_red, Day_Slow_red, Day_Vlow_red);
                scalarUpperHSV = new Scalar(Day_Hhigh_red, Day_Shigh_red, Day_Vhigh_red);
            }
            else if (CV_detectionType == Var.DetectionTypes.DAY_yellow){
                scalarLowerHSV = new Scalar(Day_Hlow_yellow, Day_Slow_yellow, Day_Vlow_yellow);
                scalarUpperHSV = new Scalar(Day_Hhigh_yellow, Day_Shigh_yellow, Day_Vhigh_yellow);
            }
            else{
                scalarLowerHSV = new Scalar(Day_Hlow_albastru, Day_Slow_albastru, Day_Vlow_albastru);
                scalarUpperHSV = new Scalar(Day_Hhigh_albastru, Day_Shigh_albastru, Day_Vhigh_albastru);
            }

            //asta converteste culorile din input de la RGB la HSV
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);


            //Core.split(input, channels);

            //aceasta linie de cod face un dreptunghi cat webcam-ul de mare si negru


//            Mat histH = new Mat();
//            Mat histS = new Mat();
//            Mat histV = new Mat();
//
//            StringBuilder s;
//            Imgproc.calcHist(Collections.singletonList(input),
//                    new MatOfInt(0),
//                    rect,
//                    histH,
//                    new MatOfInt(180),
//                    new MatOfFloat(0, 180)
//            );
//            s = new StringBuilder(String.valueOf(histH.get(0, 0)[0]));
////            for(int i = 0; i < 180; i++){
////                s.append(", ").append(histH.get(i, 0)[0]);
////            }
////            Log.d("H", s.toString());
////
//            Imgproc.calcHist(Collections.singletonList(input),
//                    new MatOfInt(1),
//                    rect,
//                    histS,
//                    new MatOfInt(256),
//                    new MatOfFloat(0, 255)
//            );
////            s = new StringBuilder(String.valueOf(histS.get(0, 0)[0]));
////            for(int i = 0; i < 256; i++){
////                s.append(", ").append(histS.get(i, 0)[0]);
////            }
////            Log.d("S", s.toString());
//
//            Imgproc.calcHist(Collections.singletonList(input),
//                    new MatOfInt(2),
//                    rect,
//                    histV,
//                    new MatOfInt(256),
//                    new MatOfFloat(0, 255)
//            );
//
//            s = new StringBuilder(String.valueOf(histV.get(0, 0)[0]));
//            for(int i = 0; i < 256; i++){
//                s.append(", ").append(histV.get(i, 0)[0]);
//            }
//            //Log.d("V", s.toString());
//
//            float[] histVArray = new float[256];
//            for(int i = 0; i < 256; i++) {
//                histVArray[i] = (float) histV.get(i, 0)[0];
//            }
//
//            int pula = 0;
//            float max = histVArray[0];
//            for (int i = 1;i < 256; i++){
//                if (histVArray[i] > max){
//                    max = histVArray[i];
//                    pula = i;
//                }
//            }
//            histVArray[pula] = 0;
//            float max2 = histVArray[0];
//            for (int i = 1; i< 256; i++){
//                if (histVArray[i] > max){
//                    max2 = histVArray[i];
//                }
//            }
//            Mat gray_image = new Mat() ;
//            Imgproc.cvtColor(input, gray_image, Imgproc.COLOR_BGR2GRAY);
//
//                Imgproc.threshold(channels.get(1), channels.get(2), 50, 255, Imgproc.THRESH_OTSU);
            //Converting matrix to JavaFX writable image

//
//
//            if(CV_detectionType == Var.DetectionTypes.DAY_White){
//                scalarLowerHSV = new Scalar(Day_Hlow_white, Day_Slow_white, max2);
//                scalarUpperHSV = new Scalar(Day_Hhigh_white, Day_Shigh_white, max);
//            }
////
////            Log.d("max", String.valueOf(max));
////            Log.d("max2", String.valueOf(max2));
//            histH.release();
//            histS.release();
//            histV.release();




            Core.inRange(input, scalarLowerHSV, scalarUpperHSV, input);

            Imgproc.erode(input, input, element);
            Imgproc.dilate(input, input, element);
            Imgproc.dilate(input, input, element);
            Imgproc.erode(input, input, element);

            Mat rect = new Mat(input.rows(), input.cols(), CvType.CV_8UC1, Scalar.all(0));

            Imgproc.rectangle(
                    rect,
                    new Point(CV_rect_x1, CV_rect_y1),
                    new Point(CV_rect_x2, CV_rect_y2),
                    new Scalar(255),
                    Imgproc.FILLED
            );

            Core.bitwise_and(input, rect, input);


            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Collections.sort(contours, new Comparator<MatOfPoint>() {
                @Override
                public int compare(MatOfPoint matOfPoint, MatOfPoint t1) {
                    return (int) ( (calculateWidth(t1) - calculateWidth(matOfPoint)));
                }
            });
            if (!contours.isEmpty()) {
                setRect(Imgproc.boundingRect(contours.get(0)));
            }else {
                setRect(new Rect());
            }
            Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGBA);

            Core.bitwise_or(input, original, input);

            Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 4);

            Imgproc.rectangle(
                    input,
                    new Point(CV_rect_x1, CV_rect_y1),
                    new Point(CV_rect_x2, CV_rect_y2),
                    new Scalar(255, 127, 0), 4);

            Imgproc.rectangle(
                    input,
                    getRect(),
                    new Scalar(0, 255, 255), 4);

            original.release();
            rect.release();

            return input;
        }
        catch (Exception E){
            return input;
        }
    }

    public void setRect(Rect rect) {
        this.dreptunghi = rect;
    }

    public Rect getRect() {
        return dreptunghi;
    }

    private double calculateWidth(@NonNull MatOfPoint contour) {
        // Find the maximum x-coordinate and minimum x-coordinate
        double maxX = Double.MIN_VALUE;
        double minX = Double.MAX_VALUE;
        for (int i = 0; i < contour.rows(); i++) {
            double[] point = contour.get(i, 0);
            double x = point[0];
            if (x > maxX) {
                maxX = x;
            }
            if (x < minX) {
                minX = x;
            }
        }

        // Calculate width as the difference between maximum and minimum x-coordinates
        return maxX - minX;
    }
}