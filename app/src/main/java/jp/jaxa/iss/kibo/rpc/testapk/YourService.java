package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
// Kibo-RPC library

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;
// astrobee library (for definition of Point and Quaternion etc.)

import android.util.Log;
// android library (for log)

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Rect;
import org.opencv.objdetect.QRCodeDetector;

import static org.opencv.android.Utils.matToBitmap;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.undistort;
// opencv library (for detect ARmarkers)

import java.security.IdentityScope;
import java.util.ArrayList;
import java.util.List;
import java.lang.Math;
// java library (for basic operate)

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    // setting for log
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long ActiveTime  = TimeRemaining.get(0) / 1000;
        Long MissionTime = TimeRemaining.get(1) / 1000;

        // the mission starts
        //example 2 5 6 7 6
        api.startMission();
        moveAndShot(0, 2);

        moveAndShot(2, 5);
        moveAndShot(5, 6);
        String reportMessage = moveAndShot(6, 7);
        moveAndShot(7, 6);

        // move to the goal
        api.notifyGoingToGoal();
        moveAndShot(6, 8);

        //report goal message and finish
        api.reportMissionCompletion(reportMessage);
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }

    public String moveAndShot(int from, int to){
        Point point1 = new Point(11.2053d, -9.92284d, 5.4736d);
        Point point2 = new Point(10.456184d, -9.196272d, 4.48d);
        Point point3 = new Point(10.7142d, -7.76727d, 4.48d);
        Point point4 = new Point(10.51d, -6.612872d, 5.20641d);
        Point point5 = new Point(11.0448d, -7.9193d, 5.3393d);
        Point point6 = new Point(11.355d, -9.0462d, 4.9416d);
        Point point7 = new Point(11.369d, -8.5518d, 4.48d);
        Point point8= new Point(11.143d, -6.7607d, 4.9654d);

        Quaternion quartanion1 = new Quaternion(0f, 0f, -0.707f, -0.707f);
        Quaternion quartanion2 = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
        Quaternion quartanion3 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quartanion4 = new Quaternion(0f, 0f, -1f, 0f);
        Quaternion quartanion5 = new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f);
        Quaternion quartanion6 = new Quaternion(0f, 0f, 0f, 1f);
        Quaternion quartanion7 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quartanion8 = new Quaternion(0f, 0f, -0.707f, 0.707f);

        //Viapoints
        Point viapoint03 = new Point();
        Point viapoint07 = new Point();

        Point viapoint12 = new Point();
        Point viapoint18 = new Point();

        Point viapoint23 = new Point(10.66512d, -8.3278d, 4.75812d);
        Point viapoint24 = new Point();
        Point viapoint26 = new Point();
        Point viapoint27 = new Point();
        Point viapoint28 = new Point();

        Point viapoint34 = new Point();
        Point viapoint35 = new Point();
        Point viapoint36 = new Point();
        Point viapoint37 = new Point();

        Point viapoint47 = new Point();

        Point viapoint78 = new Point();

        String reportMessage = null;

        switch (from){
            case 0:
                switch(to){
                    case 1:
                        api.moveTo(point1, quartanion1, true);
                        break;
                    case 2:
                        api.moveTo(point2, quartanion2, true);
                        break;
                    case 3:
                        api.moveTo(viapoint03, quartanion3, true);
                        api.moveTo(point3, quartanion3, true);
                        break;
                    case 4:
                        api.moveTo(point4, quartanion4, true);
                        break;
                    case 5:
                        api.moveTo(point5, quartanion5, true);
                        break;
                    case 6:
                        api.moveTo(point6, quartanion6, true);
                        break;
                    case 7:
                        api.moveTo(viapoint07, quartanion7, true);
                        api.moveTo(point7, quartanion7, true);
                        break;
                    default:
                        break;
                }
            case 1:
                switch(to){
                    case 2:
                        api.moveTo(viapoint12, quartanion2, true);
                        api.moveTo(point2, quartanion2, true);
                        break;
                    case 3:
                        api.moveTo(point3, quartanion3, true);
                        break;
                    case 4:
                        api.moveTo(point4, quartanion4, true);
                        break;
                    case 5:
                        api.moveTo(point5, quartanion5, true);
                        break;
                    case 6:
                        api.moveTo(point6, quartanion6, true);
                        break;
                    case 7:
                        api.moveTo(point7, quartanion7, true);
                        break;
                    case 8:
                        api.moveTo(viapoint18, quartanion8, true);
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
            case 2:
                switch(to){
                    case 1:
                        api.moveTo(viapoint12, quartanion1, true);
                        api.moveTo(point1, quartanion1, true);
                        break;
                    case 3:
                        api.moveTo(viapoint23, quartanion3, true);
                        api.moveTo(point3, quartanion3, true);
                        break;
                    case 4:
                        api.moveTo(viapoint24, quartanion4, true);
                        api.moveTo(point4, quartanion4, true);
                        break;
                    case 5:
                        api.moveTo(point5, quartanion5, true);
                        break;
                    case 6:
                        api.moveTo(viapoint26, quartanion6, true);
                        api.moveTo(point6, quartanion6, true);
                        break;
                    case 7:
                        api.moveTo(viapoint27, quartanion7, true);
                        api.moveTo(point7, quartanion7, true);
                        break;
                    case 8:
                        api.moveTo(viapoint28, quartanion8, true);
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
            case 3:
                switch(to){
                    case 1:
                        api.moveTo(point1, quartanion1, true);
                        break;
                    case 2:
                        api.moveTo(viapoint23, quartanion2, true);
                        api.moveTo(point2, quartanion2, true);
                        break;
                    case 4:
                        api.moveTo(viapoint34, quartanion4, true);
                        api.moveTo(point4, quartanion4, true);
                        break;
                    case 5:
                        api.moveTo(viapoint35, quartanion5, true);
                        api.moveTo(point5, quartanion5, true);
                        break;
                    case 6:
                        api.moveTo(viapoint36, quartanion6, true);
                        api.moveTo(point6, quartanion6, true);
                        break;
                    case 7:
                        api.moveTo(viapoint37, quartanion7, true);
                        api.moveTo(point7, quartanion7, true);
                        break;
                    case 8:
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
            case 4:
                switch(to){
                    case 1:
                        api.moveTo(point1, quartanion1, true);
                        break;
                    case 2:
                        api.moveTo(viapoint24, quartanion2, true);
                        api.moveTo(point2, quartanion2, true);
                        break;
                    case 3:
                        api.moveTo(viapoint34, quartanion4, true);
                        api.moveTo(point3, quartanion3, true);
                        break;
                    case 5:
                        api.moveTo(point5, quartanion5, true);
                        break;
                    case 6:
                        api.moveTo(point6, quartanion6, true);
                        break;
                    case 7:
                        api.moveTo(viapoint47, quartanion7, true);
                        api.moveTo(point7, quartanion7, true);
                        break;
                    case 8:
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
            case 5:
                switch(to){
                    case 1:
                        api.moveTo(point1, quartanion1, true);
                        break;
                    case 2:
                        api.moveTo(point2, quartanion2, true);
                        break;
                    case 3:
                        api.moveTo(viapoint35, quartanion3, true);
                        api.moveTo(point3, quartanion3, true);
                        break;
                    case 4:
                        api.moveTo(point4, quartanion4, true);
                        break;
                    case 6:
                        api.moveTo(point6, quartanion6, true);
                        break;
                    case 7:
                        api.moveTo(point7, quartanion7, true);
                        break;
                    case 8:
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
            case 6:
                switch(to){
                    case 1:
                        api.moveTo(point1, quartanion1, true);
                        break;
                    case 2:
                        api.moveTo(viapoint26, quartanion2, true);
                        api.moveTo(point2, quartanion2, true);
                        break;
                    case 3:
                        api.moveTo(viapoint36, quartanion3, true);
                        api.moveTo(point3, quartanion3, true);
                        break;
                    case 4:
                        api.moveTo(point4, quartanion4, true);
                        break;
                    case 5:
                        api.moveTo(point5, quartanion5, true);
                        break;
                    case 7:
                        api.moveTo(point7, quartanion7, true);
                        break;
                    case 8:
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
            case 7:
                switch(to){
                    case 1:
                        api.moveTo(point1, quartanion1, true);
                        break;
                    case 2:
                        api.moveTo(viapoint27, quartanion2, true);
                        api.moveTo(point2, quartanion2, true);
                        break;
                    case 3:
                        api.moveTo(viapoint37, quartanion3, true);
                        api.moveTo(point3, quartanion3, true);
                        break;
                    case 4:
                        api.moveTo(viapoint47, quartanion4, true);
                        api.moveTo(point4, quartanion4, true);
                        break;
                    case 5:
                        api.moveTo(point5, quartanion5, true);
                        break;
                    case 6:
                        api.moveTo(point6, quartanion6, true);
                        break;
                    case 8:
                        api.moveTo(viapoint78, quartanion8, true);
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
            default:
                break;
        }

        if(to == 7){
            reportMessage = ReadQR();
        }else if(to == 8){

        }else{
            api.laserControl(true);
            api.takeTargetSnapshot(to);
            api.laserControl(false);
        }

        return reportMessage;
    }

    public Mat image_correction(Mat image) {

        double[][] NavCamIntrinsics = api.getNavCamIntrinsics();
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distortion = new Mat(1, 5, CvType.CV_32FC1);
        cameraMat.put(0, 0, NavCamIntrinsics[0]);
        distortion.put(0, 0, NavCamIntrinsics[1]);

        Mat correct_image = new Mat();
        undistort(image, correct_image, cameraMat, distortion);

        Log.i(TAG, "arata: get correct_image");
        return correct_image;

    }

    public String ReadQR(){
        //ReadQRCode
        Mat QRimage = image_correction(api.getMatNavCam());
        //Generate png image for debug
        api.saveMatImage(QRimage, "QR.png");

        QRCodeDetector decoder = new QRCodeDetector();
        String data = decoder.detectAndDecode(QRimage);

        String reportMessage = null;
        switch(data) {
            case "JEM":
                reportMessage = "STAY_AT_JEM";
                break;
            case "COLUMBUS":
                reportMessage = "GO_TO_COLUMBUS";
                break;
            case "RACK1":
                reportMessage = "CHECK_RACK_1";
                break;
            case "ASTROBEE":
                reportMessage = "I_AM_HERE";
                break;
            case "INTBALL":
                reportMessage = "LOOKING_FORWARD_TO_SEE_YOU";
                break;
            case "BLANK":
                reportMessage = "NO_PROBLEM";
                break;
            default:
                break;
        }
        return reportMessage;
    }

}

