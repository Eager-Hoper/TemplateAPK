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
        Point point1 = new Point(11.2053d, -9.87284d, 5.4736d);
        Point point2 = new Point(10.456184d, -9.196272d, 4.53d);
        Point point3 = new Point(10.7142d, -7.76727d, 4.53d);
        Point point4 = new Point(10.56d, -6.612872d, 5.20641d );
        Point point7 = new Point(11.369d, -8.5518d, 4.48d);
        Point point8 = new Point(11.143d, -6.7607d, 4.9654d);

        Quaternion quartanion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Quaternion quartanion2 = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
        Quaternion quartanion3 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quartanion4 = new Quaternion(0f, 0f, -1f, 0f);
        Quaternion quartanion7 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quartanion8 = new Quaternion(0f, 0f, -0.707f, 0.707f);

        //Viapoints
        Point viapoint01 = new Point(10.59838d, -9.83515d, 5.24227d);
        Point viapoint03First = new Point(10.6588d, -9.19627d, 4.53d);
        Point viapoint03Second = new Point(10.6942d, -8.28308d, 4.97737d);
        Point viapoint04 = new Point(10.6588d, -9.19627d, 4.79855d);

        Point viapoint12 = new Point(11.02164d, -9.50949d, 5.2188d);
        Point viapoint13 = new Point(10.80698d, -8.16508d, 4.87503d);
        Point viapoint14 = new Point(10.78123d, -7.7305d, 5.36006d);
        Point viapoint18 = new Point(11.16135d, -7.67756d, 5.35803d);

        Point viapoint23 = new Point(10.62107d, -8.28308d, 4.97737d);
        Point viapoint24 = new Point(10.48602d, -8.45931d, 4.89368d);
        Point viapoint27 = new Point(10.93d, -8.94d, 5.12d);
        Point viapoint28 = new Point(10.49585d, -7.393d, 5.30908d);

        Point viapoint34 = new Point(10.64695d, -7.26384d, 5.02173d);

        Point viapoint47 = new Point(10.64904d, -8.22217d, 5.29178d);

        Point viapoint78 = new Point(11.22584d, -8.80419d, 5.0922d);

        Point pivotPoint11 = new Point(10.85076d, -9.314d, 5.269d);
        Point pivotPoint12 = new Point(11.2d, -9.109d, 5.191d);
        Point pivotPoint2 = new Point(11.34547d, -7.393d, 4.63477d);
        Point pivotPoint3 = new Point(10.49585d, -7.393d, 5.30908d);

        Point viaPivot11to3 = new Point(10.47d, -7.902d, 4.948d);

        //not considering the remaining time yet.
        // List<Long> time = new ArrayList<Long>();

        // remain start_log
        Log. i(TAG, "arata: start mission");
        
        // the mission starts
        api.startMission();

        // move to target_6
        Point point_6 = new Point(11.355d, -9.0462d, 4.9416d);
        Quaternion quaternion_6 = new Quaternion(0f, 0f, 0f, 1f);
        api.moveTo(point_6, quaternion_6, true);

        api.laserControl(true);
        api.takeTargetSnapshot(6);
        api.laserControl(false);

        //move to target_7(QRCode)
        Point point_7 = new Point(11.369d, -8.5518d, 4.48d);
        Quaternion quaternion_7 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        api.moveTo(point_7, quaternion_7, true);

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

        // move to target_1
        Point point_1 = new Point(11.2053d, -9.92284d, 5.4736d);
        Quaternion quaternion_1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(point_1, quaternion_1, true);

        api.laserControl(true);
        api.takeTargetSnapshot(1);
        api.laserControl(false);

        // move to taget_5
        Point point_5 = new Point(11.0448d, -7.9193d, 5.3393d);
        Quaternion quaternion_5 = new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f);
        api.moveTo(point_5, quaternion_5, true);
        
        api.laserControl(true);
        api.takeTargetSnapshot(5);
        api.laserControl(false);

        // move to taget_4
        Point point_4 = new Point(10.51d, -6.612872d, 5.20641d);
        Quaternion quaternion_4 = new Quaternion(0f, 0f, -1f, 0f);
        api.moveTo(point_4, quaternion_4, true);
        
        api.laserControl(true);
        api.takeTargetSnapshot(4);
        api.laserControl(false);

        //Declare we are approaching the goal(Required)
        api.notifyGoingToGoal();

        // move to the goal
        Point Goal = new Point(11.143d, -6.7607d, 4.9654d);
        Quaternion quaternion_Goal = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(Goal, quaternion_Goal, true);

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
    
}
