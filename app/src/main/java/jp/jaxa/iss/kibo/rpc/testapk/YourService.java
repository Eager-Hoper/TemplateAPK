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
import org.opencv.calib3d;
import org.opencv.core.Rect;
import static org.opencv.android.Utils.matToBitmap;
// opencv library (for detect ARmarkers)

import java.util.ArrayList;
import java.util.List;
// java library (for basic operate)

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    // setting for log
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
        // remain start_log
        Log. i(TAG, "start mission");

        // the mission starts
        api.startMission();

        // definition of destination and quaternion
        Point point = new Point(11.355d, -8.9929d, 4.7818d);
        Quaternion quaternion = new Quaternion(0f, 0f, 0f, 1f);

        // move to point 6
        api.moveTo(point, quaternion, false);

        // spot laser
        api.laserControl(true);

        // definition of ARmarker's id and corners
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();

        // get camera image (and save it)
        Mat image = api.getMatNavCam();

        // take target snapshot　(still don't work,,,)
        int id = 6;
        api.takeTargetSnapshot(id);

        // definition of dictionary about ARmarkers
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        // detect and draw ARmarker
        Aruco.detectMarkers(image, dictionary, corners, ids);
        Aruco.drawDetectedMarkers(image, corners, ids, new Scalar(0,255,0));

        // save image
        api.saveMatImage(image, "target_6.png");

        // undistort
        double[][] NavCamIntrinsics = api.getNavCamIntrinsics();
        double[] array_1 = NavCamIntrinsics[0];
        double[] array_2 = NavCamIntrinsics[1];
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distortion = new Mat(1, 5, CvType.CV_32FC1);
        cameraMat.put(0, 0, array_1);
        distortion.put(0, 0, array_2);

        Mat correct_image = new Mat();
        Calib3d.undistort(image, correct_image, cameraMat, distortion);
        api.saveMatImage(correct_image, "undistort_target_6.png");

    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }

}

// arataが編集したよ //