package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
// Kibo-RPC library

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;
// astrobee library

import android.util.Log;
// android library

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import static org.opencv.android.Utils.matToBitmap;
// opencv library

import java.util.ArrayList;
import java.util.List;
// java library

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
        Log. i(TAG, "start mission");

        // the mission starts
        api.startMission();

        // move to point 6
        Point point = new Point(11.355d, -8.9929d, 4.7818d);
        Quaternion quaternion = new Quaternion(0f, 0f, 0f, 1f);

        api.moveTo(point, quaternion, false);

        // get camera image
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();
        Mat image = api.getMatNavCam();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        Aruco.detectMarkers(image, dictionary, corners, ids);

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