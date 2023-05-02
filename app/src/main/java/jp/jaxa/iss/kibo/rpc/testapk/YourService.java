package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
// Kibo-RPC library

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
        Aruco.detectMarkers(image, Aruco.DICT_5X5_250, corners, ids);
        Log. i(TAG, corners);
        Log. i(TAG, ids);

        
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