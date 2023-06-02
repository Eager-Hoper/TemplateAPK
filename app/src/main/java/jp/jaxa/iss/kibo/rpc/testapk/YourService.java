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
import static org.opencv.android.Utils.matToBitmap;
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
