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

        // move to target_6
        Point point_6 = new Point(11.355d, -8.989d, 4.8305d);
        Quaternion quaternion_6 = new Quaternion(0f, 0f, 0f, 1f);
        api.moveTo(point_6, quaternion_6, false);
        Mat image = image_correction(api.getMatNavCam());
        api.saveMatImage(image, "target_6.png");

        // move to target_1
        Point point_1 = new Point(11.2746d, -9.92284d, 5.2988d);
        Quaternion quaternion_1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(point_1, quaternion_1, false);
        image = image_correction(api.getMatNavCam());
        api.saveMatImage(image, "target_1.png");

        // move to taget_5
        Point point_5 = new Point(11.114d, -7.9756d, 5.3393d);
        Quaternion quaternion_5 = new Quaternion(-0.707f, 0f, 0f, 0.707f);
        api.moveTo(point_5, quaternion_5, false);
        image = image_correction(api.getMatNavCam());
        api.saveMatImage(image, "target_5.png");

        // move to taget_4
        Point point_4 = new Point(10.51d, -6.7185d, 5.1804d);
        Quaternion quaternion_4 = new Quaternion(1f, -1f, 0f, 0.707f);
        api.moveTo(point_4, quaternion_4, false);
        image = image_correction(api.getMatNavCam());
        api.saveMatImage(image, "target_4.png");

        // spot laser
        api.laserControl(true);

    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }

    // 6-1-5-4でとりあえず行く
    
    // 画像の歪み補正のメソッド
    public Mat image_correction(Mat image) {

        double[][] NavCamIntrinsics = api.getNavCamIntrinsics();
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distortion = new Mat(1, 5, CvType.CV_32FC1);
        cameraMat.put(0, 0, NavCamIntrinsics[0]);
        distortion.put(0, 0, NavCamIntrinsics[1]);

        Mat correct_image = new Mat();
        undistort(image, correct_image, cameraMat, distortion);
        api.saveMatImage(correct_image, "undistort_target.png");

        return correct_image;

    }

    // ARmarkarを認識して自己位置修正をするメソッド（開発中）
    public void self_positioning(Mat image) {

        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();

        Aruco.detectMarkers(image, dictionary, corners, ids);
        Aruco.drawDetectedMarkers(image, corners, ids);

        double sum_x = 0;
        double sum_y = 0;
        for (int i=0; i<4; i++) {
            sum_x += corners.get(0).get(0,i)[0];
            sum_y += corners.get(0).get(0,i)[1];
        }
        double ARcenter_x = sum_x / 4;
        double ARcenter_y = sum_y / 4;
        
    }

}

// arataが編集したよ //
