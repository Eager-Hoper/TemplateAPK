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
        Point point_6 = new Point(11.355d, -8.989d, 4.8305d);
        Quaternion quaternion_6 = new Quaternion(0f, 0f, 0f, 1f);
        api.moveTo(point_6, quaternion_6, true);
        
        // 画像からターゲット中心からの相対座標を取得
        double[] relative = getRelative(api.getMatNavCam());

        // 現在地から相対座標分修正
        Kinematics kinematics = api.getRobotKinematics();
        Point real_point = kinematics.getPosition();
        double dest_y = real_point.getY() - relative[0];
        double dest_z = real_point.getZ() - relative[1];
        Log.i(TAG, "arata: get dest_x,y");

        // 新座標を指定
        Point new_point = new Point(real_point.getX(), dest_y, dest_z);
        Log.i(TAG, "arata: get new_point");

        // 再移動
        api.moveTo(new_point, quaternion_6, true);
        // spot laser
        api.laserControl(true);
        api.saveMatImage(image_correction(api.getMatNavCam()), "target_6.png");
        api.laserControl(false);
        Log.i(TAG, "arata: moveTo new_point");

//        // spot laser
//        api.laserControl(true);

        // move to target_1
        // Point point_1 = new Point(11.2746d, -9.92284d, 5.2988d);
        // Quaternion quaternion_1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        // api.moveTo(point_1, quaternion_1, true);
        // image = image_correction(api.getMatNavCam());
        // api.saveMatImage(image, "target_1.png");

        // move to taget_5
        // Point point_5 = new Point(11.114d, -7.9756d, 5.3393d);
        // Quaternion quaternion_5 = new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f);
        // api.moveTo(point_5, quaternion_5, true);
        // image = image_correction(api.getMatNavCam());
        // api.saveMatImage(image, "target_5.png");

        // move to taget_4
        // Point point_4 = new Point(10.51d, -6.7185d, 5.1804d);
        // Quaternion quaternion_4 = new Quaternion(0f, 0f, -1f, 0f);
        // api.moveTo(point_4, quaternion_4, true);
        // image = image_correction(api.getMatNavCam());
        // api.saveMatImage(image, "target_4.png");

        // Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        // Mat list_ids = new Mat();
        // List<Mat> corners = new ArrayList<>();

        // Aruco.detectMarkers(image, dictionary, corners, list_ids);
        // Aruco.drawDetectedMarkers(image, corners, list_ids);

        
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
    
    // 画像の歪み補正のメソッド 撮影した画像（Mat形式）を入力、歪み補正した画像（Mat形式）を出力
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

    // 画像を入力して、ターゲットの中心から現在地の誤差（double[2]={rx,ry}）を出力 ※画像内座標であることに注意
    public double[] getRelative (Mat image) {

        // 画像処理を行う
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Mat list_ids = new Mat();
        List<Mat> corners = new ArrayList<>();
        Aruco.detectMarkers(image_correction(image), dictionary, corners, list_ids);

        // AR_infoを取得
        double[][] AR_info = getAR_info(corners, list_ids);

        // ターゲットの中心座標を取得
        double[] target_center = getTargetCenter(AR_info, corners);

        // 画像内での中有心からターゲットの中心までの差分（doubleで宣言しているが縮尺はpixel）を取得
        double[] relative = new double[2];
        relative[0] = 640 - target_center[0];
        relative[1] = 480 - target_center[1];

        Log.i(TAG, "arata: get relative");

        // 縮尺をmeterに変換
        double scale = getScale(corners);
        relative[0] = relative[0] / scale;
        relative[1] = relative[1] / scale;

        Log.i(TAG, "arata: get relative in picture");
        return relative;

    }

    // AR_markerの四隅(corners)を入力して、AR_markerの中心座標（画像内座標）を出力
    public double[] getAR_center (int n, List<Mat> corners) {

        Log.i(TAG, "arata: move to getAR_center");

        double[][] AR_corners =
        {
            {(int) corners.get(n).get(0,0)[0], (int) corners.get(n).get(0,0)[1]}, // 左上
            {(int) corners.get(n).get(0,1)[0], (int) corners.get(n).get(0,1)[1]}, // 右上
            {(int) corners.get(n).get(0,2)[0], (int) corners.get(n).get(0,2)[1]}, // 右下
            {(int) corners.get(n).get(0,3)[0], (int) corners.get(n).get(0,3)[1]}, // 左下
        };

        Log.i(TAG, "arata: get AR_corner");

        double sum_x = 0;
        double sum_y = 0;
        for (int i=0; i<4; i++) {
            sum_x += AR_corners[i][0];
            sum_y += AR_corners[i][1];
        }

        Log.i(TAG, "arata: get sum_x and sum_y");

        double[] AR_center = new double[2];
        AR_center[0] = sum_x / 4;
        AR_center[1] = sum_y / 4;

        Log.i(TAG, "arata: get AR_center");
        return AR_center;

    }    

    // 画像を入力して、AR_markerの情報(AR_info)を出力
    // AR_infoについて　　1列目：AR_markerのID番号　2,3列目：AR_markerの中心座標（x,y）
    public double[][] getAR_info (List<Mat> corners, Mat list_ids) {

        // AR_info作成パート
        int n = corners.size(); // 画像中にあるAR_markerの数
        double[][] AR_info = new double[n][3]; // AR_infoを宣言

        Log.i(TAG, "arata: declare AR_info");
        
        for (int i=0; i<n; i++) {

            AR_info[i][0] = list_ids.get(i,0)[0]; // i行1列にID番号を代入

            double[] AR_center = new double[2];
            AR_center = getAR_center(i, corners); // AR_markerの中心座標を取得
            AR_info[i][1] = AR_center[0]; // i行2列にi個目のAR_marker中心x座標を代入
            AR_info[i][2] = AR_center[1]; // i行3列にi個目のAR_marker中心y座標を代入

            Log.i(TAG, "arata: substitute to AR_corner");

        }

        Log.i(TAG, "arata: get AR_info");
        return AR_info;

    }

    // 各AR＿markerのIDと中心座標の情報(AR_info)を入力して、ターゲットの中心座標(target_center)を求める
    public double[] getTargetCenter (double[][] AR_info, List<Mat> corners) {

        int n = corners.size();

        for (int i=0; i<n; i++) {
            double ID = AR_info[i][0];

            // IDを4で割った余りが1ならx方向に-10cm分、y方向に+3.75cm分移動
            if (ID%4 == 1) { 
                AR_info[i][1] += -0.1 * getScale(corners);
                AR_info[i][2] += 0.0375 * getScale(corners);

            // IDを4で割った余りが2ならx方向に+10cm分、y方向に+3.75cm分移動
            } else if (ID%4 == 2) {
                AR_info[i][1] += 0.1 * getScale(corners);
                AR_info[i][2] += 0.0375 * getScale(corners);

            // IDを4で割った余りが3ならx方向に+10cm分、y方向に-3.75cm分移動
            }else if (ID%4 == 3) {
                AR_info[i][1] += 0.1 * getScale(corners);
                AR_info[i][2] += -0.0375 * getScale(corners);

            // IDを4で割った余りが0ならx方向に-10cm分、y方向に-3.75cm分移動
            } else if (ID%4 == 0) {
                AR_info[i][1] += -0.1 * getScale(corners);
                AR_info[i][2] += -0.0375 * getScale(corners);

            } else {
                Log.i(TAG, "arata: can't caluculate target_center");
            }

        }

        Log.i(TAG, "arata: calculate AR_info");

        double target_x = 0;
        double target_y = 0;
        
        for (int i=0; i<n; i++) {
            target_x += AR_info[i][1];
            target_y += AR_info[i][2];
        }
        target_x = target_x / n;
        target_y = target_y / n;

        Log.i(TAG, "arata: get target_x,y");

        double[] target_center = new double[2];
        target_center[0] = target_x;
        target_center[1] = target_y;

        Log.i(TAG, "arata: caluculate target_center");
        return target_center; // 最終的に各AR_markerから推測したターゲットの中心座標の平均値を求めた

    }

    // 画像の縮尺を求める（pixel/meter)
    public double getScale (List<Mat> corners) {

        double[][] AR_corners =
        {
            {(int) corners.get(0).get(0,0)[0], (int) corners.get(0).get(0,0)[1]}, // 左上
            {(int) corners.get(0).get(0,1)[0], (int) corners.get(0).get(0,1)[1]}, // 右上
            {(int) corners.get(0).get(0,2)[0], (int) corners.get(0).get(0,2)[1]}, // 右下
            {(int) corners.get(0).get(0,3)[0], (int) corners.get(0).get(0,3)[1]}, // 左下
        };

        double side_length = ((AR_corners[1][0]-AR_corners[0][0])+(AR_corners[2][1]-AR_corners[1][1])+
        (AR_corners[2][0]-AR_corners[3][0])+(AR_corners[3][1]-AR_corners[0][1])) / 4;

        double scale = side_length / 0.05;

        Log.i(TAG, "ararta: get Scale");
        return scale;

    }
}

// arataが編集したよ //
