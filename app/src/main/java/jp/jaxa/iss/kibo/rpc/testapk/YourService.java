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
import java.util.Arrays;
import java.util.List;
import java.lang.Math;
import java.util.concurrent.TimeoutException;
// java library (for basic operate)

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    // setting for log
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
        //create random turbulence 0~10cm
        double TurbX = new java.util.Random().nextInt(11) / 100d;
        double TurbY = new java.util.Random().nextInt(11) / 100d;
        double TurbZ = new java.util.Random().nextInt(11) / 100d;

        boolean MinusX = new java.util.Random().nextBoolean();
        boolean MinusY = new java.util.Random().nextBoolean();
        boolean MinusZ = new java.util.Random().nextBoolean();

        if (MinusX) {
            TurbX = -TurbX;
        }
        if (MinusY) {
            TurbY = -TurbY;
        }
        if (MinusZ) {
            TurbZ = -TurbZ;
        }
        Point point1 = new Point(11.2053d, -9.87284d, 5.4736d);
        Point point2 = new Point(10.456184d, -9.196272d, 4.53d);
        Point point3 = new Point(10.7142d+TurbX, -7.76727d+TurbY, 4.53d+TurbZ);
        Point point4 = new Point(10.56d+TurbX, -6.612872d+TurbY, 5.20641d+TurbZ);
        Point point7 = new Point(11.369d+TurbX, -8.5518d+TurbY, 4.48d+TurbZ);
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

        Point viapoint47 = new Point(10.628d, -8.841d, 5.288d);
        //Point viapoint47 = new Point(10.64904d, -8.22217d, 5.29178d);

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

        List<Long> TimeRemaining;
        Long Countstart;;
        Long Countend;

        // move to target_4
        api.moveTo(viapoint04, quartanion4, true);
        api.moveTo(point4, quartanion4, true);

        api.laserControl(true);
        api.takeTargetSnapshot(4);
        api.laserControl(false);

        TimeRemaining = api.getTimeRemaining();
        Countstart = TimeRemaining.get(1);

        // move to target_7
        api.moveTo(viapoint47, quartanion4, true);
        api.moveTo(point7, quartanion7, true);

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

        TimeRemaining = api.getTimeRemaining();
        Countend = TimeRemaining.get(1);

        Log.i(TAG, "-------------- LOG: TimeRequired47=" + (Countstart - Countend));

        TimeRemaining = api.getTimeRemaining();
        Countstart = TimeRemaining.get(1);

        api.moveTo(viapoint78, quartanion3, true);
        api.moveTo(viapoint13, quartanion3, true);
        api.moveTo(point3, quartanion3, true);

        reMove_AR_moveTo(3);

        api.laserControl(true);
        api.takeTargetSnapshot(3);
        api.laserControl(false);

        TimeRemaining = api.getTimeRemaining();
        Countend = TimeRemaining.get(1);

        Log.i(TAG, "-------------- LOG: TimeRequired73=" + (Countstart - Countend));

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

    public void reMove_AR_moveTo (int to){

        /*　api.moveTo()を用いてターゲット前での自己位置修正を行うメソッド
         *
         * @param
         * to：目的地のターゲット番号
         *
         * @return
         * void：自己位置修正を行う
         */

        Quaternion quaternion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Quaternion quaternion2 = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
        Quaternion quaternion3 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quaternion4 = new Quaternion(0f, 0f, -1f, 0f);


        // ターゲットまでの相対位置を取得
        double[] relative = getRelative(to);
        Kinematics kinematics = api.getRobotKinematics();
        Point current_point = kinematics.getPosition();

        /*　目的地のターゲット番号によってx,y,z座標の修正を場合分けしている
         *　また、位置修正時に5cmターゲット方向に前進する（最小移動距離確保のため）
         */
        switch (to) {
            case 1:

                double dest_x1 = current_point.getX() + relative[0];
                double dest_z1 = current_point.getZ() + relative[1];
                Point new_point1 = new Point(dest_x1, current_point.getY()-0.05, dest_z1);
                api.moveTo(new_point1, quaternion1, true);
                break;

            case 2:
                double dest_x2 = current_point.getX() + relative[0];
                double dest_y2 = current_point.getY() - relative[1];
                Point new_point2 = new Point(dest_x2, dest_y2, current_point.getZ()-0.05);
                api.moveTo(new_point2, quaternion2, true);
                break;

            case 3:
                double dest_y3 = current_point.getY() + relative[0];
                double dest_x3 = current_point.getX() + relative[1];
                Point new_point3 = new Point(dest_x3, dest_y3, current_point.getZ()-0.05);
                api.moveTo(new_point3, quaternion3, true);
                break;

            case 4:
                double dest_y4 = current_point.getY() - relative[0];
                double dest_z4 = current_point.getZ() + relative[1];
                Point new_point4 = new Point(current_point.getX()-0.05, dest_y4, dest_z4);
                api.moveTo(new_point4, quaternion4, true);
                break;

            default:
                break;
        }

        }

    public double[] getRelative(int to) {

        /*　ARmarkerを認識してターゲット中心にレーザーが当たるような相対移動座標を計算するメソッド
         *
         * @param
         * to：目的地のターゲット番号
         *
         * @return
         * relative：現在地からターゲット中心までの移動座標
         * 　relative[0]：画像内x方向を正とする相対座標
         * 　relative[1]：画像内y方向を正とする相対座標
         */

        // ARmarkerを検出　(@see AR_detect)
        List<List<Mat>> AR_info = AR_detect(to);
        Mat ids = AR_info.get(0).get(0);
        List<Mat> corners = AR_info.get(1);

        // ターゲット中心座標を計算　(@see getTargetCneter
        double[] target_center = getTargetCenter(ids, corners);

        /*　astrobeeが修正する相対座標へ変換
         *
         * 以下は各項の説明
         * (target_center[] - ¥¥¥)：画像の中心からの相対座標を計算
         * (target_center[] - ¥¥¥) / getScale()：縮尺をpixelからmeterに変更
         * (target_center[] - ¥¥¥) / getScale() ± ¥¥¥：NavCam搭載位置とレーザー搭載位置の差分を修正
         */
        double relative[] =
                {   ((target_center[0] - 640) / getScale(corners)) - 0.0994,
                        ((target_center[1] - 480) / getScale(corners)) + 0.0285   };

        return relative;


    }

    public List<List<Mat>> AR_detect(int to) {

        /*　astrobeeの撮影した画像からARmarkerを検出する
         *
         * @param
         * to：目的地のターゲット番号
         *
         * @return
         * AR_info：ARmarkerのIDリスト（list_ids）と四隅座標のリスト（corners）を格納する
         * List<List<Mat>> AR_info = {List<Mat> list_ids, List<Mat> corners}
         */

        // ARmarkerのdictionaryは5*5でIDが250番まであるものを使用
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        // astrobeeカメラ画像を取得
        Mat image = api.getMatNavCam();

        // 取得画像からARmarkerを検出
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();
        Aruco.detectMarkers(image_correction(image), dictionary, corners, ids);

        /*　ARmarkerの検出に失敗した場合の処理
         *
         * @throws
         * corners == null || list_ids == null：以下の二つの原因でARmarker関連情報を取得できない場合
         * 原因1, Aruco.detectMarkers()が画像を認識できずreturnがnull
         * 原因2, astrobee取得画像内にARmarkerが存在しない
         */

        /*　原因1に対する対処
         * Aruco.detectMarkers()を3回繰り返す
         */
        int loopCounter = 0;
        int LOOP_MAX = 3;
        while((corners == null || ids == null) && loopCounter < LOOP_MAX) {
            Aruco.detectMarkers(image_correction(image), dictionary, corners, ids);
            loopCounter++;
        }

        /*　原因2に対する対処
         * ターゲット向かって後方に10cm移動して再度Aruco.detectMarkers()を3回まで繰り返す
         *
         * TODO:後方に10cmではなくpivotPointへの移動でも良いのでは
         */
        if (corners == null || ids == null) {
            Kinematics error_kinematics = api.getRobotKinematics();
            Point error_point = error_kinematics.getPosition();
            Quaternion quaternion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
            Quaternion quaternion2 = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
            Quaternion quaternion3 = new Quaternion(0f, 0.707f, 0f, 0.707f);
            Quaternion quaternion4 = new Quaternion(0f, 0f, -1f, 0f);

            switch (to) {
                case 1:
                    Point debug_point1 = new Point(error_point.getX(), error_point.getY()+0.10, error_point.getZ());
                    api.moveTo(debug_point1, quaternion1, true);

                case 2:
                    Point debug_point2 = new Point(error_point.getX(), error_point.getY(), error_point.getZ()+0.10);
                    api.moveTo(debug_point2, quaternion2, true);

                case 3:
                    Point debug_point3 = new Point(error_point.getX(), error_point.getY(), error_point.getZ()+0.10);
                    api.moveTo(debug_point3, quaternion3, true);

                case 4:
                    Point debug_point4 = new Point(error_point.getX()+0.10, error_point.getY(), error_point.getZ());
                    api.moveTo(debug_point4, quaternion4, true);
            }

            while((corners == null || ids == null) && loopCounter < LOOP_MAX) {
                Aruco.detectMarkers(image_correction(image), dictionary, corners, ids);
                loopCounter++;
            }

        }

        // return用に変数型を揃える
        List<Mat> list_ids = new ArrayList<Mat>(Arrays.asList(ids));
        List<List<Mat>> AR_info = new ArrayList<List<Mat>>(Arrays.asList(list_ids, corners));

        return AR_info;

    }

    public double[] getTargetCenter (Mat list_ids, List < Mat > corners){

        /*　astrobeeの撮影した画像でのターゲットの中心座標を計算するメソッド
         *
         * @param
         * list_ids：AR_detectで検出したARmarkerのID
         * corners：AR_detectで検出したARmarkerの四隅の座標
         *
         * @return
         * target_center：画像でのターゲットの中心座標　(x,y)=(target_center[0],target_center[1])
         */

        // ARmarkerの数をnに代入
        int n = corners.size();

        /*　画像が傾いていてもターゲット中心を計算できるように傾きを表す三角比を設定
         *
         * @param
         * a：斜辺
         * b：短辺（x方向）
         * c：長辺（y方向）
         * sin：傾きの正弦
         * cos：傾きの余弦
         */

        double a = Math.sqrt(Math.pow(corners.get(0).get(0, 3)[1] - corners.get(0).get(0, 0)[1], 2) + Math.pow(corners.get(0).get(0, 0)[0] - corners.get(0).get(0, 3)[0], 2));
        double b = corners.get(0).get(0, 0)[0] - corners.get(0).get(0, 3)[0];
        double c = corners.get(0).get(0, 3)[1] - corners.get(0).get(0, 0)[1];
        double sin = b / a;
        double cos = c / a;

        /*　ARmarkerのIDを4で割った時の余りによってターゲット中心までの平行移動値を変更する
         *
         * (±0.0125 * sin)：ターゲット中心のx方向への平行移動
         * (±0.075  * cos)：ターゲット中心のy方向への平行移動
         */

        double[][] center_cand = new double[n][2];
        double scale = getScale(corners);

        for (int i = 0; i < n; i++) {
            double ID = list_ids.get(i, 0)[0];

            // if ID≡1(mod4) UR, X=x-10[cm] and Y=y+3.75[cm]
            if (ID % 4 == 1) {
                center_cand[i][0] = corners.get(i).get(0, 3)[0] + (-0.0125 * sin - 0.075 * cos) * scale;
                center_cand[i][1] = corners.get(i).get(0, 3)[1] + (0.0125 * cos - 0.075 * sin) * scale;

                // if ID≡2(mod4) UL, X=x+10[cm] and Y=y+3.75[cm]
            } else if (ID % 4 == 2) {
                center_cand[i][0] = corners.get(i).get(0, 2)[0] + (-0.0125 * sin + 0.075 * cos) * scale;
                center_cand[i][1] = corners.get(i).get(0, 2)[1] + (0.0125 * cos + 0.075 * sin) * scale;

                // if ID≡3(mod4) BL, X=x+10[cm] and Y=y-3.75[cm]
            } else if (ID % 4 == 3) {
                center_cand[i][0] = corners.get(i).get(0, 1)[0] + (0.0125 * sin + 0.075 * cos) * scale;
                center_cand[i][1] = corners.get(i).get(0, 1)[1] + (-0.0125 * cos + 0.075 * sin) * scale;

                // if ID≡0(mod4) BR, X=x-10[cm] and Y=y-3.75[cm]
            } else if (ID % 4 == 0) {
                center_cand[i][0] = corners.get(i).get(0, 0)[0] + (0.0125 * sin - 0.075 * cos) * scale;
                center_cand[i][1] = corners.get(i).get(0, 0)[1] + (-0.0125 * cos - 0.075 * sin) * scale;

            } else {

            }

        }

        /*　各ARmarkerの四隅座標でのターゲット中心座標を求めたら、その平均値をtarget_centerに代入する */
        double target_x = 0;
        double target_y = 0;
        for (int i = 0; i < n; i++) {
            target_x += center_cand[i][0];
            target_y += center_cand[i][1];
        }
        double[] target_center = {target_x / n, target_y / n};

        return target_center;

    }

    public double getScale (List<Mat> corners) {

        /*　astrobeeの撮影した画像内座標とISS内座標の縮尺を計算するメソッド
         *
         * @param
         * corners：AR_detectで検出したARmarkerの四隅の座標
         *
         * @return
         * scale：ISS内座標から画像内座標への縮尺 単位は[pixel/meter]
         */

        // cornersを扱いやすい変数型に変更
        double[][] AR_corners =
                {
                        {(int) corners.get(0).get(0, 0)[0], (int) corners.get(0).get(0, 0)[1]}, // UL
                        {(int) corners.get(0).get(0, 1)[0], (int) corners.get(0).get(0, 1)[1]}, // UR
                        {(int) corners.get(0).get(0, 2)[0], (int) corners.get(0).get(0, 2)[1]}, // BR
                        {(int) corners.get(0).get(0, 3)[0], (int) corners.get(0).get(0, 3)[1]}, // BL
                };

        // ARmarkerの４辺の長さを足し上げる　
        // @param  side_length：ARmarkerの周長
        double side_length = 0;
        for (int i = 0; i < 4; i++) {
            if (i < 3) {
                side_length += Math.sqrt(Math.pow(AR_corners[i + 1][0] - AR_corners[i][0], 2) + Math.pow(AR_corners[i + 1][1] - AR_corners[i][1], 2));
            } else if (i == 3) {
                side_length += Math.sqrt(Math.pow(AR_corners[0][0] - AR_corners[i][0], 2) + Math.pow(AR_corners[0][1] - AR_corners[i][1], 2));
            }
        }

        // 周長が20cm（5cm * 4辺）であることから縮尺を計算
        double scale = side_length / (4 * 0.05);

        return scale;

    }
}
