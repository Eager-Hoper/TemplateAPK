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

        // required time data
        //[end point-1][start point]
        long[][] times = new long[8][8];
        times[0][0] = 40880;
        times[0][1] = 0; // never used
        times[0][2] = 41776;
        times[0][3] = 50608;
        times[0][4] = 42928;
        times[0][5] = 33928;
        times[0][6] = 25424;
        times[0][7] = 31480;
        times[1][0] = 20992;
        times[1][1] = 41776;
        times[1][2] = 0; // never used
        times[1][3] = 44624;
        times[1][4] = 52856;
        times[1][5] = 31016;
        times[1][6] = 35312;
        times[1][7] = 38416;
        times[2][0] = 50136;
        times[2][1] = 50608;
        times[2][2] = 44624;
        times[2][3] = 0; // never used
        times[2][4] = 41944;
        times[2][5] = 36648;
        times[2][6] = 41272;
        times[2][7] = 40432;
        times[3][0] = 59544;
        times[3][1] = 42928;
        times[3][2] = 52856;
        times[3][3] = 42496;
        times[3][4] = 0; // never used
        times[3][5] = 29016;
        times[3][6] = 37976;
        times[3][7] = 50200;
        times[4][0] = 35880;
        times[4][1] = 33928;
        times[4][2] = 31016;
        times[4][3] = 36648;
        times[4][4] = 29016;
        times[4][5] = 0; // never used
        times[4][6] = 27336;
        times[4][7] = 38712;
        times[5][0] = 29504;
        times[5][1] = 25424;
        times[5][2] = 35304;
        times[5][3] = 41272;
        times[5][4] = 37976;
        times[5][5] = 27336;
        times[5][6] = 0; // never used
        times[5][7] = 18152;
        times[6][0] = 41080;
        times[6][1] = 31480;
        times[6][2] = 38416;
        times[6][3] = 40432;
        times[6][4] = 50200;
        times[6][5] = 38712;
        times[6][6] = 18200;
        times[6][7] = 0; // never used
        times[7][0] = 0; // never used
        times[7][1] = 55568;
        times[7][2] = 49312;
        times[7][3] = 24008;
        times[7][4] = 18392;
        times[7][5] = 24208;
        times[7][6] = 32992;
        times[7][7] = 43032;

        //points data
        int[] points = {30, 20, 40, 20, 30, 30};

        //define the varients
        long route1;
        long route2;

        //initialize current point
        int currentPoint = 0;
        String reportMessage = null;
        int QRcount = 0;

        //data end here
        //start mission
        api.startMission();

        //for viapoint test
        //moveAndShot(0, 3);
        //moveAndShot(3, 5);
        //moveAndShot(5, 6);
        //moveAndShot(6, 3);
        //reportMessage = ReadQR();
        //api.notifyGoingToGoal();
        //moveAndShot(3, 8);
        //api.reportMissionCompletion(reportMessage);
        //viapoint test end here

        //get time
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long ActiveTimeRemaining = TimeRemaining.get(0);
        Long MissionTimeRemaining = TimeRemaining.get(1);

        //get active targets
        List<Integer> ActiveTargets = api.getActiveTargets();
        int NumberOfActiveTargets = ActiveTargets.size();
        int points1;
        int points2 = 0 ;
        points1 = points[(ActiveTargets.get(0)-1)];
        if(NumberOfActiveTargets == 2) {
            points2 = points[(ActiveTargets.get(1)-1)];
        }

        //move between targets
        while (MissionTimeRemaining > 90000) {
            if (NumberOfActiveTargets == 1) {
                moveAndShot(currentPoint, ActiveTargets.get(0));
                if(ActiveTargets.get(0) == 6 && QRcount == 0){
                    moveAndShot(6, 7);
                    reportMessage = ReadQR();
                    QRcount ++;
                    currentPoint = 7;
                }else{
                    currentPoint = ActiveTargets.get(0);
                }
            } else {
                //search for the best route
                route1 = times[(ActiveTargets.get(0)-1)][currentPoint] + times[(ActiveTargets.get(1)-1)][ActiveTargets.get(0)];
                route2 = times[(ActiveTargets.get(1)-1)][currentPoint] + times[(ActiveTargets.get(0)-1)][ActiveTargets.get(1)];
                if (route1 >= route2 && route2 < ActiveTimeRemaining) {
                    moveAndShot(currentPoint, ActiveTargets.get(1));
                    if(ActiveTargets.get(1) == 6 && QRcount == 0){
                        TimeRemaining = api.getTimeRemaining();
                        ActiveTimeRemaining = TimeRemaining.get(0);
                        if(ActiveTimeRemaining > times[6][6] + times[(ActiveTargets.get(0)-1)][7]){
                            moveAndShot(6, 7);
                            reportMessage = ReadQR();
                            QRcount ++;
                            moveAndShot(7, ActiveTargets.get(0));
                        }else{
                            moveAndShot(ActiveTargets.get(1), ActiveTargets.get(0));
                        }
                    }else{
                        moveAndShot(ActiveTargets.get(1), ActiveTargets.get(0));
                    }
                    currentPoint = ActiveTargets.get(0);
                    if(ActiveTargets.get(0) == 6 && QRcount == 0){
                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount ++;
                        currentPoint = 7;
                    }
                }else if(route1 < route2 && route1 < ActiveTimeRemaining){
                    moveAndShot(currentPoint, ActiveTargets.get(0));
                    if(ActiveTargets.get(0) == 6 && QRcount == 0){
                        TimeRemaining = api.getTimeRemaining();
                        ActiveTimeRemaining = TimeRemaining.get(0);
                        if(ActiveTimeRemaining > times[6][6] + times[(ActiveTargets.get(1)-1)][7]){
                            moveAndShot(6, 7);
                            reportMessage = ReadQR();
                            QRcount ++;
                            moveAndShot(7, ActiveTargets.get(1));
                        }else{
                            moveAndShot(ActiveTargets.get(0), ActiveTargets.get(1));
                        }
                    }else{
                        moveAndShot(ActiveTargets.get(0), ActiveTargets.get(1));
                    }
                    currentPoint = ActiveTargets.get(1);
                    if(ActiveTargets.get(1) == 6 && QRcount == 0){
                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount ++;
                        currentPoint = 7;
                    }
                }else if(points1 > points2){
                    moveAndShot(currentPoint, ActiveTargets.get(0));
                    currentPoint = ActiveTargets.get(0);
                    if(ActiveTargets.get(0) == 6 && QRcount == 0){
                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount ++;
                        currentPoint = 7;
                    }
                }else if(points1 < points2){
                    moveAndShot(currentPoint, ActiveTargets.get(1));
                    currentPoint = ActiveTargets.get(1);
                    if(ActiveTargets.get(1) == 6 && QRcount == 0){
                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount ++;
                        currentPoint = 7;
                    }
                }else if(times[(ActiveTargets.get(0)-1)][currentPoint] > times[(ActiveTargets.get(1)-1)][currentPoint]){
                    moveAndShot(currentPoint, ActiveTargets.get(1));
                    currentPoint = ActiveTargets.get(1);
                    if(ActiveTargets.get(1) == 6 && QRcount == 0){
                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount ++;
                        currentPoint = 7;
                    }
                }else{
                    moveAndShot(currentPoint, ActiveTargets.get(0));
                    currentPoint = ActiveTargets.get(0);
                    if(ActiveTargets.get(0) == 6 && QRcount == 0){
                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount ++;
                        currentPoint = 7;
                    }
                }

            }

            //get next target
            ActiveTargets = api.getActiveTargets();
            NumberOfActiveTargets = ActiveTargets.size();
            points1 = points[(ActiveTargets.get(0)-1)];
            if(NumberOfActiveTargets == 2) {
                points2 = points[(ActiveTargets.get(1)-1)];
            }

            //get current time remaining
            TimeRemaining = api.getTimeRemaining();
            ActiveTimeRemaining  = TimeRemaining.get(0);
            MissionTimeRemaining = TimeRemaining.get(1);
        }

        api.notifyGoingToGoal();
        moveAndShot(currentPoint, 8);
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

    public void moveAndShot(int from, int to){
        //get time
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long MissionTimeRemaining = TimeRemaining.get(1);

        Point point1 = new Point(11.2053d, -9.92284d, 5.4736d);
        Point point2 = new Point(10.456184d, -9.196272d, 4.48d);
        Point point3 = new Point(10.7142d, -7.76727d, 4.48d);
        Point point4 = new Point(10.51d, -6.612872d, 5.20641d);
        Point point5 = new Point(11.0448d, -7.9193d, 5.3393d);
        Point point6 = new Point(11.355d, -9.0462d, 4.9416d);
        Point point7 = new Point(11.369d, -8.5518d, 4.48d);
        Point point8= new Point(11.143d, -6.7607d, 4.9654d);

        Quaternion quartanion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Quaternion quartanion2 = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
        Quaternion quartanion3 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quartanion4 = new Quaternion(0f, 0f, -1f, 0f);
        Quaternion quartanion5 = new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f);
        Quaternion quartanion6 = new Quaternion(0f, 0f, 0f, 1f);
        Quaternion quartanion7 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quartanion8 = new Quaternion(0f, 0f, -0.707f, 0.707f);

        //Viapoints
        Point viapoint01 = new Point(10.88628d , -9.9605d , 5.06316d);
        Point viapoint03 = new Point(10.5d, -8.3326d, 4.9425d);
        Point viapoint04 = new Point(10.4d, -8.3826d, 4.8995d);
        Point viapoint07 = new Point(11.1228d, -9.2334d, 4.388d);

        Point viapoint12 = new Point(11.2973d, -9.6929d, 5.0665d);
        Point viapoint13 = new Point(10.95975d, -8.2826d, 4.8988d);
        Point viapoint18 = new Point(11.2053d , -8.0635d , 4.87923d);

        Point viapoint23 = new Point(10.66512d, -8.3278d, 5d);
        Point viapoint24 = new Point(10.47268d , -8.40436d , 4.73903d);
        Point viapoint26 = new Point(10.90559d, -9.12124d, 4.86637d);
        Point viapoint27 = new Point(10.8652d , -8.50513d , 4.48d);
        Point viapoint28 = new Point(10.6795d , -8.40436d , 4.73903d);

        Point viapoint34 = new Point(10.6121d , -7.3049d , 4.9764d);
        Point viapoint35 = new Point(10.97867d , -7.63738d , 5.16743d);
        Point viapoint36 = new Point(10.95772d , -8.25329d , 4.74769d);
        Point viapoint37 = new Point(11.0416d , -8.3826d , 4.95651d);

        Point viapoint47 = new Point(10.9395d , -8.3826d , 4.89877d);

        Point viapoint57 = new Point(11.2069d , -8.28977d , 5.1305d);

        Point viapoint78 = new Point(11.256d, -8.3826d, 4.89877d);

        long countStart = MissionTimeRemaining;
        Log.i(TAG, "-------------- LOG: initialTime=" + MissionTimeRemaining);

        switch (from){
            case 0:
                switch(to){
                    case 1:
                        api.moveTo(viapoint01, quartanion1, true);
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
                        api.moveTo(viapoint04, quartanion4, true);
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
                break;
            case 1:
                switch(to){
                    case 2:
                        api.moveTo(viapoint12, quartanion1, true);
                        api.moveTo(point2, quartanion2, true);
                        break;
                    case 3:
                        api.moveTo(viapoint13, quartanion3, true);
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
                break;
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
                break;
            case 3:
                switch(to){
                    case 1:
                        api.moveTo(viapoint13, quartanion3, true);
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
                break;
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
                        api.moveTo(viapoint34, quartanion3, true);
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
                break;
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
                        api.moveTo(viapoint57, quartanion7, true);
                        api.moveTo(point7, quartanion7, true);
                        break;
                    case 8:
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
                break;
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
                break;
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
                        api.moveTo(viapoint37, quartanion7, true);
                        api.moveTo(point3, quartanion3, true);
                        break;
                    case 4:
                        api.moveTo(viapoint47, quartanion7, true);
                        api.moveTo(point4, quartanion4, true);
                        break;
                    case 5:
                        api.moveTo(viapoint57, quartanion5, true);
                        api.moveTo(point5, quartanion5, true);
                        break;
                    case 6:
                        api.moveTo(point6, quartanion6, true);
                        break;
                    case 8:
                        api.moveTo(viapoint78, quartanion7, true);
                        api.moveTo(point8, quartanion8, true);
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }

        if(to == 7 || to == 8){

        }else{
            api.laserControl(true);
            api.takeTargetSnapshot(to);
            api.laserControl(false);
        }

        //get time required (can be deleted)
        TimeRemaining = api.getTimeRemaining();
        Long ActiveTimeRemaining = TimeRemaining.get(0);
        MissionTimeRemaining = TimeRemaining.get(1);

        long countEnd = MissionTimeRemaining;
        long timeRequired = countStart - countEnd;
        Log.i(TAG, "-------------- LOG: timerequired" + from + to + "=" + timeRequired);
        Log.i(TAG, "-------------- LOG: ActiveTimeRemaining=" + ActiveTimeRemaining);

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
        QRCodeDetector decoder = new QRCodeDetector();
        String data = decoder.detectAndDecode(QRimage);

        //Generate png image for debug
        //api.saveMatImage(QRimage, "QR.png");

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

