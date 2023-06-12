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
        long[][] times = new long[8][8];
        times[0][0] = 0;
        times[0][1] = 0; // never used
        times[0][2] = 0;
        times[0][3] = 0;
        times[0][4] = 42928;
        times[0][5] = 33928;
        times[0][6] = 25424;
        times[0][7] = 31480;
        times[1][0] = 20872;
        times[1][1] = 0;
        times[1][2] = 0; // never used
        times[1][3] = 44624;
        times[1][4] = 0;
        times[1][5] = 31016;
        times[1][6] = 0;
        times[1][7] = 0;
        times[2][0] = 0;
        times[2][1] = 0;
        times[2][2] = 44624;
        times[2][3] = 0; // never used
        times[2][4] = 0;
        times[2][5] = 0;
        times[2][6] = 0;
        times[2][7] = 0;
        times[3][0] = 0;
        times[3][1] = 42928;
        times[3][2] = 0;
        times[3][3] = 0;
        times[3][4] = 0; // never used
        times[3][5] = 29016;
        times[3][6] = 37976;
        times[3][7] = 0;
        times[4][0] = 35880;
        times[4][1] = 33928;
        times[4][2] = 31016;
        times[4][3] = 0;
        times[4][4] = 29016;
        times[4][5] = 0; // never used
        times[4][6] = 27304;
        times[4][7] = 0;
        times[5][0] = 29456;
        times[5][1] = 25424;
        times[5][2] = 0;
        times[5][3] = 0;
        times[5][4] = 37976;
        times[5][5] = 27304;
        times[5][6] = 0; // never used
        times[5][7] = 18152;
        times[6][0] = 0;
        times[6][1] = 31480;
        times[6][2] = 0;
        times[6][3] = 0;
        times[6][4] = 0;
        times[6][5] = 0;
        times[6][6] = 18152;
        times[6][7] = 0; // never used
        times[7][0] = 0;
        times[7][1] = 0;
        times[7][2] = 0;
        times[7][3] = 24008;
        times[7][4] = 18392;
        times[7][5] = 24152;
        times[7][6] = 32992;
        times[7][7] = 0;

        //points data
        int[] points = {30, 20, 40, 20, 30, 30};

        //start mission
        api.startMission();

        //get time
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long ActiveTimeRemaining = TimeRemaining.get(0);
        Long MissionTimeRemaining = TimeRemaining.get(1);

        //get active targets
        List<Integer> ActiveTargets = api.getActiveTargets();
        int NumberOfActiveTargets = ActiveTargets.size();
        int points1 = 0;
        int points2 = 0 ;
        points1 = points[(ActiveTargets.get(0)-1)];
        if(NumberOfActiveTargets == 2) {
            points2 = points[(ActiveTargets.get(1)-1)];
        }

        //initialize the varients
        long route1 = 0;
        long route2 = 0;

        //initialize current point
        int currentPoint = 0;
        String reportMessage = null;
        int QRcount = 0;

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


    public void moveAndShot(int p1, int p2, int p3){
        moveAndShot(p1, p2);
        moveAndShot(p2, p3);
    }

    public void moveAndShot(int from, int to){
        //get time
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long ActiveTimeRemaining  = TimeRemaining.get(0);
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
        Point viapoint03 = new Point();
        Point viapoint07 = new Point();

        Point viapoint12 = new Point();
        Point viapoint18 = new Point();

        Point viapoint23 = new Point(10.66512d, -8.3278d, 5d);
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

        long countStart = MissionTimeRemaining;
        Log.i(TAG, "-------------- LOG: initialTime=" + MissionTimeRemaining);

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
                break;
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
        ActiveTimeRemaining  = TimeRemaining.get(0);
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

    //trush

    public void trip(int p1, int p2){
        //timer
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long ActiveTimeRemaining  = TimeRemaining.get(0);
        Long MissionTimeRemaining = TimeRemaining.get(1);

        String reportMessage = null;

        moveAndShot(p1, p2);
        if (p2 == 7){
            reportMessage = ReadQR();
        }

        if(MissionTimeRemaining < 60000) {
            api.notifyGoingToGoal();
            moveAndShot(p2, 8);
            api.reportMissionCompletion(reportMessage);
        }
    }

    public void trip(int p1, int p2, int p3){
        //timer
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long ActiveTime  = TimeRemaining.get(0) / 1000;
        Long MissionTime = TimeRemaining.get(1) / 1000;

        //start
        String reportMessage = null;
        api.startMission();

        //move between targets
        moveAndShot(0, p1);
        if (p1 == 7){
            reportMessage = ReadQR();
        }

        moveAndShot(p1, p2);
        if (p2 == 7){
            reportMessage = ReadQR();
        }

        moveAndShot(p2, p3);
        if (p3 == 7){
            reportMessage = ReadQR();
        }

        // add if timer
        api.notifyGoingToGoal();
        moveAndShot(p3, 8);
        api.reportMissionCompletion(reportMessage);
    }

    public void trip(int p1, int p2, int p3, int p4){
        //timer
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long ActiveTime  = TimeRemaining.get(0) / 1000;
        Long MissionTime = TimeRemaining.get(1) / 1000;

        //start
        String reportMessage = null;
        api.startMission();

        //move between targets
        moveAndShot(0, p1);
        if (p1 == 7){
            reportMessage = ReadQR();
        }

        moveAndShot(p1, p2);
        if (p2 == 7){
            reportMessage = ReadQR();
        }

        moveAndShot(p2, p3);
        if (p3 == 7){
            reportMessage = ReadQR();
        }

        moveAndShot(p3, p4);
        if (p4 == 7){
            reportMessage = ReadQR();
        }

        // add if timer
        api.notifyGoingToGoal();
        moveAndShot(p4, 8);
        api.reportMissionCompletion(reportMessage);
    }

}

