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

        //time data
        //[end point-1][start point]
        long[][] times = new long[8][8];
        times[0][0] = 40880;
        times[0][1] = 0; // never used
        times[0][2] = 40240;
        times[0][3] = 49440;
        times[0][4] = 42928;
        times[0][5] = 33928;
        times[0][6] = 25424;
        times[0][7] = 31480;
        times[1][0] = 20992;
        times[1][1] = 40240;
        times[1][2] = 0; // never used
        times[1][3] = 44624;
        times[1][4] = 52856;
        times[1][5] = 31016;
        times[1][6] = 36144;
        times[1][7] = 38416;
        times[2][0] = 48696;
        times[2][1] = 49440;
        times[2][2] = 44624;
        times[2][3] = 0; // never used
        times[2][4] = 41944;
        times[2][5] = 36648;
        times[2][6] = 41272;
        times[2][7] = 40432;
        times[3][0] = 59392;
        times[3][1] = 42928;
        times[3][2] = 52856;
        times[3][3] = 41944;
        times[3][4] = 0; // never used
        times[3][5] = 29016;
        times[3][6] = 37976;
        times[3][7] = 46288;
        times[4][0] = 35880;
        times[4][1] = 33928;
        times[4][2] = 31016;
        times[4][3] = 36648;
        times[4][4] = 29016;
        times[4][5] = 0; // never used
        times[4][6] = 27536;
        times[4][7] = 38712;
        times[5][0] = 29504;
        times[5][1] = 25424;
        times[5][2] = 36144;
        times[5][3] = 41272;
        times[5][4] = 37976;
        times[5][5] = 27536;
        times[5][6] = 0; // never used
        times[5][7] = 18280;
        times[6][0] = 41080;
        times[6][1] = 31480;
        times[6][2] = 38416;
        times[6][3] = 40432;
        times[6][4] = 46288;
        times[6][5] = 38712;
        times[6][6] = 18280;
        times[6][7] = 0; // never used
        times[7][0] = 0; // never used
        times[7][1] = 56568;
        times[7][2] = 50312;
        times[7][3] = 25008;
        times[7][4] = 19568;
        times[7][5] = 25448;
        times[7][6] = 33992;
        times[7][7] = 44032;

        //points data
        int[] points = {30, 20, 40, 20, 30, 30};

        //varients
        long route1;
        long route2;
        long routeToGoal1;
        long routeToGoal2;

        //initialize current status
        int currentPoint = 0;
        String reportMessage = null;
        int QRcount = 0;

        //start mission
        api.startMission();

        //for viapoint test
        //moveAndShot(0, 1);
        //reportMessage = ReadQR();
        //api.notifyGoingToGoal();
        //moveAndShot(1, 8);
        //api.reportMissionCompletion(reportMessage);

        //get time
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long ActiveTimeRemaining = TimeRemaining.get(0);
        Long MissionTimeRemaining = TimeRemaining.get(1);

        //get active targets
        List<Integer> ActiveTargets = new ArrayList<>();
        ActiveTargets.add(0);
        while(ActiveTargets.get(0) == 0) {
            ActiveTargets = api.getActiveTargets();
        }
        int NumberOfActiveTargets = ActiveTargets.size();
        int points1;
        int points2 = 0 ;
        points1 = points[(ActiveTargets.get(0)-1)];
        if(NumberOfActiveTargets == 2) {
            points2 = points[(ActiveTargets.get(1)-1)];
        }

        long currentToFirstTargetTime = times[(ActiveTargets.get(0)-1)][currentPoint];
        long currentToSecondTargetTime = 0;
        long FirstTargetToGoalTime = times[7][ActiveTargets.get(0)];
        long SecondTargetToGoalTime = 0;
        long FirstTargetToSecondTarget = 0;
        long SecondTargetToFirstTarget = 0;
        // TODO: check again
        if(NumberOfActiveTargets == 2) {
            SecondTargetToGoalTime = times[7][ActiveTargets.get(1)];
            currentToSecondTargetTime = times[(ActiveTargets.get(1)-1)][currentPoint];
            FirstTargetToSecondTarget = FirstTargetToSecondTarget;
            SecondTargetToFirstTarget = SecondTargetToFirstTarget;
        }

        //move
        while (MissionTimeRemaining > 0) {

            if (NumberOfActiveTargets == 1) {
                if ((currentToFirstTargetTime + FirstTargetToGoalTime) < MissionTimeRemaining) {
                    if((currentToFirstTargetTime + FirstTargetToGoalTime) < ActiveTimeRemaining) {

                        moveAndShot(currentPoint, ActiveTargets.get(0));
                        TimeRemaining = api.getTimeRemaining();
                        MissionTimeRemaining = TimeRemaining.get(1);

                        if (ActiveTargets.get(0) == 6 && QRcount == 0 && (times[6][ActiveTargets.get(0)] + times[7][7]) < MissionTimeRemaining) {

                            moveAndShot(6, 7);
                            reportMessage = ReadQR();
                            QRcount++;
                            currentPoint = 7;

                        } else {
                            currentPoint = ActiveTargets.get(0);
                        }
                    }
                }else{
                    if(times[6][currentPoint]+times[7][7] < MissionTimeRemaining && QRcount == 0){
                        moveAndShot(currentPoint, 7);
                        reportMessage = ReadQR();
                        QRcount++;
                        currentPoint = 7;
                    }
                    break;
                }

            } else {
                route1 = currentToFirstTargetTime + FirstTargetToSecondTarget;
                route2 = currentToSecondTargetTime + SecondTargetToFirstTarget;
                routeToGoal1 = route1 + SecondTargetToGoalTime;
                routeToGoal2 = route2 + FirstTargetToGoalTime;

                if (route1 >= route2 && routeToGoal2 < MissionTimeRemaining && route2 < ActiveTimeRemaining) {

                    moveAndShot(currentPoint, ActiveTargets.get(1));

                    if (ActiveTargets.get(1) == 6 && QRcount == 0) {

                        TimeRemaining = api.getTimeRemaining();
                        ActiveTimeRemaining = TimeRemaining.get(0);
                        MissionTimeRemaining = TimeRemaining.get(1);

                        if (MissionTimeRemaining> times[6][6] + times[(ActiveTargets.get(0) - 1)][7] + FirstTargetToGoalTime &&
                                ActiveTimeRemaining > times[6][6] + times[(ActiveTargets.get(0) - 1)][7]) {

                            moveAndShot(6, 7);
                            reportMessage = ReadQR();
                            QRcount++;
                            moveAndShot(7, ActiveTargets.get(0));

                        } else {
                            moveAndShot(ActiveTargets.get(1), ActiveTargets.get(0));
                        }
                    } else {
                        moveAndShot(ActiveTargets.get(1), ActiveTargets.get(0));
                    }

                    currentPoint = ActiveTargets.get(0);
                    TimeRemaining = api.getTimeRemaining();
                    MissionTimeRemaining = TimeRemaining.get(1);

                    if (ActiveTargets.get(0) == 6 && QRcount == 0 && (times[6][ActiveTargets.get(0)] + times[7][7]) < MissionTimeRemaining) {

                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount++;
                        currentPoint = 7;
                    }

                } else if (route1 < route2 && routeToGoal1 < MissionTimeRemaining && route1 < ActiveTimeRemaining) {

                    moveAndShot(currentPoint, ActiveTargets.get(0));

                    if (ActiveTargets.get(0) == 6 && QRcount == 0) {

                        TimeRemaining = api.getTimeRemaining();
                        ActiveTimeRemaining = TimeRemaining.get(0);
                        MissionTimeRemaining = TimeRemaining.get(1);

                        if (MissionTimeRemaining > times[6][6] + times[(ActiveTargets.get(1) - 1)][7] + SecondTargetToGoalTime &&
                                ActiveTimeRemaining > times[6][6] + times[(ActiveTargets.get(1) - 1)][7]) {

                            moveAndShot(6, 7);
                            reportMessage = ReadQR();
                            QRcount++;
                            moveAndShot(7, ActiveTargets.get(1));

                        } else {
                            moveAndShot(ActiveTargets.get(0), ActiveTargets.get(1));
                        }
                    } else {
                        moveAndShot(ActiveTargets.get(0), ActiveTargets.get(1));
                    }

                    currentPoint = ActiveTargets.get(1);
                    TimeRemaining = api.getTimeRemaining();
                    MissionTimeRemaining = TimeRemaining.get(1);

                    if (ActiveTargets.get(1) == 6 && QRcount == 0 && (times[6][ActiveTargets.get(1)] + times[7][7]) < MissionTimeRemaining) {

                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount++;
                        currentPoint = 7;
                    }

                } else if (points1 > points2 && times[ActiveTargets.get(0)-1][currentPoint] + FirstTargetToGoalTime < MissionTimeRemaining &&
                        times[ActiveTargets.get(0)-1][currentPoint] < ActiveTimeRemaining) {

                    moveAndShot(currentPoint, ActiveTargets.get(0));
                    currentPoint = ActiveTargets.get(0);

                    TimeRemaining = api.getTimeRemaining();
                    MissionTimeRemaining = TimeRemaining.get(1);

                    if (ActiveTargets.get(0) == 6 && QRcount == 0 && times[6][ActiveTargets.get(0)] + times[7][7] < MissionTimeRemaining) {

                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount++;
                        currentPoint = 7;
                    }

                } else if (points1 < points2 && times[ActiveTargets.get(1)-1][currentPoint] + SecondTargetToGoalTime < MissionTimeRemaining &&
                        times[ActiveTargets.get(1)-1][currentPoint] < ActiveTimeRemaining) {

                    moveAndShot(currentPoint, ActiveTargets.get(1));
                    currentPoint = ActiveTargets.get(1);

                    TimeRemaining = api.getTimeRemaining();
                    MissionTimeRemaining = TimeRemaining.get(1);

                    if (ActiveTargets.get(1) == 6 && QRcount == 0 && times[6][ActiveTargets.get(1)] + times[7][7] < MissionTimeRemaining) {

                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount++;
                        currentPoint = 7;
                    }

                } else if (currentToFirstTargetTime >= currentToSecondTargetTime &&
                        times[ActiveTargets.get(1)-1][currentPoint] + SecondTargetToGoalTime < MissionTimeRemaining &&
                        times[ActiveTargets.get(1)-1][currentPoint] < ActiveTimeRemaining) {

                    moveAndShot(currentPoint, ActiveTargets.get(1));
                    currentPoint = ActiveTargets.get(1);

                    TimeRemaining = api.getTimeRemaining();
                    MissionTimeRemaining = TimeRemaining.get(1);

                    if (ActiveTargets.get(1) == 6 && QRcount == 0 && times[6][ActiveTargets.get(1)] + times[7][7] < MissionTimeRemaining) {

                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount++;
                        currentPoint = 7;
                    }

                } else if (currentToFirstTargetTime < currentToSecondTargetTime &&
                        times[ActiveTargets.get(0)-1][currentPoint] + FirstTargetToGoalTime < MissionTimeRemaining &&
                        times[ActiveTargets.get(0)-1][currentPoint] < ActiveTimeRemaining) {

                    moveAndShot(currentPoint, ActiveTargets.get(0));
                    currentPoint = ActiveTargets.get(0);

                    TimeRemaining = api.getTimeRemaining();
                    MissionTimeRemaining = TimeRemaining.get(1);

                    if (ActiveTargets.get(0) == 6 && QRcount == 0 && times[6][ActiveTargets.get(0)] + times[7][7] < MissionTimeRemaining) {

                        moveAndShot(6, 7);
                        reportMessage = ReadQR();
                        QRcount++;
                        currentPoint = 7;
                    }

                }else{
                    if(times[6][currentPoint]+times[7][7] < MissionTimeRemaining && QRcount == 0){
                        moveAndShot(currentPoint, 7);
                        reportMessage = ReadQR();
                        QRcount++;
                        currentPoint = 7;
                    }
                    break;
                }

            }

            //get next target
            ActiveTargets = api.getActiveTargets();
            NumberOfActiveTargets = ActiveTargets.size();
            points1 = points[(ActiveTargets.get(0) - 1)];
            if (NumberOfActiveTargets == 2) {
                points2 = points[(ActiveTargets.get(1) - 1)];
            }

            //get current time remaining
            TimeRemaining = api.getTimeRemaining();
            ActiveTimeRemaining = TimeRemaining.get(0);
            MissionTimeRemaining = TimeRemaining.get(1);

            currentToFirstTargetTime = times[(ActiveTargets.get(0)-1)][currentPoint];
            FirstTargetToGoalTime = times[7][ActiveTargets.get(0)];
            SecondTargetToGoalTime = times[7][ActiveTargets.get(1)];

        }

        api.notifyGoingToGoal();
        moveAndShot(currentPoint, 8);
        Log.i(TAG, "-------------- LOG: QRcount=" + QRcount);
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
        //get time for count timeRequired (LOG)
        List<Long> TimeRemaining = api.getTimeRemaining();
        long countStart = TimeRemaining.get(1);

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
        Point viapoint03 = new Point(10.5d,-8.3326d,4.8025d);
        Point viapoint04 = new Point(10.51d,-8.3826d,4.7695d);
        Point viapoint07 = new Point(11.1228d, -9.2334d, 4.388d);

        Point viapoint12 = new Point(10.89137d, -9.61836d, 5.13316d);
        Point viapoint13 = new Point(10.82433d , -8.25068d , 4.74585d);
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

        Point viapoint47 = new Point(11.31976d , -8.44065d , 4.74025d);

        Point viapoint57 = new Point(11.2069d , -8.28977d , 5.1305d);

        Point viapoint78 = new Point(11.256d, -8.3826d, 4.89877d);

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
                        api.moveTo(viapoint35, quartanion5, true);
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

        //get timeRequired
        TimeRemaining = api.getTimeRemaining();
        long countEnd = TimeRemaining.get(1);
        long timeRequired = countStart - countEnd;
        long ActiveTimeRemaining = TimeRemaining.get(0);

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
        api.flashlightControlFront(0.05f);
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
