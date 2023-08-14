package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
// Kibo-RPC library

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
// astrobee library (for definition of Point and Quaternion etc.)

import android.util.Log;
// android library (for log)

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;
import static org.opencv.imgproc.Imgproc.*;
// opencv library (for detect ARmarkers)

import java.util.ArrayList;
import java.util.Arrays;
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
        times[0][0] = 54368;
        times[0][1] = 0;
        times[0][2] = 52136;
        times[0][3] = 62240;
        times[0][4] = 70832;
        times[0][7] = 31704;
        times[1][0] = 35616;
        times[1][1] = 52592;
        times[1][2] = 0;
        times[1][3] = 57256;
        times[1][4] = 62640;
        times[1][7] = 56024;
        times[2][0] = 73672;
        times[2][1] = 60648;
        times[2][2] = 57256;
        times[2][3] = 0;
        times[2][4] = 54648;
        times[2][7] = 75824;
        times[3][0] = 70064;
        times[3][1] = 70288;
        times[3][2] = 62640;
        times[3][3] = 54648;
        times[3][4] = 0;
        times[3][7] = 90960;
        times[6][0] = 41080;
        times[6][1] = 31656;
        times[6][2] = 51064;
        times[6][3] = 58008;
        times[6][4] = 46288;
        times[6][7] = 0;
        times[7][0] = 0;
        times[7][1] = 58864;
        times[7][2] = 55744;
        times[7][3] = 27376;
        times[7][4] = 23504;
        times[7][7] = 86304;

        //points data
        int[] points = {30, 20, 40, 20};

        //varients
        long route1;
        long route2;
        long routeToGoal1;
        long routeToGoal2;

        //initialize current status
        int currentPoint = 0;
        String reportMessage = "empty";
        boolean QRflag = false;

        //start mission
        api.startMission();

        //get time
        List<Long> TimeRemaining = api.getTimeRemaining();
        Long MissionTimeRemaining = TimeRemaining.get(1);
        Long ActiveTimeRemaining = TimeRemaining.get(0);

        //get active targets
        List<Integer> ActiveTargets = new ArrayList<>();
        ActiveTargets.add(0);
        while (ActiveTargets.get(0) == 0) {
            ActiveTargets = api.getActiveTargets();
        }
        int NumberOfActiveTargets = ActiveTargets.size();
        int points1;
        int points2 = 0;
        points1 = points[(ActiveTargets.get(0) - 1)];
        if (NumberOfActiveTargets == 2) {
            points2 = points[(ActiveTargets.get(1) - 1)];
        }

        long currentToFirstTargetTime = times[(ActiveTargets.get(0) - 1)][currentPoint];
        long currentToSecondTargetTime = 0;
        long FirstTargetToGoalTime = times[7][ActiveTargets.get(0)];
        long SecondTargetToGoalTime = 0;
        long FirstTargetToSecondTarget = 0;
        long SecondTargetToFirstTarget = 0;
        if (NumberOfActiveTargets == 2) {
            SecondTargetToGoalTime = times[7][ActiveTargets.get(1)];
            currentToSecondTargetTime = times[(ActiveTargets.get(1) - 1)][currentPoint];
            FirstTargetToSecondTarget = times[(ActiveTargets.get(1) - 1)][ActiveTargets.get(0)];
            SecondTargetToFirstTarget = times[(ActiveTargets.get(0) - 1)][ActiveTargets.get(1)];
        }

        int numberOfPhotos = 1;
        boolean stayingCondition = false;

        //Action Start
        while (MissionTimeRemaining > 0) {

            //if Astrobee is staying, skip the moving motion. In this case, there is always one Active Target remaining.
            if(stayingCondition){
                Log.i(TAG, "-------------- LOG: Staying");
                //if there is no time to stay, go to the goal immediately
                if(!checkMissionTime(ActiveTimeRemaining+times[7][currentPoint])){
                    //check whether we can take QR at last
                    if (checkMissionTime(times[6][currentPoint] + times[7][7]) && !QRflag) {
                        moveAndShot(currentPoint, 7, numberOfPhotos);
                        reportMessage = ReadQR();
                        QRflag = true;
                        currentPoint = 7;
                    }
                    break;
                }

                //if there is no possibility to reach another point after staying, go to the goal
                long minimumTimeRequired;
                if(currentPoint == 1){
                    minimumTimeRequired = times[1][1];
                }else{
                    minimumTimeRequired = times[0][currentPoint];
                }
                for (int i = 1; i < 4; i++) {
                    //the last condition is necessary for the case Astrobee is at point7
                    if (times[i][currentPoint] < minimumTimeRequired && !(times[i][currentPoint] == 0) && !((i+1) == ActiveTargets.get(0))){
                        minimumTimeRequired = times[i][currentPoint];
                    }
                }
                Log.i(TAG, "-------------- LOG: minimumtime=" + minimumTimeRequired + ", currentpoint is" + currentPoint + "Active target is" + ActiveTargets.get(0));
                if(!(checkMissionTime(ActiveTimeRemaining+minimumTimeRequired))){
                    if (checkMissionTime(times[6][currentPoint] + times[7][7]) && !QRflag) {

                        moveAndShot(currentPoint, 7, numberOfPhotos);
                        reportMessage = ReadQR();
                        QRflag = true;
                        currentPoint = 7;

                    }
                    break;
                }

                if (checkMissionTime(times[6][currentPoint] + times[7][7]) && !QRflag) {

                    moveAndShot(currentPoint, 7, numberOfPhotos);
                    reportMessage = ReadQR();
                    QRflag = true;
                    currentPoint = 7;

                    stayingCondition = false;

                    //get info
                    ActiveTargets = api.getActiveTargets();
                    NumberOfActiveTargets = ActiveTargets.size();
                    points1 = points[(ActiveTargets.get(0) - 1)];

                    TimeRemaining = api.getTimeRemaining();
                    MissionTimeRemaining = TimeRemaining.get(1);

                    currentToFirstTargetTime = times[(ActiveTargets.get(0) - 1)][currentPoint];
                    FirstTargetToGoalTime = times[7][ActiveTargets.get(0)];

                    if (NumberOfActiveTargets == 2) {
                        points2 = points[(ActiveTargets.get(1) - 1)];
                        currentToSecondTargetTime = times[(ActiveTargets.get(1) - 1)][currentPoint];
                        SecondTargetToGoalTime = times[7][ActiveTargets.get(1)];
                        FirstTargetToSecondTarget = times[(ActiveTargets.get(1) - 1)][ActiveTargets.get(0)];
                        SecondTargetToFirstTarget = times[(ActiveTargets.get(0) - 1)][ActiveTargets.get(1)];
                    }

                }

                //when new targets appeared
                if(!(ActiveTargets.get(0) == api.getActiveTargets().get(0))){

                    stayingCondition = false;

                    //get info
                    ActiveTargets = api.getActiveTargets();
                    NumberOfActiveTargets = ActiveTargets.size();
                    points1 = points[(ActiveTargets.get(0) - 1)];

                    TimeRemaining = api.getTimeRemaining();
                    MissionTimeRemaining = TimeRemaining.get(1);

                    currentToFirstTargetTime = times[(ActiveTargets.get(0) - 1)][currentPoint];
                    FirstTargetToGoalTime = times[7][ActiveTargets.get(0)];

                    if (NumberOfActiveTargets == 2) {
                        points2 = points[(ActiveTargets.get(1) - 1)];
                        currentToSecondTargetTime = times[(ActiveTargets.get(1) - 1)][currentPoint];
                        SecondTargetToGoalTime = times[7][ActiveTargets.get(1)];
                        FirstTargetToSecondTarget = times[(ActiveTargets.get(1) - 1)][ActiveTargets.get(0)];
                        SecondTargetToFirstTarget = times[(ActiveTargets.get(0) - 1)][ActiveTargets.get(1)];
                    }
                }
                continue;
            }

            //moving motion
            if (NumberOfActiveTargets == 1) {
                if (checkMissionTime(currentToFirstTargetTime + FirstTargetToGoalTime)) {
                    if (checkActiveTime(currentToFirstTargetTime)) {

                        moveAndShot(currentPoint, ActiveTargets.get(0), numberOfPhotos);
                        numberOfPhotos += 2;
                        currentPoint = ActiveTargets.get(0);

                    }
                } else {
                    if (checkMissionTime(times[6][currentPoint] + times[7][7]) && !QRflag) {

                        moveAndShot(currentPoint, 7, numberOfPhotos);
                        numberOfPhotos += 2;
                        reportMessage = ReadQR();
                        currentPoint = 7;
                        QRflag = true;

                    }
                    break;
                }

            } else {
                route1 = currentToFirstTargetTime + FirstTargetToSecondTarget;
                route2 = currentToSecondTargetTime + SecondTargetToFirstTarget;
                routeToGoal1 = route1 + SecondTargetToGoalTime;
                routeToGoal2 = route2 + FirstTargetToGoalTime;

                if (route1 >= route2 && checkMissionTime(routeToGoal2) && checkActiveTime(route2)) {

                    moveAndShot(currentPoint, ActiveTargets.get(1), numberOfPhotos);
                    numberOfPhotos += 2;
                    moveAndShot(ActiveTargets.get(1), ActiveTargets.get(0), numberOfPhotos);
                    numberOfPhotos += 2;
                    currentPoint = ActiveTargets.get(0);

                } else if (route1 < route2 && checkMissionTime(routeToGoal1) && checkActiveTime(route1)) {

                    moveAndShot(currentPoint, ActiveTargets.get(0), numberOfPhotos);
                    numberOfPhotos += 2;
                    moveAndShot(ActiveTargets.get(0), ActiveTargets.get(1), numberOfPhotos);
                    numberOfPhotos += 2;
                    currentPoint = ActiveTargets.get(1);

                } else if (points1 > points2 && checkMissionTime(times[ActiveTargets.get(0) - 1][currentPoint] + FirstTargetToGoalTime) &&
                        checkActiveTime(times[ActiveTargets.get(0) - 1][currentPoint])){

                    moveAndShot(currentPoint, ActiveTargets.get(0), numberOfPhotos);
                    numberOfPhotos += 2;
                    currentPoint = ActiveTargets.get(0);

                } else if (points1 < points2 && checkMissionTime(times[ActiveTargets.get(1) - 1][currentPoint] + SecondTargetToGoalTime) &&
                        checkActiveTime(times[ActiveTargets.get(1) - 1][currentPoint])) {

                    moveAndShot(currentPoint, ActiveTargets.get(1), numberOfPhotos);
                    numberOfPhotos += 2;
                    currentPoint = ActiveTargets.get(1);

                } else if (currentToFirstTargetTime >= currentToSecondTargetTime &&
                        checkMissionTime(times[ActiveTargets.get(1) - 1][currentPoint] + SecondTargetToGoalTime) &&
                        checkActiveTime(times[ActiveTargets.get(1) - 1][currentPoint])) {

                    moveAndShot(currentPoint, ActiveTargets.get(1), numberOfPhotos);
                    numberOfPhotos += 2;
                    currentPoint = ActiveTargets.get(1);

                } else if (currentToFirstTargetTime < currentToSecondTargetTime &&
                        checkMissionTime(times[ActiveTargets.get(0) - 1][currentPoint] + FirstTargetToGoalTime) &&
                        checkActiveTime(times[ActiveTargets.get(0) - 1][currentPoint])) {

                    moveAndShot(currentPoint, ActiveTargets.get(0), numberOfPhotos);
                    numberOfPhotos += 2;
                    currentPoint = ActiveTargets.get(0);

                } else {
                    if (checkMissionTime(times[6][currentPoint] + times[7][7]) && !QRflag) {

                        moveAndShot(currentPoint, 7, numberOfPhotos);
                        reportMessage = ReadQR();
                        QRflag = true;
                        currentPoint = 7;

                    }
                    break;
                }

            }


            //staying condition check
            if(ActiveTargets.size() == api.getActiveTargets().size() && ActiveTargets.get(0) == api.getActiveTargets().get(0)){
                stayingCondition = true;
            }

            //get next target
            ActiveTargets = api.getActiveTargets();
            NumberOfActiveTargets = ActiveTargets.size();
            points1 = points[(ActiveTargets.get(0) - 1)];

            //get current time remaining
            TimeRemaining = api.getTimeRemaining();
            MissionTimeRemaining = TimeRemaining.get(1);

            currentToFirstTargetTime = times[(ActiveTargets.get(0) - 1)][currentPoint];
            FirstTargetToGoalTime = times[7][ActiveTargets.get(0)];

            if (NumberOfActiveTargets == 2) {
                points2 = points[(ActiveTargets.get(1) - 1)];
                currentToSecondTargetTime = times[(ActiveTargets.get(1) - 1)][currentPoint];
                SecondTargetToGoalTime = times[7][ActiveTargets.get(1)];
                FirstTargetToSecondTarget = times[(ActiveTargets.get(1) - 1)][ActiveTargets.get(0)];
                SecondTargetToFirstTarget = times[(ActiveTargets.get(0) - 1)][ActiveTargets.get(1)];
            }

        }

        api.notifyGoingToGoal();
        moveAndShot(currentPoint, 8, numberOfPhotos);
        Log.i(TAG, "-------------- LOG: QRflag=" + QRflag);
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

    public void moveAndShot(int from, int to, int numberOfPhotos) {
        //get time for count timeRequired (LOG)
        List<Long> TimeRemaining = api.getTimeRemaining();
        long countStart = TimeRemaining.get(1);

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

        Log.i(TAG, "-------------- DEBUG: Turbulence(X,Y,Z)=(" + TurbX + "," + TurbY + "," + TurbZ + ") for point" + to);

        Point point1 = new Point(11.2053d + TurbX, -9.87284d + TurbY, 5.4736d + TurbZ);
        Point point2 = new Point(10.456184d + TurbX, -9.196272d + TurbY, 4.53d + TurbZ);
        Point point3 = new Point(10.7142d + TurbX, -7.76727d + TurbY, 4.53d + TurbZ);
        Point point4 = new Point(10.56d + TurbX, -6.612872d + TurbY, 5.20641d + TurbZ);
        Point point7 = new Point(11.369d, -8.5518d, 4.48d);
        Point point8 = new Point(11.143d + TurbX, -6.7607d + TurbY, 4.9654d + TurbZ);

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

        //Point viapoint47 = new Point(10.64904d, -8.22217d, 5.29178d);
        Point viapoint47 = new Point(10.628d, -8.841d, 5.288d);
        
        Point viapoint78 = new Point(11.22584d, -8.80419d, 5.0922d);

        Point pivotPoint11 = new Point(10.85076d, -9.314d, 5.269d);
        Point pivotPoint12 = new Point(11.2d, -9.109d, 5.191d);
        Point pivotPoint2 = new Point(11.34547d, -7.393d, 4.63477d);
        Point pivotPoint3 = new Point(10.49585d, -7.393d, 5.30908d);

        Point viaPivot11to3 = new Point(10.47d, -7.902d, 4.948d);
        //Added special !!

        Result result;
        switch (from) {
            //prototype of pivoting method
            case 0:
                switch (to) {
                    case 1:
                        result = MoveTo(viapoint01, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                        }
                        result = MoveTo(point1, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                            MoveTo(point1, quartanion1);
                        }
                        break;
                    case 2:
                        result = MoveTo(point2, quartanion2);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion2);
                            MoveTo(point2, quartanion2);
                        }
                        try{
                            Thread.sleep(2000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        break;
                    case 3:
                        result = MoveTo(viapoint03First, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion3);
                            MoveTo(pivotPoint3, quartanion3);
                        }else {
                            result = MoveTo(viapoint03Second, quartanion3);
                            if(!result.hasSucceeded()){
                                MoveTo(pivotPoint11, quartanion3);
                                MoveTo(pivotPoint2, quartanion3);
                            }
                        }
                        result = MoveTo(point3, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion3);
                            MoveTo(pivotPoint2, quartanion3);
                        }
                        break;
                    case 4:
                        result = MoveTo(viapoint04, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion4);
                            MoveTo(pivotPoint3, quartanion4);
                        }
                        result = MoveTo(point4, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion4);
                            MoveTo(pivotPoint2, quartanion4);
                            MoveTo(point4, quartanion4);
                        }
                        break;
                    default:
                        break;
                }
                break;
            case 1:
                switch (to) {
                    case 2:
                        result = MoveTo(viapoint12, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                        }
                        result = MoveTo(point2, quartanion2);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                            MoveTo(point2, quartanion2);
                        }
                        try{
                            Thread.sleep(2000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        break;
                    case 3:
                        result = MoveTo(viapoint13, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion1);
                            MoveTo(pivotPoint2, quartanion3);
                        }
                        result = MoveTo(point3, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint2, quartanion3);
                            MoveTo(point3, quartanion3);
                        }
                        break;
                    case 4:
                        result = MoveTo(viapoint14, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                            MoveTo(pivotPoint3, quartanion4);
                        }
                        result = MoveTo(point4, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion4);
                            MoveTo(point4, quartanion4);
                        }
                        break;
                    case 7:
                        result = MoveTo(point7, quartanion7);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion1);
                            MoveTo(point7, quartanion7);
                        }
                        break;
                    case 8:
                        result = MoveTo(viapoint18, quartanion8);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                            MoveTo(pivotPoint3, quartanion1);
                        }
                        result = MoveTo(point8, quartanion8);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion8);
                            MoveTo(point8, quartanion8);
                        }
                        break;
                    default:
                        break;
                }
                break;
            case 2:
                switch (to) {
                    case 1:
                        result = MoveTo(viapoint12, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                        }
                        result = MoveTo(point1, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                            MoveTo(point1, quartanion1);
                        }
                        break;
                    case 3:
                        result = MoveTo(viapoint23, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion3);
                            MoveTo(viaPivot11to3, quartanion3);
                        }
                        result = MoveTo(point3, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion3);
                            MoveTo(point3, quartanion3);
                        }
                        break;
                    case 4:
                        result = MoveTo(viapoint24, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion4);
                            MoveTo(pivotPoint3, quartanion4);
                        }
                        result = MoveTo(point4, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion4);
                            MoveTo(point4, quartanion4);
                        }
                        break;
                    case 7:
                        result = MoveTo(viapoint27, quartanion7);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion7);
                        }
                        result = MoveTo(point7, quartanion7);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion7);
                            MoveTo(point7, quartanion7);
                        }
                        break;
                    case 8:
                        result = MoveTo(viapoint28, quartanion8);
                        // Via28 is same as Pivot3
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion8);
                            MoveTo(pivotPoint3, quartanion8);
                        }
                        MoveTo(point8, quartanion8);
                        break;
                    default:
                        break;
                }
                break;
            case 3:
                switch (to) {
                    case 1:
                        result = MoveTo(viapoint13, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint2, quartanion3);
                            MoveTo(pivotPoint12, quartanion1);
                        }
                        result = MoveTo(point1, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion3);
                            MoveTo(point1, quartanion1);
                        }
                        break;
                    case 2:
                        result = MoveTo(viapoint23, quartanion2);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion2);
                            MoveTo(pivotPoint11, quartanion2);
                        }
                        result = MoveTo(point2, quartanion2);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion2);
                            MoveTo(point2, quartanion2);
                        }
                        break;
                    case 4:
                        result = MoveTo(viapoint34, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion4);
                        }
                        result = MoveTo(point4, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion4);
                            MoveTo(point4, quartanion4);
                        }
                        break;
                    case 7:
                        result = MoveTo(viapoint13, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion7);
                            MoveTo(pivotPoint11, quartanion7);
                        }
                        result = MoveTo(viapoint78, quartanion7);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion7);
                        }
                        result = MoveTo(point7, quartanion7);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion7);
                            MoveTo(point7, quartanion7);
                        }
                        break;
                    case 8:
                        result = MoveTo(point8, quartanion8);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint2, quartanion8);
                            MoveTo(point8, quartanion8);
                        }
                        break;
                    default:
                        break;
                }
                break;
            case 4:
                switch (to) {
                    case 1:
                        result = MoveTo(viapoint14, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion4);
                            MoveTo(pivotPoint11, quartanion1);
                        }
                        result = MoveTo(point1, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion1);
                            MoveTo(point1, quartanion1);
                        }
                        break;
                    case 2:
                        result = MoveTo(viapoint24, quartanion2);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion4);
                            MoveTo(pivotPoint11, quartanion2);
                        }
                        result = MoveTo(point2, quartanion2);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion2);
                            MoveTo(point2, quartanion2);
                        }
                        break;
                    case 3:
                        result = MoveTo(viapoint34, quartanion3);
                        //Is route 4toVia34 long enough to change quartanion ???
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion3);
                        }
                        result = MoveTo(point3, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion3);
                            MoveTo(point3, quartanion3);
                        }
                        break;
                    case 7:
                        result = MoveTo(viapoint47, quartanion7);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion4);
                            MoveTo(pivotPoint11, quartanion7);
                        }
                        result = MoveTo(point7, quartanion7);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion7);
                            MoveTo(point7, quartanion7);
                        }
                        break;
                    case 8:
                        result = MoveTo(point8, quartanion8);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion8);
                            MoveTo(point8, quartanion8);
                        }
                        break;
                    default:
                        break;
                }
                break;
            case 7:
                switch (to) {
                    case 1:
                        result = MoveTo(point1, quartanion1);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion1);
                            MoveTo(point1, quartanion1);
                        }
                        break;
                    case 2:
                        result = MoveTo(viapoint27, quartanion2);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion2);
                        }
                        result = MoveTo(point2, quartanion2);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion2);
                            MoveTo(point2, quartanion2);
                        }
                        break;
                    case 3:
                        result = MoveTo(viapoint78, quartanion3);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion3);
                            MoveTo(viaPivot11to3, quartanion3);
                            MoveTo(point3, quartanion3);
                        }else{
                            result = MoveTo(viapoint13, quartanion3);
                            if(!result.hasSucceeded()){
                                MoveTo(pivotPoint11, quartanion3);
                                MoveTo(viaPivot11to3, quartanion3);
                            }
                            result = MoveTo(point3, quartanion3);
                            if(!result.hasSucceeded()){
                                MoveTo(viaPivot11to3, quartanion3);
                                MoveTo(point3, quartanion3);
                            }
                        }
                        break;
                    case 4:
                        result = MoveTo(viapoint47, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion4);
                            MoveTo(pivotPoint3, quartanion4);
                        }
                        result = MoveTo(point4, quartanion4);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint3, quartanion4);
                            MoveTo(point4, quartanion4);
                        }
                        break;
                    case 8:
                        result = MoveTo(viapoint78, quartanion7);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint12, quartanion8);
                        }
                        result = MoveTo(viapoint47, quartanion8);
                        if(!result.hasSucceeded()){
                            MoveTo(pivotPoint11, quartanion7);
                        }
                        MoveTo(pivotPoint3, quartanion8);
                        MoveTo(point8, quartanion8);
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }

        if(!(to == 8)){
            // 移動後、astrobeeが安定してから画像を撮影するために2秒待つ
            try{
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (!(to == 7 || to == 8)) {
            reMove_AR_moveTo(to, numberOfPhotos);
            api.laserControl(true);
            api.takeTargetSnapshot(to);
        }

        //get timeRequired(LOG)
        TimeRemaining = api.getTimeRemaining();
        long countEnd = TimeRemaining.get(1);
        long timeRequired = countStart - countEnd;
        long ActiveTimeRemaining = TimeRemaining.get(0);

        Log.i(TAG, "-------------- LOG: timerequired" + from + to + "=" + timeRequired);
        Log.i(TAG, "-------------- LOG: ActiveTimeRemaining=" + ActiveTimeRemaining);
    }

    public Result MoveTo(Point point, Quaternion quaternion) {
        Result result;
        final int LOOP_MAX = 2;

        result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            Log.i(TAG, "-------------- DEBUG: move failed");
            result = api.moveTo(point, quaternion, true);
            loopCounter++;
        }
        return result;
    }

        public Mat image_correction (Mat image){

            double[][] NavCamIntrinsics = api.getNavCamIntrinsics();
            Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
            Mat distortion = new Mat(1, 5, CvType.CV_32FC1);
            cameraMat.put(0, 0, NavCamIntrinsics[0]);
            distortion.put(0, 0, NavCamIntrinsics[1]);

            Mat correct_image = new Mat();
            undistort(image, correct_image, cameraMat, distortion);

            return correct_image;
          
        }

        public String ReadQR () {
            //get time for count timeRequired (LOG)
            List<Long> TimeRemaining = api.getTimeRemaining();
            long countStart = TimeRemaining.get(1);

            //ReadQRCode
            api.flashlightControlFront(0.05f);
            Mat QRimage = image_correction(api.getMatNavCam());
            QRCodeDetector decoder = new QRCodeDetector();
            String data = decoder.detectAndDecode(QRimage);

            //Generate png image for debug
            api.saveMatImage(QRimage, "QR.png");

            String reportMessage = "empty";
            switch (data) {
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

            //get timeRequired(LOG)
            TimeRemaining = api.getTimeRemaining();
            long countEnd = TimeRemaining.get(1);
            long timeRequired = countStart - countEnd;
            long ActiveTimeRemaining = TimeRemaining.get(0);

            Log.i(TAG, "-------------- LOG: QRtimerequired=" + timeRequired);
            Log.i(TAG, "-------------- LOG: ActiveTimeRemaining=" + ActiveTimeRemaining);

            return reportMessage;
        }

        public boolean checkMissionTime ( long requiredTime){
            List<Long> TimeRemaining = api.getTimeRemaining();
            Long MissionTimeRemaining = TimeRemaining.get(1);
            Log.i(TAG, "-------------- DEBUG: requiredTime = " + requiredTime);
            Log.i(TAG, "-------------- DEBUG: MissionTimeRemaining = " + MissionTimeRemaining);

            // if MissionTimeRemaining is larger than requiredTime, Astrobee can go to the target
            if (MissionTimeRemaining > requiredTime) {
                Log.i(TAG, "-------------- DEBUG: checkMissionTime = true");
                return true;
            } else {
                Log.i(TAG, "-------------- DEBUG: checkMissionTime = false");
                return false;
            }
        }
  
        public boolean checkActiveTime ( long requiredTime){
            List<Long> TimeRemaining = api.getTimeRemaining();
            Long ActiveTimeRemaining = TimeRemaining.get(0);
            Log.i(TAG, "-------------- DEBUG: requiredTime = " + requiredTime);
            Log.i(TAG, "-------------- DEBUG: ActiveTimeRemaining = " + ActiveTimeRemaining);

            // if ActiveTimeRemaining is larger than requiredTime, Astrobee can go to the target
            if (ActiveTimeRemaining > requiredTime) {
                Log.i(TAG, "-------------- DEBUG: checkActiveTime = true");
                return true;
            } else {
                Log.i(TAG, "-------------- DEBUG: checkActiveTime = false");
                return false;
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

            Log.i(TAG, "-------------- DEBUG: relative[0](in real)=" + relative[0]);
            Log.i(TAG, "-------------- DEBUG: relative[1](in real)=" + relative[1]);
          
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
            Mat image = getMatNavCam();

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
                    // TODO: Concern what to do if camera can't find AR marker
                    Log.i(TAG, "can't caluculate target_center");
                }

            }

            /*　各ARmarkerの四隅座標でのターゲット中心座標を求めたら、その平均値をtarget_centerに代入する */
            double target_x = 0;
            double target_y = 0;
            for (int i = 0; i < n; i++) {
                target_x += center_cand[i][0];
                target_y += center_cand[i][1];
                Log.i(TAG, "-------------- DEBUG: center_cand_" + i + "=" + center_cand[i][0] + " and " + center_cand[i][1]);
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

            Log.i(TAG, "-------------- DEBUG: scale[pixel/meter]=" + scale);
            Log.i(TAG, "-------------- DEBUG: 700~900ぐらいの値が予想される");

            return scale;

        }

    public void reMove_AR_moveTo (int to, int numberOfPhotos){

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

        Log.i(TAG, "-------------- DEBUG: current_point=" + current_point);

        /*　目的地のターゲット番号によってx,y,z座標の修正を場合分けしている
         *　また、位置修正時に5cmターゲット方向に前進する（最小移動距離確保のため）
         */
        switch (to) {
            case 1:
                api.saveMatImage(image_correction(getMatNavCam()), numberOfPhotos + ":target1Image__before.png");
                double dest_x1 = current_point.getX() + relative[0];
                double dest_z1 = current_point.getZ() + relative[1];
                Point new_point1 = new Point(dest_x1, current_point.getY()-0.05, dest_z1);
                api.moveTo(new_point1, quaternion1, true);
                api.saveMatImage(image_correction(getMatNavCam()), (numberOfPhotos + 1) + ":target1Image__after.png");
                break;

            case 2:
                api.saveMatImage(image_correction(getMatNavCam()), numberOfPhotos + ":target2Image__before.png");
                double dest_x2 = current_point.getX() + relative[0];
                double dest_y2 = current_point.getY() - relative[1];
                Point new_point2 = new Point(dest_x2, dest_y2, current_point.getZ()-0.05);
                api.moveTo(new_point2, quaternion2, true);
                api.saveMatImage(image_correction(getMatNavCam()), (numberOfPhotos + 1) + ":target2Image__after.png");
                break;

            case 3:
                api.saveMatImage(image_correction(getMatNavCam()), numberOfPhotos + ":target3Image__before.png");
                double dest_y3 = current_point.getY() + relative[0];
                double dest_x3 = current_point.getX() + relative[1];
                Point new_point3 = new Point(dest_x3, dest_y3, current_point.getZ()-0.05);
                api.moveTo(new_point3, quaternion3, true);
                api.saveMatImage(image_correction(getMatNavCam()), (numberOfPhotos + 1) + ":target3Image__after.png");
                break;

            case 4:
                api.saveMatImage(image_correction(getMatNavCam()), numberOfPhotos + ":target4Image__before.png");
                double dest_y4 = current_point.getY() - relative[0];
                double dest_z4 = current_point.getZ() + relative[1];
                Point new_point4 = new Point(current_point.getX()-0.05, dest_y4, dest_z4);
                api.moveTo(new_point4, quaternion4, true);
                api.saveMatImage(image_correction(getMatNavCam()), (numberOfPhotos + 1) + ":target4Image__after.png");
                break;

            default:
                break;
        }

        // for DEBUG　本番実装時には除いて良い
        Kinematics kinematics_after = api.getRobotKinematics();
        Point after_point = kinematics_after.getPosition();

        Log.i(TAG, "-------------- DEBUG: after_point=" + after_point);
        Log.i(TAG, "-------------- DEBUG: current_pointとの差分がrelative[](in real)と同じであれば移動は成功");

    }
        // NULL check
        public Mat getMatNavCam () {

            int LOOP_MAX = 5;
            int loopCounter = 0;
            Mat image = api.getMatNavCam();

            while (image == null && loopCounter < LOOP_MAX) {
                image = api.getMatNavCam();
            }
          
            return image;
        }

    }

