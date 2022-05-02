package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    public static final boolean PRINT_ROBOT_POSITION = true;
    public static final boolean SAVE_IMAGE = true;

    private Point lastPoint;
    private Quaternion lastQuaternion;

    @Override
    protected void runPlan1() {
        // the mission starts
        api.startMission();

        // move to a point
        Point point1 = new Point(10.71f, -7.7f, 4.48f);
        Quaternion quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
        moveToWithRetry(point1, quaternion);

        // report point1 arrival
        api.reportPoint1Arrival();

        if (SAVE_IMAGE) {
            Bitmap image = api.getBitmapNavCam();
            Canvas canvas = new Canvas(image);
            Paint paint = new Paint();
            paint.setStrokeWidth(2);
            paint.setColor(Color.GREEN);
            canvas.drawCircle(640, 480, 2, paint);
            canvas.drawCircle(640, 440, 2, paint);
            api.saveBitmapImage(image, "NavCam_original");
            image.recycle();
        }

        alignmentPoint1();

        // irradiate the laser
        api.laserControl(true);
        // take target1 snapshots
        api.takeTarget1Snapshot();
        if (SAVE_IMAGE) {
            Bitmap image = api.getBitmapNavCam();
            Canvas canvas = new Canvas(image);
            Paint paint = new Paint();
            paint.setStrokeWidth(2);
            paint.setColor(Color.GREEN);
            canvas.drawCircle(640, 480, 2, paint);
            api.saveBitmapImage(image, "NavCam_laser1");
            image.recycle();
        }
        // turn the laser off
        api.laserControl(false);



        Quaternion quaternion2 = new Quaternion(0, 0, -0.707f, 0.707f);

        Point pointTemp;
        pointTemp = new Point(11.2, -8.2, 4.5);
        moveToWithRetry(pointTemp, quaternion2);
        pointTemp = new Point(11.2, -9.5, 4.5);
        moveToWithRetry(pointTemp, quaternion2);

        Point point2 = new Point(11.27460, -9.92284, 5.29881);
        moveToWithRetry(point2, quaternion2);

        alignmentPoint2();

        // irradiate the laser
        api.laserControl(true);
        // take target2 snapshots
        api.takeTarget2Snapshot();
        if (SAVE_IMAGE) {
            Bitmap image = api.getBitmapNavCam();
            Canvas canvas = new Canvas(image);
            Paint paint = new Paint();
            paint.setStrokeWidth(2);
            paint.setColor(Color.GREEN);
            canvas.drawCircle(640, 480, 2, paint);
            api.saveBitmapImage(image, "NavCam_laser2");
            image.recycle();
        }
        // turn the laser off
        api.laserControl(false);

        pointTemp = new Point(11.05, -9.5, 4.96538);
        moveToWithRetry(pointTemp, quaternion2);

        pointTemp = new Point(11, -9.1 , 4.96538);
        moveToWithRetry(pointTemp, quaternion2);


        Point pointGoal = new Point(11.27460, -7.89178, 4.96538);
        Quaternion quaternionGoal = new Quaternion(0, 0, -0.707f, 0.707f);
        moveToWithRetry(pointGoal, quaternionGoal);

        // send mission completion
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }


    private boolean moveToWithRetry(Point point, Quaternion quaternion) {
        Result result;
        final int LOOP_MAX = 10;
        final float MAX_THRESHOLD = 0.02f;
        result = api.moveTo(point, quaternion, PRINT_ROBOT_POSITION);
        Quaternion currentQuaternion = api.getRobotKinematics().getOrientation();
        int loopCounter = 0;
        while ((Math.abs(currentQuaternion.getX() - quaternion.getX()) >= MAX_THRESHOLD
                || Math.abs(currentQuaternion.getY() - quaternion.getY()) >= MAX_THRESHOLD
                || Math.abs(currentQuaternion.getZ() - quaternion.getZ()) >= MAX_THRESHOLD
                || Math.abs(currentQuaternion.getW() - quaternion.getW()) >= MAX_THRESHOLD
                || !result.hasSucceeded())
                && loopCounter < LOOP_MAX) {
            result = api.moveTo(point, quaternion, PRINT_ROBOT_POSITION);
            ++loopCounter;
        }
        lastPoint = point;
        lastQuaternion = quaternion;
        return result.hasSucceeded();
    }

    private boolean moveToPointWithRetry(Point point) {
        return moveToWithRetry(point, lastQuaternion);
    }

    private boolean moveToQuaternionWithRetry(Quaternion quaternion) {
        return moveToWithRetry(lastPoint, quaternion);
    }

    private void alignmentPoint1() {
        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        final int LOOP_MAX = 5;
        int loopCounter = 0;
        while (arucoIDs.size().area() == 0 && loopCounter < LOOP_MAX) {
            moveToWithRetry(lastPoint, lastQuaternion);
            Mat navCamMat = api.getMatNavCam();
            Aruco.detectMarkers(
                    navCamMat,
                    Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                    arucoCorners,
                    arucoIDs);
            ++loopCounter;
//            if (SAVE_IMAGE) {
//                api.saveMatImage(navCamMat, "NavCam_" + loopCounter);
//            }
        }
        if (arucoIDs.size().area() == 0) return;

        double targetPixelX = 0;
        double targetPixelY = 0;

        for (int i = 0; i < arucoIDs.size().height; i++) {
            Mat mat = arucoCorners.get(i);
            double[] pixelLeftUp = mat.get(0, 0);
            double[] pixelLeftDown = mat.get(0, 1);
            double[] pixelRightUP = mat.get(0, 2);
            double[] pixelRightDown = mat.get(0, 3);
            targetPixelX += pixelLeftUp[0] + pixelLeftDown[0] + pixelRightUP[0] + pixelRightDown[0];
            targetPixelY += pixelLeftUp[1] + pixelLeftDown[1] + pixelRightUP[1] + pixelRightDown[1];
        }

        targetPixelX /= 16;
        targetPixelY /= 16;

        if (SAVE_IMAGE) {
            Bitmap image = api.getBitmapNavCam();
            Canvas canvas = new Canvas(image);
            Paint paint = new Paint();
            paint.setStrokeWidth(2);
            paint.setColor(Color.GREEN);
            canvas.drawCircle(640, 480, 2, paint);
            paint.setColor(Color.BLUE);
            canvas.drawCircle((float) targetPixelX, (float) targetPixelY, 2, paint);
            api.saveBitmapImage(image, "NavCam_alignment1");
            image.recycle();
        }

        double dX = targetPixelX - 640;
        double dY = targetPixelY - 480;


        // 8 pixel is 1 cm
        dX /= 8;
        dY /= 8;

        float xOffset = 0.15f;

        double roll = 0;
        double pitch = -Math.atan2(dY, 110 + xOffset * 100) + 0.5 * Math.PI;
        double yaw = Math.atan2(dX, 110 + xOffset * 100);


        Point point = new Point(10.71f, -7.7f, 4.48f + xOffset);
        moveToWithRetry(point, euler_to_quaternion(roll, pitch, yaw));
    }

    private void alignmentPoint2() {
        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        final int LOOP_MAX = 5;
        int loopCounter = 0;
        while (arucoIDs.size().area() == 0 && loopCounter < LOOP_MAX) {
            moveToWithRetry(lastPoint, lastQuaternion);
            Mat navCamMat = api.getMatNavCam();
            Aruco.detectMarkers(
                    navCamMat,
                    Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                    arucoCorners,
                    arucoIDs);
            ++loopCounter;
//            if (SAVE_IMAGE) {
//                api.saveMatImage(navCamMat, "NavCam_" + loopCounter);
//            }
        }
        if (arucoIDs.size().area() == 0) return;

        double targetPixelX = 0;
        double targetPixelY = 0;

        for (int i = 0; i < arucoIDs.size().height; i++) {
            Mat mat = arucoCorners.get(i);
            double[] pixelLeftUp = mat.get(0, 0);
            double[] pixelLeftDown = mat.get(0, 1);
            double[] pixelRightUP = mat.get(0, 2);
            double[] pixelRightDown = mat.get(0, 3);
            targetPixelX += pixelLeftUp[0] + pixelLeftDown[0] + pixelRightUP[0] + pixelRightDown[0];
            targetPixelY += pixelLeftUp[1] + pixelLeftDown[1] + pixelRightUP[1] + pixelRightDown[1];
        }

        targetPixelX /= 16;
        targetPixelY /= 16;

        if (SAVE_IMAGE) {
            Bitmap image = api.getBitmapNavCam();
            Canvas canvas = new Canvas(image);
            Paint paint = new Paint();
            paint.setStrokeWidth(2);
            paint.setColor(Color.GREEN);
            canvas.drawCircle(640, 480, 2, paint);
            paint.setColor(Color.BLUE);
            canvas.drawCircle((float) targetPixelX, (float) targetPixelY, 2, paint);
            api.saveBitmapImage(image, "NavCam_alignment2");
            image.recycle();
        }

        double dX = targetPixelX - 640;
        double dY = targetPixelY - 480;


        // 8 pixel is 1 cm
        dX /= 8;
        dY /= 8;

        float xOffset = 0.15f;

        double roll = 0;
        double pitch = -Math.atan2(dY, 110 + xOffset * 100) + 0.5 * Math.PI;
        double yaw = Math.atan2(dX, 110 + xOffset * 100);


        Point point = new Point(10.71f, -7.7f, 4.48f + xOffset);
        //moveToWithRetry(point, euler_to_quaternion(roll, pitch, yaw));
    }

    public Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {

        final float sin_roll = (float) Math.sin(roll * 0.5f);
        final float cos_roll = (float) Math.cos(roll * 0.5f);

        final float sin_pitch = (float) Math.sin(pitch * 0.5f);
        final float cos_pitch = (float) Math.cos(pitch * 0.5f);

        final float sin_yaw = (float) Math.sin(yaw * 0.5f);
        final float cos_yaw = (float) Math.cos(yaw * 0.5f);

        float qx = (sin_roll * cos_pitch * cos_yaw) - (cos_roll * sin_pitch * sin_yaw) * (-1);
        float qy = (cos_roll * sin_pitch * cos_yaw) + (sin_roll * cos_pitch * sin_yaw) * (-1);
        float qz = (cos_roll * cos_pitch * sin_yaw) - (sin_roll * sin_pitch * cos_yaw) * (-1);
        float qw = (cos_roll * cos_pitch * cos_yaw) + (sin_roll * sin_pitch * sin_yaw) * (-1);

        return new Quaternion(qx, qy, qz, qw);

    }
}

