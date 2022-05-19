package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

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

    public static final int MAX_RETRY_SET_LASER = 5;

    public static final Point point1 = new Point(10.71f, -7.7f, 4.48f);
    public static final Point point2 = new Point(11.27460, -9.92284, 5.29881);
    public static final Point pointGoal = new Point(11.27460, -7.89178, 4.96538);
    public static final Quaternion point1Quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
    public static final Quaternion point2Quaternion = new Quaternion(0f, 0, -0.707f, 0.707f);
    public static final Quaternion pointGoalQuaternion = new Quaternion(0, 0, -0.707f, 0.707f);

    private Point lastPoint;
    private Quaternion lastQuaternion;

    @Override
    protected void runPlan1() {
        // the mission starts
        api.startMission();

        // move to point 1
        moveToWithRetry(point1, point1Quaternion);

        sleep(5000);

        // report point1 arrival
        api.reportPoint1Arrival();

        // save target 1 original picture
        saveImage("NavCam_target1_original.png");

        // align to target 1
        alignmentPoint1();

        // turn on laser
        setLaser(true);

        // take target 1 snapshots
        api.takeTarget1Snapshot();

        // save target 1 laser image
        saveImage("NavCam_target1_laser.png");

        // turn off laser
        setLaser(false);

        // move to point 2
        Point pointTemp;
        pointTemp = new Point(11.2, -8.2, 4.5);
        moveToWithRetry(pointTemp, point2Quaternion);
        pointTemp = new Point(11.2, -9.5, 4.5);
        moveToWithRetry(pointTemp, point2Quaternion);
        moveToWithRetry(point2, point2Quaternion);

        // wait 2 seconds before taking picture from NavCam
        //sleep(5000);

        // save target 2 original picture
        saveImage("NavCam_target2_original.png");

        // align to target 2
        alignmentPoint2();

        // turn on laser
        setLaser(true);

        // take target 2 snapshots
        api.takeTarget2Snapshot();

        // save target 2 laser picture
        saveImage("NavCam_target2_laser.png");

        // turn off laser
        setLaser(false);

        // move to point goal
        pointTemp = new Point(11.05, -9.5, 4.96538);
        moveToWithRetry(pointTemp, point2Quaternion);
        pointTemp = new Point(11, -9.1, 4.96538);
        moveToWithRetry(pointTemp, point2Quaternion);
        moveToWithRetry(pointGoal, pointGoalQuaternion);

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

    private void alignmentPoint1() {
        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();
        Mat navCamMat = api.getMatNavCam();
        final int LOOP_MAX = 5;
        int loopCounter = 0;

        Aruco.detectMarkers(
                navCamMat,
                Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                arucoCorners,
                arucoIDs);
        ++loopCounter;
        while (arucoIDs.size().area() == 0 && loopCounter < LOOP_MAX) {
            moveToWithRetry(point1, point1Quaternion);
            navCamMat = api.getMatNavCam();
            Aruco.detectMarkers(
                    navCamMat,
                    Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                    arucoCorners,
                    arucoIDs);
            ++loopCounter;
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
            api.saveBitmapImage(image, "NavCam_target1_alignment1.png");
            image.recycle();
        }

        double dX = targetPixelX - 640;
        double dY = targetPixelY - 480;


        // 8 pixel is 1 cm
        dX /= 8;
        dY /= 8;

        dX -= 9.94;
        dY += 2.85;

        Point point = new Point(point1.getX() + dY / 100.0, point1.getY() + dX / 100.0, point1.getZ());
        moveToWithRetry(point, new Quaternion(0f, 0.707f, 0f, 0.707f));
    }

    private void alignmentPoint2() {
        Point lastPoint = new Point(point2.getX(), point2.getY(), point2.getZ());
        final int steps = 2;
        for (int step = 0; step < steps; step++) {
            sleep(5000);
            Mat navCamMat = api.getMatNavCam();

            List<Mat> arucoCorners = new ArrayList<>();
            Mat arucoIDs = new Mat();

            final int LOOP_MAX = 5;
            int loopCounter = 0;
            Aruco.detectMarkers(
                    navCamMat,
                    Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                    arucoCorners,
                    arucoIDs);
            while (arucoIDs.size().area() == 0 && loopCounter < LOOP_MAX) {
                moveToWithRetry(lastPoint, point2Quaternion);
                navCamMat = api.getMatNavCam();
                Aruco.detectMarkers(
                        navCamMat,
                        Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                        arucoCorners,
                        arucoIDs);
                ++loopCounter;
            }
            if (arucoIDs.size().area() == 0) continue;

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

            Mat navCamGrayCroppedMat = navCamMat.submat((int) targetPixelY - 60, (int) targetPixelY + 60, (int) targetPixelX - 60, (int) targetPixelX + 60);
            Imgproc.threshold(navCamGrayCroppedMat, navCamGrayCroppedMat, 70, 255, Imgproc.THRESH_BINARY);
            Mat circles = new Mat();
            Imgproc.HoughCircles(navCamGrayCroppedMat, circles, Imgproc.HOUGH_GRADIENT, 1, 20, 50, 40);
            double[] minCircle = {0, 0};
            int minRadius = 1000;
            for (int col = 0; col < circles.cols(); col++) {
                double[] circle = circles.get(0, col);
                int radius = (int) Math.round(circle[2]);
                Log.i("Kibo", "find circle x=" + circle[0] + " y=" + circle[1] + " radius=" + radius);
                if (circle[0]!=0 && circle[1]!=0 && radius <= minRadius) {
                    minCircle[0] = circle[0];
                    minCircle[1] = circle[1];
                    minRadius = radius;
                }
            }
            Log.i("Kibo", "target circle x=" + minCircle[0]);
            Log.i("Kibo", "target circle y=" + minCircle[1]);
            Log.i("Kibo", "target circle radius=" + minRadius);
            Imgproc.circle(navCamGrayCroppedMat, new org.opencv.core.Point(minCircle[0], minCircle[1]), 3, new Scalar(255, 0, 255), 2, 0);
            if (SAVE_IMAGE) {
                api.saveMatImage(navCamGrayCroppedMat, "NavCam_target2_cropped" + step + ".png");
            }

            if (minRadius == 1000) continue;

            targetPixelX += minCircle[0] - 60;
            targetPixelY += minCircle[1] - 60;


            if (SAVE_IMAGE) {
                Bitmap image = api.getBitmapNavCam();
                Canvas canvas = new Canvas(image);
                Paint paint = new Paint();
                paint.setStrokeWidth(2);
                paint.setColor(Color.GREEN);
                canvas.drawCircle(640, 480, 4, paint);
                paint.setColor(Color.BLUE);
                canvas.drawCircle((float) targetPixelX, (float) targetPixelY, 4, paint);
                api.saveBitmapImage(image, "NavCam_alignmentPoint2_" + step + ".png");
                image.recycle();
            }

            double dX = targetPixelX - 640;
            double dY = targetPixelY - 480;

            // 9 pixel is 1 cm
            dX /= 9;
            dY /= 9;

            // fix the distance between camera and laser
            dX -= 9.94;
            dY += 2.85;

            // fix custom distance offset
            dX -= 0.4;
            dY -= 0.3;

//            double roll = -Math.atan2(dY, 75 + yOffset * 100);
//            double pitch = 0f;
//            double yaw = Math.atan2(dX, 75 + yOffset * 100) - 0.5 * Math.PI;

            // 平移不旋轉
            Point point = new Point(lastPoint.getX() + dX / 100, point2.getY(), lastPoint.getZ() + dY / 100);

            moveToWithRetry(point, point2Quaternion);
            lastPoint = point;
        }

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

    public Result setLaser(boolean enable) {
        Result laserResult = api.laserControl(enable);
        int loop = 0;
        while (!laserResult.hasSucceeded() && loop < MAX_RETRY_SET_LASER) {
            laserResult = api.laserControl(enable);
            ++loop;
        }
        return laserResult;
    }

    public void saveImage(String name) {
        if (!SAVE_IMAGE) return;
        Bitmap image = api.getBitmapNavCam();
        Canvas canvas = new Canvas(image);
        Paint paint = new Paint();
        paint.setStrokeWidth(2);
        paint.setColor(Color.GREEN);
        canvas.drawCircle(640, 480, 2, paint);
        api.saveBitmapImage(image, name);
        image.recycle();
    }

    public void sleep(long millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

