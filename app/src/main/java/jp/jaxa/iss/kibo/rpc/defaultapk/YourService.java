package jp.jaxa.iss.kibo.rpc.defaultapk;

//JAXA LIBRARY
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.android.gs.MessageType;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

//ZXING library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import android.graphics.Bitmap;
import android.util.Log;

import org.apache.commons.lang.builder.ReflectionToStringBuilder;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import static android.content.ContentValues.TAG;
import static java.lang.Math.sqrt;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        // write here your plan 1

        api.startMission();     // astrobee is undocked and the mission starts

        //move to Point A
        moveToWrapper(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707);

        String pos_A_prime = qrScan();      //scanning the QR
        api.sendDiscoveredQR(pos_A_prime);      //Sending to judge the read string

        //Debugging Purposes
        //sendData(MessageType.STRING, "Position_data", pos_A_prime);

        //String regex = "{\"p\":\\d{1},\"x\":\\d{1,2}.\\d{1,8},\"y\":-8.12,\"z\":5.45}";   //Not needed

        //Debugging Purposes
        //sendData(MessageType.STRING, "Position_data", String.valueOf(extractor(pos_A_prime, "\"p\":(.*?),")));
        //sendData(MessageType.STRING, "Position_data", String.valueOf(extractor(pos_A_prime, "\"x\":(.*?),")));
        //sendData(MessageType.STRING, "Position_data", String.valueOf(extractor(pos_A_prime, "\"y\":(.*?),")));
        //sendData(MessageType.STRING, "Position_data", String.valueOf(extractor(pos_A_prime, "\"z\":(.*?)\\}")));

        Double KOZ_pattern = S2D(extractor(pos_A_prime, "\"p\":(.*?),"));
        Double pos_x = S2D(extractor(pos_A_prime, "\"x\":(.*?),"));
        Double pos_y = S2D(extractor(pos_A_prime, "\"y\":(.*?),"));
        Double pos_z = S2D(extractor(pos_A_prime, "\"z\":(.*?)\\}"));

        //Move to point A prime
        moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);

        //Aruco AR Tag reading starts from here
        Mat img = api.getMatNavCam();
        //Vector<Integer> markerIDs = new Vector<>();
        Mat markerIDs = new Mat();
        List<Mat> markerCorners = new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters param = DetectorParameters.create();
        List<Mat> rejectedImgpoints = new ArrayList<>();
        double[][] camIntrinsics = api.getNavCamIntrinsics();

        //double[] cameraMatrix = Arrays.stream(camIntrinsics).map(r -> r[0]).toArray(double[]::new);
        //double[] distcoeff = Arrays.stream(camIntrinsics).map(r -> r[1]).toArray(double[]::new);
        // Needs java 8 for lambda expression

        double[] camMat = camIntrinsics[0];
        //sendData(MessageType.STRING, "data", String.valueOf(camMat));   //for debugging purpose
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32F);
        cameraMatrix.put(0, 0, camMat);

        double[] distDouble = camIntrinsics[1];
        //sendData(MessageType.STRING, "data", String.valueOf(distDouble));   //for debugging purpose
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32F);
        distCoeffs.put(0, 0, distDouble);


        Aruco.detectMarkers(img, dictionary, markerCorners, markerIDs, param, rejectedImgpoints, cameraMatrix, distCoeffs);

        int id[] = new int[markerIDs.rows()];
        for (int i = 0; i < markerIDs.rows(); i++) {
            //for (int j = 0; j< markerIDs.cols(); j++){
                id[i] = (int) markerIDs.get(i, 0)[0];
            //}
        }
        Log.i("MarkerIDs", Arrays.toString(id));//sending data for each individual marker

        /*
        for(Mat eachArr : markerCorners) {
            String dump = eachArr.dump();
            Log.d(TAG, dump);
        } */



        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        float markerLen = 0.05f;

        Aruco.estimatePoseSingleMarkers(markerCorners, markerLen, cameraMatrix, distCoeffs, rvecs, tvecs);

        String dump = rvecs.dump();
        Log.d(TAG, dump);
        String dump1 = tvecs.dump();
        Log.d(TAG, dump1);

        Mat rotMat = new Mat(3,3, CvType.CV_32F);
        for(int i=0; i < id.length; i++) {
            for (int j = 0; j < 3; j++) {
                Log.i("rvecs", String.valueOf(rvecs.get(i, 0)[j]));
                //Calib3d.Rodrigues(rvecs.get(i,0), rotMat);   // 1x3 rotation vector to 3x3 rotation matrix
            }
        }





        Mat coordinates = getCoordinates((int) (markerIDs.size().height), rvecs, tvecs);

        String Coord = new String();
        for (int i = 0; i < coordinates.rows(); i++) {
            for (int j = 0; j < coordinates.cols(); j++) {
                double coord = coordinates.get(i, j)[0];
                Coord += ", " + coord;
                Log.i("CoOrdinate", Coord);
            }
        }
        sendData(MessageType.STRING, "data", Coord);

        //Log.i("trust kine", java.util.Arrays.toString(new Kinematics[]{api.getTrustedRobotKinematics()}));
        //Log.i("trust kine", ReflectionToStringBuilder.toString(api.getTrustedRobotKinematics()));


    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }


    // ###################### Our Own methods or Functions ##############################

    //for moving to a certain coordinate including rotation
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private String qrScan() {            //Seperate QR scanning function with redundancy enabled
        final int LOOP_MAX = 3;
        com.google.zxing.Result qrcode = null;
        int loopCounter = 0;
        while (qrcode == null || loopCounter < LOOP_MAX) {
            Bitmap bm = api.getBitmapNavCam();
            int[] intB1 = new int[bm.getWidth() * bm.getHeight()];
            bm.getPixels(intB1, 0, bm.getWidth(), 0, 0, bm.getWidth(), bm.getHeight());
            LuminanceSource source = new RGBLuminanceSource(bm.getWidth(), bm.getHeight(), intB1);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            try {
                qrcode = new MultiFormatReader().decode(bitmap);
            } catch (Exception e) {
                qrcode = null;
            }
            ++loopCounter;
        }
        return qrcode.getText();
    }

    private String extractor(String data, String pattern) {
        Pattern ptrn = Pattern.compile(pattern);
        Matcher matcher = ptrn.matcher(data);
        if (matcher.find()) {
            data = matcher.group(1);
        }

        return data;
    }

    private double S2D(String s) {
        return Double.parseDouble(s);
    }

    private void getQuaternion(Mat R, double Q[]) {         //Rotation matrix to Quaternion
        double trace = R.get(0, 0)[0] + R.get(1, 1)[0] + R.get(2, 2)[0];

        if (trace > 0.0) {
            double s = sqrt(trace + 1.0);
            Q[3] = (s * 0.5);
            s = 0.5 / s;
            Q[0] = ((R.get(2, 1)[0] - R.get(1, 2)[0]) * s);
            Q[1] = ((R.get(0, 2)[0] - R.get(2, 0)[0]) * s);
            Q[2] = ((R.get(1, 0)[0] - R.get(0, 1)[0]) * s);
        } else {
            int i = R.get(0, 0)[0] < R.get(1, 1)[0] ? (R.get(1, 1)[0] < R.get(2, 2)[0] ? 2 : 1) : (R.get(0, 0)[0] < R.get(2, 2)[0] ? 2 : 0);
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;

            double s = sqrt(R.get(i, i)[0] - R.get(j, j)[0] - R.get(k, k)[0] + 1.0);
            Q[i] = s * 0.5;
            s = 0.5 / s;

            Q[3] = (R.get(k, j)[0] - R.get(j, k)[0]) * s;
            Q[j] = (R.get(j, i)[0] + R.get(i, j)[0]) * s;
            Q[k] = (R.get(k, i)[0] + R.get(i, k)[0]) * s;
        }

    }

    private Mat getCoordinates(int len, Mat rvecs, Mat tvecs) {
        Mat result = new Mat();
        Mat.zeros(3, 1, CvType.CV_64F).copyTo(result);
        Mat rotMatrix = new Mat();
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        Mat tempResult = new Mat();
        Mat.zeros(3, 1, CvType.CV_64F).copyTo(rvec);
        Mat.zeros(3, 1, CvType.CV_64F).copyTo(tvec);
        Mat.zeros(3, 1, CvType.CV_64F).copyTo(tempResult);
        //TODO: 20-06-2018 "change 1 below to len";
        //len=1;
        for (int i = 0; i < len; i++) {
            rvec.put(0, 0, rvecs.get(i, 0)[0]);
            rvec.put(1, 0, rvecs.get(i, 0)[1]);
            rvec.put(2, 0, rvecs.get(i, 0)[2]);
            tvec.put(0, 0, tvecs.get(i, 0)[0]);
            tvec.put(1, 0, tvecs.get(i, 0)[1]);
            tvec.put(2, 0, tvecs.get(i, 0)[2]);

            // Camera coordinates can be obtained using 'camera position = - rot_matrix.inverse() * tvec'
            // Rotation matrix can be obtained by Rodrigues function
            Calib3d.Rodrigues(rvec, rotMatrix);
            //rotMatrix= rotMatrix.t();
            rotMatrix = rotMatrix.inv();
            Core.multiply(rotMatrix, Scalar.all(-1), rotMatrix);
            Core.gemm(rotMatrix, tvec, 1, tvec, 0, tempResult, 0);
            Core.add(result, tempResult, result);
            //debugMatrix("result",result);
        }
        Core.divide(result, Scalar.all(len), result);
        return result;


    }
}

