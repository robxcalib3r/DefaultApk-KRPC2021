package jp.jaxa.iss.kibo.rpc.defaultapk;

//JAXA LIBRARY
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

//ZXING library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.LuminanceSource;
import com.google.zxing.qrcode.QRCodeReader;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

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
        Log.i("Mission", "Mission started");

        //move to Point A
        moveToWrapper(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707);

        String pos_A_prime = qrScan();      //scanning the QR
        api.sendDiscoveredQR(pos_A_prime);      //Sending to judge the read string


        Double KOZ_pattern = S2D(extractor(pos_A_prime, "\"p\":(.*?),"));
        Double pos_x = S2D(extractor(pos_A_prime, "\"x\":(.*?),"));
        Double pos_y = S2D(extractor(pos_A_prime, "\"y\":(.*?),"));
        Double pos_z = S2D(extractor(pos_A_prime, "\"z\":(.*?)\\}"));

        //Move to point A prime
        moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);

        //Aruco AR Tag reading starts from here
        Mat img = api.getMatNavCam();
        //Core.rotate(img, img, Core.ROTATE_180);
        //Core.flip(img, img, 1);
        Mat markerIDs = new Mat();
        List<Mat> markerCorners = new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters param = DetectorParameters.create();
        List<Mat> rejectedImgpoints = new ArrayList<>();
        double[][] camIntrinsics = api.getNavCamIntrinsics();

        double[] camMat = camIntrinsics[0];
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32F);
        cameraMatrix.put(0, 0, camMat);

        double[] distDouble = camIntrinsics[1];
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32F);
        distCoeffs.put(0, 0, distDouble);


        Aruco.detectMarkers(img, dictionary, markerCorners, markerIDs, param, rejectedImgpoints, cameraMatrix, distCoeffs);

        for(Mat eachArr : markerCorners) {
            String dump = eachArr.dump();
            Log.d(TAG, dump);
        }

        // List of detected tags
        List<Tag> tags = new ArrayList<>();

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();

        if (markerIDs.rows() > 0) {

            float markerLen = 0.05f;

            Aruco.estimatePoseSingleMarkers(markerCorners, markerLen, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < markerIDs.rows(); i++) {
                Tag tag;

                // Individual vectors for the tags
                double[] tagRvecs = new double[3];
                double[] tagTvecs = new double[3];

                tagRvecs[0] = rvecs.get(i, 0)[0];
                tagRvecs[1] = rvecs.get(i, 0)[1];
                tagRvecs[2] = rvecs.get(i, 0)[2];
                tagTvecs[0] = tvecs.get(i, 0)[0];
                tagTvecs[1] = tvecs.get(i, 0)[1];
                tagTvecs[2] = tvecs.get(i, 0)[2];

                tag = new Tag(markerCorners.get(i), markerIDs.get(i, 0)[0], tagRvecs, tagTvecs);
                tags.add(i, tag);
            }

        }

        //TagListener(tags);

        Log.i("Log_RVEC", rvecs.dump());
        Log.i("LOG_TVEC", tvecs.dump());


        Log.i("current Position", api.getRobotKinematics().getPosition().toString());
        Log.i("current Orientation", api.getRobotKinematics().getOrientation().toString());


        double[] midpoint = getMidPoint(tags);
        double[] q_mid = rot2point(midpoint[0], midpoint[1], midpoint[2]);

        Quaternion q_2 = new Quaternion((float)q_mid[0], (float)q_mid[1], (float)q_mid[2],(float)q_mid[3]);

        Point cord = new Point();
        //Point cord = tagGetPoint(tags);
        api.relativeMoveTo(cord, q_2, true );


        try{
            api.flashlightControlFront(1);
            api.laserControl(true);

            api.takeSnapshot();
            api.flashlightControlFront((0));
            api.laserControl(false);
        }
        catch (Exception e){
            Log.e("Error in Post-laser cond", e.toString());
        }

        moveToWrapper(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707);
        moveToWrapper(10.505, -8.65, 4.7,0, 0, -0.707, 0.707);

        moveToWrapper(10.6, -8.0, 4.5, 0, 0, -0.707, 0.707);

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
        if(result.hasSucceeded()){
            Log.i("arrival bee", "Success");
        }
        else {
            Log.i("arrival bee", "Failed");
        }
    }

    private String qrScan() {            //Seperate QR scanning function with redundancy enabled
        final int LOOP_MAX = 3;
        com.google.zxing.Result qrcode = null;
        int loopCounter = 0;
        while (qrcode == null || loopCounter < LOOP_MAX) {
            api.flashlightControlFront((float)0.5);
            Bitmap img = api.getBitmapNavCam();
            final int w = img.getWidth(), h = img.getHeight();
            Bitmap bm = Bitmap.createBitmap(img, w/2-50, h/2,
                    w/4, w/4);
            int[] intB1 = new int[w * h];
            bm.getPixels(intB1, 0, w, 0, 0, w, h);
            LuminanceSource source = new RGBLuminanceSource(w, h, intB1);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            try {
                qrcode = new QRCodeReader().decode(bitmap);
            } catch (Exception e) {
                qrcode = null;
                Log.e("qrCode error", e.toString());
            }
            ++loopCounter;
        }
        api.flashlightControlFront(0);
        Log.i("qrCode", "QR done");
        return qrcode.getText();
    }


    private String extractor(String data, String pattern) {
        Pattern ptrn = Pattern.compile(pattern);
        Matcher matcher = ptrn.matcher(data);
        if (matcher.find()) {
            data = matcher.group(1);
        }
        Log.i("String pattern", "Successfully extracted");
        return data;
    }

    private double S2D(String s) {
        return Double.parseDouble(s);
    }


    /*
    private void TagListener(List<Tag> tags){
        for (Tag tag : tags){
            Log.i("id", tag.getId() + "");
            double[] q = tag.getQuaternion();
            Log.i("quaternion_0", String.valueOf(q[0]));
            Log.i("quaternion_1", String.valueOf(q[1]));
            Log.i("quaternion_2", String.valueOf(q[2]));
            Log.i("quaternion_3", String.valueOf(q[3]));
            Log.i("corners", tag.getCorners().dump());
            Log.i("rvec_0", String.valueOf(tag.getRvecs()[0]));
            Log.i("rvec_1", String.valueOf(tag.getRvecs()[1]));
            Log.i("rvec_2", String.valueOf(tag.getRvecs()[2]));
            Log.i("tvec_0", String.valueOf(tag.getTvecs()[0]));
            Log.i("tvec_1", String.valueOf(tag.getTvecs()[1]));
            Log.i("tvec_2", String.valueOf(tag.getTvecs()[2]));

        }
    } */


    public double[] getMidPoint(List<Tag> tags){
        double temp_evenX = 0, temp_oddX = 0,
                temp_evenY = 0, temp_oddY = 0,
                temp_evenZ = 0, temp_oddZ = 0;
        for (Tag tag : tags){
            if(tag.getId() % 2 == 0){
                temp_evenX += tag.getTvecs()[0];
                temp_evenY += tag.getTvecs()[1];
                temp_evenZ += tag.getTvecs()[2];
            }
            else{
                temp_oddX += tag.getTvecs()[0];
                temp_oddY += tag.getTvecs()[1];
                temp_oddZ += tag.getTvecs()[2];
            }
        }

        temp_evenX /= 2;
        temp_evenY /= 2;
        temp_evenZ /= 2;

        temp_oddX /= 2;
        temp_oddY /= 2;
        temp_oddZ /= 2;

        double x = (temp_evenX + temp_oddX) / 2;
        double y = (temp_evenY + temp_oddY) / 2;
        double z = (temp_evenZ + temp_oddZ) / 2;

        double[] point = new double[]{x - 0.075,y + 0.075,z};
        Log.i("getMidPoint", Arrays.toString(point));
        return point;

    }

    public double[] rot2point(double x, double y, double z){
        double hype = sqrt(x*x + z*z);
        double hype_1 = sqrt(hype*hype + y*y);
        double[][] Rz = {{z/hype, -x/hype,  0}, {x/hype, z/hype, 0}, {0,0,1}};
        double[][] Rx = {{1,0,0}, {0, hype/hype_1, y/hype_1}, {0, -y/hype_1, hype/hype_1}};
        double[][] flip = {{0,1,0}, {-1,0,0}, {0,0,1}};
        double[] q = new double[4];
        Mat r = new Mat(3, 3, CvType.CV_64F);
        Mat RZ = new Mat(3, 3, CvType.CV_64F);
        Mat RX = new Mat(3, 3, CvType.CV_64F);
        Mat r_flip = new  Mat(3, 3, CvType.CV_64F);

        for (int i = 0; i<3; i++){
            for (int j = 0; j<3; j++){
                RZ.put(i, j, Rz[i][j]);
                RX.put(i, j, Rx[i][j]);
                r_flip.put(i, j, flip[i][j]);
            }
        }

        Core.gemm(RZ, RX, 1, new Mat(),0, r);
        Core.gemm(r, r_flip, 1, new Mat(), 0, r);

        double trace = r.get(0, 0)[0] +
                r.get(1, 1)[0] +
                r.get(2, 2)[0];
//
        if (trace > 0.0) {
            double s = sqrt(trace + 1.0);
            q[3] = (s * 0.5);
            s = 0.5 / s;
            q[0] = ((r.get(2, 1)[0] - r.get(1, 2)[0]) * s);
            q[1] = ((r.get(0, 2)[0] - r.get(2, 0)[0]) * s);
            q[2] = ((r.get(1, 0)[0] - r.get(0, 1)[0]) * s);
        } else {
            int i = r.get(0, 0)[0] < r.get(1, 1)[0] ?
                    (r.get(1, 1)[0] < r.get(2, 2)[0] ? 2 : 1) :
                    (r.get(0, 0)[0] < r.get(2, 2)[0] ? 2 : 0);
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;
//
            double s = sqrt(r.get(i, i)[0] - r.get(j, j)[0] - r.get(k, k)[0] + 1.0);
            q[i] = s * 0.5;
            s = 0.5 / s;
//
            q[3] = (r.get(k, j)[0] - r.get(j, k)[0]) * s;
            q[j] = (r.get(j, i)[0] + r.get(i, j)[0]) * s;
            q[k] = (r.get(k, i)[0] + r.get(i, k)[0]) * s;
        }
        Log.i("rot2Point", Arrays.toString(q));
        return q;
    }


}

