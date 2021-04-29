package jp.jaxa.iss.kibo.rpc.defaultapk;

//JAXA LIBRARY
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

import java.util.LinkedList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import android.graphics.Bitmap;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
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
        MatOfInt markerIDs = new MatOfInt();
        LinkedList<Mat> markerCorners = new LinkedList<>();
        Dictionary dictionary= Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        //DetectorParameters param = DetectorParameters.create();


        Aruco.detectMarkers(img, dictionary, markerCorners, markerIDs); //, param);
        double[] data =  markerIDs.get(0, 0);

        //sendData(MessageType.STRING, "data", String.valueOf(data));


    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    // Our Own methods or Functions

    //for moving to a certain coordinate including rotation
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private String qrScan(){            //Seperate QR scanning function with redundancy enabled
        final int LOOP_MAX = 3;
        boolean succeed = false;
        com.google.zxing.Result qrcode = null;
        int loopCounter = 0;
        while (!succeed || loopCounter < LOOP_MAX){
            Bitmap bm = api.getBitmapNavCam();
            int[] intB1 = new int[bm.getWidth()*bm.getHeight()];
            bm.getPixels(intB1, 0 , bm.getWidth(), 0, 0, bm.getWidth(), bm.getHeight());
            LuminanceSource source = new RGBLuminanceSource(bm.getWidth(), bm.getHeight(), intB1);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            try{
                qrcode = new MultiFormatReader().decode(bitmap);
                succeed = true;
            }
            catch(Exception e){}
            ++loopCounter;
        }
        return qrcode.getText();
    }

    private String extractor(String data, String pattern){
        Pattern ptrn = Pattern.compile(pattern);
        Matcher matcher = ptrn.matcher(data);
        if (matcher.find())
        {
            data = matcher.group(1);
        }

        return data;
    }

    private double S2D(String s){
        return Double.parseDouble(s);
    }

}

