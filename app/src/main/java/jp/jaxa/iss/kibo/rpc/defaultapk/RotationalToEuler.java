package jp.jaxa.iss.kibo.rpc.defaultapk;

import org.opencv.core.Mat;

public class RotationalToEuler {
    public static double[] MatrixToYawPitchRoll( Mat A )
    {
        double[] angle = new double[3];
        angle[1] = -Math.asin( A.get(2,0)[0] );  //Pitch

        //Gymbal lock: pitch = -90
        if( A.get(2,0)[0] == 1 ){
        angle[0] = 0.0;             //yaw = 0
        angle[2] = Math.atan2( -A.get(0,1)[0], -A.get(0,2)[0] );    //Roll
//            System.out.println("Gimbal lock: pitch = -90");
        }

        //Gymbal lock: pitch = 90
        else if( A.get(2,0)[0] == -1 ){
        angle[0] = 0.0;             //yaw = 0
        angle[2] = Math.atan2( A.get(0,1)[0], A.get(0,2)[0] );    //Roll
//            System.out.println("Gimbal lock: pitch = 90");
        }
        //General solution
        else{
        angle[0] = Math.atan2(  A.get(1,0)[0], A.get(0,0)[0] );
        angle[2] = Math.atan2(  A.get(2,1)[0], A.get(2,1)[0] );
//            System.out.println("No gimbal lock");
        }
        return angle;   //Euler angles in order yaw, pitch, roll
    }
}
