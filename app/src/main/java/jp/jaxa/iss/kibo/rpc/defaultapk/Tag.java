package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import static java.lang.Math.sqrt;

public class Tag {
    private Mat corners;
    private int id;
    private double[] rvecs;
    private double[] tvecs;

    public Tag(Mat corners, double id, double[] rvecs, double[] tvecs) {
        this.corners = corners;
        this.id = (int) id;
        this.rvecs = rvecs;
        this.tvecs = tvecs;
    }

    public int getId() {
        return id;
    }

    public double[] getRvecs() {
        return rvecs;
    }

    public double[] getTvecs() {
        return tvecs;
    }

    public Mat getCorners() { return corners; }

    public Mat getRotMat(){
        Mat rvec = new Mat(1, 3, CvType.CV_64F),
                r = new Mat(3, 3, CvType.CV_64F);
        rvec.put(0, 0, rvecs);
        Calib3d.Rodrigues(rvec, r);
        return r;
    }

    //public double[] getCoOrd(){

    //}


    public double[] getQuaternion() {
        double[] q = new double[4];
        Mat rvec = new Mat(1, 3, CvType.CV_64F),
                rv = new Mat(3, 3, CvType.CV_64F),
                r_flip = new Mat(3, 3, CvType.CV_64F),
                r_flip_1 = new Mat(3, 3, CvType.CV_64F);
        Mat r = new Mat(3, 3, CvType.CV_64F);

        rvec.put(0, 0, rvecs);
        Calib3d.Rodrigues(rvec, rv);
        Log.i("rodrigues", rv.dump());

        double[][] flip = {{0,1,0}, {-1,0,0}, {0,0,1}};
        double[][] flip_1 = {{1,0,0}, {0,0,-1}, {0,1,0}};
        for (int i = 0; i<3; i++){
            for (int j = 0; j<3; j++){
                r_flip.put(i, j, flip[i][j]);
                r_flip_1.put(i, j, flip_1[i][j]);
            }
        }


        Core.gemm(r_flip, rv, 1, new Mat(),0, r);   //to get camera rotation
        Log.i("multiply_rotMat", r.dump());
        //this.tvecs = -r * tvecs;

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
            int i = r.get(0,0)[0] < r.get(1,1)[0] ?
                    (r.get(1,1)[0] < r.get(2,2)[0] ? 2 : 1) :
                    (r.get(0,0)[0] < r.get(2,2)[0] ? 2 : 0);
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;
//
            double s = sqrt(r.get(i, i)[0] - r.get(j,j)[0] - r.get(k,k)[0] + 1.0);
            q[i] = s * 0.5;
            s = 0.5 / s;
//
            q[3] = (r.get(k,j)[0] - r.get(j,k)[0]) * s;
            q[j] = (r.get(j,i)[0] + r.get(i,j)[0]) * s;
            q[k] = (r.get(k,i)[0] + r.get(i,k)[0]) * s;
        }
        return q;
    }

}
