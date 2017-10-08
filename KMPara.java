package jxl.LocalLocateCore.kalman;

public class KMPara {
	static{ 
        System.loadLibrary("KMPara");
    }
	public static native double[][] getQ();
	public static native double[][] getR();
}
