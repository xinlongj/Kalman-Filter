package jxl.LocalLocateCore.kalman;

import jxl.LocalLocateCore.jama.Matrix;

public class LocateKalmanFilter {
	/**
	 * init Locate Kalman Filter
	 */
	public LocateKalmanFilter() {
		try {
			jKalman = new JKalman(dp, mp);
			
			Q = KMPara.getQ();
			R = KMPara.getR();
			
			predictX = new Matrix(dp, 1);
			correctionX = new Matrix(dp, 1);
			measurementZ = new Matrix(mp, 1);

			transition_matrix = new Matrix(A);
			measurement_matrix = new Matrix(H);
			process_noise_cov = new Matrix(Q);
			measurement_noise_cov = new Matrix(R);
			error_cov_post = new Matrix(err_cov_post_init);

			jKalman.setTransition_matrix(transition_matrix);
			jKalman.setMeasurement_matrix(measurement_matrix);
			jKalman.setProcess_noise_cov(process_noise_cov);
			jKalman.setMeasurement_noise_cov(measurement_noise_cov);
			jKalman.setError_cov_post(error_cov_post);
			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void resetFilter() {
		this.count = 0;
		this.A[0][2] = 1.0f;
		this.A[1][3] = 1.0f;
	}

	public void setTransition(double[][] A) {
		transition_matrix = new Matrix(A);
		jKalman.setTransition_matrix(transition_matrix);
		this.A = A;
	}

	/**
	 * @param state_post
	 *            dp x 1 dim array [x,y,Vx,Vy]
	 */
	public void setStatePost(double[][] state_post) {
		jKalman.setState_post(new Matrix(state_post));
	}

	/**
	 * 设置过程激励噪声协方差矩阵
	 * 
	 * @param Q
	 *            过程激励噪声协方差矩阵
	 */
	public void setProcessNoiseCov(double[][] Q) {
		process_noise_cov = new Matrix(Q);
		jKalman.setProcess_noise_cov(process_noise_cov);
		this.Q = Q;
	}

	/**
	 * 设置观测噪声协方差矩阵
	 * 
	 * @param R
	 *            观测噪声协方差矩阵
	 */
	public void setMeasurementNoiseCov(double[][] R) {
		measurement_noise_cov = new Matrix(R);
		jKalman.setMeasurement_noise_cov(measurement_noise_cov);
		this.R = R;
	}

	/**
	 * @param z
	 *            mp x 1 dim array
	 * @return x(k)
	 */
	public Matrix Correct(double[][] z) {
		this.count++;
		if (count == 1) { // 初始化估计值
			Matrix statePostMatrix = new Matrix(4, 1);
			statePostMatrix.set(0, 0, z[0][0]);
			statePostMatrix.set(1, 0, z[1][0]);
			statePostMatrix.set(2, 0, 0);
			statePostMatrix.set(3, 0, 0);
			jKalman.setState_post(statePostMatrix);

			return statePostMatrix;
		}

		measurementZ = new Matrix(z);

		predictX = jKalman.Predict();
		correctionX = jKalman.Correct(measurementZ);

		return correctionX;
	}

	private int count = 0;
	/** number of state vector dimensions */
	private int dp = 4;
	/** number of measurement vector dimensions */
	private int mp = 2;

	// 转换矩阵A
	private double[][] A = { { 1, 0, 1, 0 }, { 0, 1, 0, 1 }, { 0, 0, 1, 0 },
			{ 0, 0, 0, 1 } };

	private double[][] H = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 } };

	// 过程激励噪声协方差矩阵 var[x,y,Vx,Vy]
	private double[][] Q = null/* = {
			{ 80.8750483209580f, 32.1436412098060f, 24.4212906344850f, 7.27126818822454f },
			{ 32.1436412098060f, 76.4473738523203f, 7.27126818822454f, 21.8344526327883f },
			{ 24.4212906344850f, 7.27126818822454f, 7.88367705060713f, 1.92775984544685f },
			{ 7.27126818822454f, 21.8344526327883f, 1.92775984544685f, 6.91148654062645f }}*/;
	// 观测噪声协方差矩阵 var[x,y]
	private double[][] R = null/* = { { 12941.7817201094f, -2562.02090990456f },
		       { -2562.02090990456f, 8369.80364536069f } }*/;

	// P(k)的初始值,0方阵 --->动态定位中的卡尔曼滤波研究P21
	private double[][] err_cov_post_init = { { 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f } };

	/** measurement state: z(k) = H*x(k) + v(k) */
	private Matrix measurementZ;
	/*** predicted state: x'(k)=A*x(k-1)+B*u(k) ***/
	private Matrix predictX;
	/*** corrected state: x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) ***/
	private Matrix correctionX;
	/** state transition matrix (A) */
	private Matrix transition_matrix;
	/** measurement matrix (H) */
	private Matrix measurement_matrix;
	/** process noise covariance matrix (Q) */
	private Matrix process_noise_cov;
	/** measurement noise covariance matrix (R) */
	private Matrix measurement_noise_cov;
	/** priori error estimate covariance matrix(P'(k)): P'(k)=A*P(k-1)*At+Q) */
	private Matrix error_cov_pre;
	/** Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R) */
	private Matrix gain_K;
	/**
	 * posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)
	 */
	private Matrix error_cov_post;

	private JKalman jKalman;
}
