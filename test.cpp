#define dt 0.039691
#define pi 3.141592
#include "math.h"
float distance[2];
float phi_d_ref = 0;
float theta_dr = 0;
float eta_dr[3];
float p_2dot_dr;
float* meter_conv(float lat1, float lon1, float lat2, float lon2, float* distance);
float* euler_conv(float eta_d, float p_2dot_d[]);
float* torque_conv(float eta[], float eta_ref[]);
float force_conv(float eta[], float p_2dot_ref);
float * time_int(float t_d[], float v_dot_d);
//float* linspace(float* missionway1, float*missionway2, int n);
float now_xyz[3] = { 0,0,0 };
float bef_xyz[3];
float pre_err_xyz_dot[3];
float p_2dot_d[3];
float eta[3];
float bef_eta[3];
float pre_err_eta_dot[3];
float eta_ref[3];
float err_xyz[3];
float err_xyz_dot[3];
float p_dot[3];
float v_dot_d;
// 지상
float way0_lat = 37.603825;
float way0_lon = 126.865642;
float way0_al = 0;

// 공중
float way1_lat = 37.603825;
float way1_lon = 126.865642;
float way1_al = 150;

// way 1
float way2_lat = 37.602;
float way2_lon = 126.864;
float way2_al = 150;

// way2
float way3_lat = 37.6056;
float way3_lon = 126.8604;
float way3_al = 150;

float way4_lat = 37.602;
float way4_lon = 126.8584;
float way4_al = 150;

float way5_lat = 37.5996;
float way5_lon = 126.8616;
float way5_al = 150;

float way6_lat = 37.603825;
float way6_lon = 126.865642;
float way6_al = 150;


float way7_lat = 37.603825;
float way7_lon = 126.865642;
float way7_al = 0;

float lat = 0;
float lon = 0;
float eta_d = 1;
int p = 0;
float now_xyz[3];
float bef_xyz[3];

float Ta;
float Tb;
float Tc;
float Td;
float missionway[24] = { way0_lat, way0_lon, way0_al,
way1_lat, way1_lon, way1_al,
way2_lat, way2_lon, way2_al,
way3_lat, way3_lon, way3_al,
way4_lat, way4_lon, way4_al,
way5_lat, way5_lon, way5_al,
way6_lat, way6_lon, way6_al,
way7_lat, way7_lon, way7_al
};
float sub_waypoint[2100];//0to6
float mid_waypoint[2100];

// 경로점1 37.602N 126.864E
//경로점2 37.6056N 126.8604E
//경로점3 37.602N 126.8584E
//경로점4 37.5996N 126.8616E
//홈 (본부석) 37.603825N 126.865642E
float dis_err = 100000;
//PID 제어값
float I_gain[3];
float P_gain[3] = { 2,2,2 };
float I[3] = { 0,0,0 };
float P_gain2_1[3] = { 4.750,4.750,0 };

float I_gain2[3] = { 0.440, 0.440, 0.650 };
float P_gain2[3] = { 2.833, 2.933,3.325 };
float I2[3] = { 0,0,0 };

float Kt = 1;
float Kd = 0.2;
float l = 0.3;
float m = 2.2;
float g[3] = { 0, 0, -9.81 };
float lv[9] = { 0.0170, 0, 0, 0, 0.0170, 0, 0, 0, 0.0230 };
float lr[9] = { 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001 };

float err_eta[3];
float err_eta_dot[3];
float eta_dot[3];
float eta_ref_check[21];

float t_d[3];
void setup() {
	// put your setup code here, to run once:
	int i = 0;
	int j = 0;
	for (i = 1; i<9; i++) {
		int a = 0;
		a = sqrt((missionway[i * 3] - missionway[0])*(missionway[i * 3] - missionway[0]) + (missionway[i * 3 + 1] - missionway[1])*(missionway[i * 3 + 1] - missionway[1]));

		if (a != 0) {
			meter_conv(missionway[i * 3], missionway[i * 3 + 1], missionway[0], missionway[1]);
		}
		else {
			distance[0] = 0;
			distance[1] = 0;
		}

		missionway[i * 3] = distance[0];
		missionway[i * 3 + 1] = distance[1];
		}
		//첫번째 missionway 초기화
		missionway[0] = 0;
		missionway[1] = 0;
		missionway[2] = 0;

		now_xyz[0] = missionway[0];
		now_xyz[1] = missionway[1];
		now_xyz[2] = missionway[2];
		bef_xyz[0] = now_xyz[0];
		bef_xyz[1] = now_xyz[1];
		bef_xyz[2] = now_xyz[2];
		for (p = 0; p<7; p++) {
			int j = 0;
			float missionway_1 = (missionway[3] - missionway[0]) / 100;
			float missionway_2 = (missionway[6] - missionway[3]) / 100;
			float missionway_3 = (missionway[9] - missionway[6]) / 100;
			float missionway_4 = (missionway[12] - missionway[9]) / 100;
			float missionway_5 = (missionway[15] - missionway[12]) / 100;
			float missionway_6 = (missionway[18] - missionway[15]) / 100;
			float missionway_7 = (missionway[21] - missionway[18]) / 100;
			//subpoint 나중에하기

			for (i = 0; i<100; i++) {
				mid_waypoint[i * 3] = sub_waypoint
					mid_waypoint[i * 3 + 1] = sub_waypoint
					mid_waypoint[i * 3 + 2] = subwaypoint

					dis_err = 100000;

				while (dis_err>0.1) {

					err_xyz[0] = mid_waypoint[0] - now_xyz[0];
					err_xyz[1] = mid_waypoint[1] - now_xyz[1];
					err_xyz[2] = mid_waypoint[2] - now_xyz[2];

					p_dot[0] = (now_xyz[0] - bef_xyz[0]) / dt;
					p_dot[1] = (now_xyz[1] - bef_xyz[1]) / dt;
					p_dot[2] = (now_xyz[2] - bef_xyz[2]) / dt;

					err_xyz_dot[0] = err_xyz[0] - p_dot[0];
					err_xyz_dot[1] = err_xyz[1] - p_dot[1];
					err_xyz_dot[2] = err_xyz[2] - p_dot[2];

					for (int z = 0; z < 3; z++) {
						I[z] = (err_xyz_dot[z] + pre_err_xyz_dot[z])*I_gain[z] * 0.5*dt;
						p_2dot_d[z] = err_xyz_dot[z] * P_gain[z] + I[z];
						pre_err_xyz_dot[z] = err_xyz_dot[z]; 
					}

					float* answer2;
					answer2 = euler_conv(eta_d, p_2dot_d);

					eta_dr[0] = *(answer2);
					eta_dr[1] = *(answer2 + 1);
					eta_dr[2] = *(answer2 + 2);
					p_2dot_dr = *(answer2 + 3);

					
					err_eta[0] = eta_dr[0] - eta[0];
					err_eta[1] = eta_dr[1] - eta[1];
					err_eta[2] = eta_dr[2] - eta[2];

					err_eta[0] = P_gain2_1[0] * err_eta[0];
					err_eta[1] = P_gain2_1[1] * err_eta[1];
					err_eta[2] = err_eta[2];

					eta_dot[0] = (eta[0] - bef_eta[0]) / dt;
					eta_dot[1] = (eta[1] - bef_eta[1]) / dt;
					eta_dot[2] = (eta[2] - bef_eta[2]) / dt;

					err_eta_dot[0] = (err_eta[0]) - eta_dot[0];
					err_eta_dot[1] = (err_eta[1]) - eta_dot[1];
					err_eta_dot[2] = (err_eta[2]) - eta_dot[2];

					for (int nn = 0; nn < 3; nn++) {
						I2[nn] = (err_eta_dot[nn] + pre_err_eta_dot[nn])*I_gain2[nn] * 0.5*dt;
						eta_ref[nn] = err_eta_dot[nn] * P_gain2[nn] + I2[nn];
						pre_err_eta_dot[nn] = err_eta_dot[nn];
					}

					//한번 다시 생각해보기
					eta_ref_check[3 * j] = eta_ref[0];
					eta_ref_check[3 * j + 1] = eta_ref[1];
					eta_ref_check[3 * j + 2] = eta_ref[2];

					float* answer3;
					answer3 = torque_conv(eta, eta_ref);
					
					t_d[0] = (*answer3);
					t_d[1] = (*answer3 + 1);
					t_d[2] = (*answer3 + 2);

					float answer4;
					answer4 = force_conv(eta, p_2dot_dr);

					v_dot_d = answer4;

					float* answer5;
					answer5 = time_int(t_d, v_dot_d);

					float sphi = sin(eta[0]);
					float cphi = cos(eta[0]);
					float stht = sin(eta[1]);
					float ctht = cos(eta[1]);
					float spsi = sin(eta[2]);
					float cpsi = cos(eta[2]);

					Ta = (-0.0378*cphi - 0.02875*sphi)*eta_ref[1] + (-0.0378*sphi*ctht + 0.0285*cphi*ctht)*eta_ref[2] + 0.3*v_dot_d;
					Tb = -0.0378*eta_ref[0] + (0.02875*sphi)*eta_ref[1] + (0.0378*stht - 0.0285*cphi*ctht)*eta_ref[2] + 0.3*v_dot_d;
					Tc = (0.0378*cphi - 0.0285*sphi)*eta_ref[1] + (0.0378*sphi*ctht + 0.0285*cphi*ctht)*eta_ref[2] + 0.3*v_dot_d;
					Td = 0.03781*eta_ref[0] + (-0.0378*stht + 0.02875*sphi)*eta_ref[1] + (-0.02875*cphi*ctht)*eta_ref[2] + 0.3*v_dot_d;
				}
			}
		}
	}

	void loop() {
		// put your main code here, to run repeatedly:

	}

	float* meter_conv(float lat1, float lon1, float lat2, float lon2, float*distance) {
		//UNTITLED 이 함수의 요약 설명 위치
		//자세한 설명 위치
		 float R = 6378.137;

		float dLat = (lat2 - lat1) * PI / 180;
		float dLon = (lon2 - lon1) * PI / 180;
		float meter = R * 1000;
		float heading = atan(dLon / dLat);
		distance[0] = meter*dLat;
		distance[1] = meter*dLon;
		return distance;
	}

	float*euler_conv(float eta_d, float p_2dot_d[]) {

		float psi_d = eta_d;
		float d;
		float x_2dot_d = p_2dot_d[0];
		float y_2dot_d = p_2dot_d[1];
		float z_2dot_d = p_2dot_d[2] + 9.81;

		d = sqrt(x_2dot_d * x_2dot_d + y_2dot_d * y_2dot_d + z_2dot_d * z_2dot_d);

		if (-0.001 < d && d < 0.001) {
			phi_d_ref = 0;
		}
		else {
			phi_d_ref = asin((x_2dot_d * sin(psi_d) - y_2dot_d * cos(psi_d)) / d);
		}

		theta_dr = atan2(x_2dot_d * cos(psi_d) + y_2dot_d * sin(psi_d), z_2dot_d);

		float min_ang = -3.14 / 6;//-PI/6
		float max_ang = 3.14 / 6;//PI/6

		if (phi_d_ref < min_ang) {
			phi_d_ref = min_ang;
		}
		if (phi_d_ref > max_ang) {
			phi_d_ref = max_ang;
		}

		if (theta_dr < min_ang) {
			theta_dr = min_ang;
		}
		if (theta_dr > max_ang) {
			theta_dr = max_ang;
		}

		float eta_dr[3] = { phi_d_ref, theta_dr, psi_d };

		float result[] = { eta_dr[0], eta_dr[1], eta_dr[2], z_2dot_d };

		return result;
	}
	float* torque_conv(float eta[], float eta_ref[]) {

		float answer[3];
		float sphi = sin(eta[0]);
		float cphi = cos(eta[0]);
		float stht = sin(eta[1]);
		float ctht = cos(eta[1]);
		float spsi = sin(eta[2]);
		float cpsi = cos(eta[2]);

		float C[9] = { 1, 0, -(stht), 0, cphi, (sphi * ctht), 0, -(sphi), (cphi
			* ctht) };

		for (int i = 0; i < 3; i++) {
			float s1, s2, s3, s4, s5, s6, s7, s8, s9;
			s1 = C[0] * lv[0] + C[1] * lv[3] + C[2] * lv[6];
			s2 = C[0] * lv[1] + C[1] * lv[4] + C[2] * lv[7];
			s3 = C[0] * lv[2] + C[1] * lv[5] + C[2] * lv[8];
			s4 = C[3] * lv[0] + C[4] * lv[3] + C[5] * lv[6];
			s5 = C[3] * lv[1] + C[4] * lv[4] + C[5] * lv[7];
			s6 = C[3] * lv[2] + C[4] * lv[5] + C[5] * lv[8];
			s7 = C[6] * lv[0] + C[7] * lv[3] + C[8] * lv[6];
			s8 = C[6] * lv[1] + C[7] * lv[4] + C[8] * lv[7];
			s9 = C[6] * lv[2] + C[7] * lv[5] + C[8] * lv[8];
			answer[0] = s1 * eta_ref[0] + s2 * eta_ref[1] + s3 * eta_ref[2];
			answer[1] = s4 * eta_ref[0] + s5 * eta_ref[1] + s6 * eta_ref[2];
			answer[2] = s7 * eta_ref[0] + s8 * eta_ref[1] + s9 * eta_ref[2];

		}
		float t[3] = { 0, 0, 0 };
		t[0] = answer[0];
		t[1] = answer[1];
		t[2] = answer[2];

		return t;


	}
	float force_conv(float eta[], float p_2dot_ref) {
		float sphi = sin(eta[0]);
		float cphi = cos(eta[0]);
		float stht = sin(eta[1]);
		float ctht = cos(eta[1]);
		float spsi = sin(eta[2]);
		float cpsi = cos(eta[2]);

		float R[9];
		R[0] = cpsi * ctht;
		R[1] = cpsi * stht * sphi - spsi * cphi;
		R[2] = cpsi * stht * cphi + spsi * sphi;
		R[3] = spsi * ctht;
		R[4] = spsi * stht * sphi + cpsi * cphi;
		R[5] = spsi * stht * cphi - cpsi * sphi;
		R[6] = -stht;
		R[7] = ctht * sphi;
		R[8] = ctht * cphi;

		float u[3] = { 0, 0, p_2dot_ref };
		int i, j;
		float d[3] = { 0, 0, 0 };

		d[0] = R[0] * u[0] + R[1] * u[1] + R[2] * u[2];
		d[1] = R[3] * u[0] + R[4] * u[1] + R[5] * u[2];
		d[2] = R[6] * u[0] + R[7] * u[1] + R[8] * u[2];

		float v_dot_d = d[2];
		return v_dot_d;
	}

	float * time_int(float t_d[], float v_dot_d) {

		float r = Kd;

		float T[16] = { 1 / m, 1 / m, 1 / m, 1 / m, 0, -l, 0, l, -l, 0, l, 0, r, -r,
			r, -r };

		float u[4] = { v_dot_d, t_d[0], t_d[1], t_d[2] };

		float invT[16] = { 1 / m, 0, -l, r, 1 / m, -l, 0, -r, 1 / m, 0, l, r, 1 / m,
			l, 0, -r };

		float f[4];

		f[0] = invT[0] * u[0] + invT[1] * u[1] + invT[2] * u[2] + invT[3] * u[3];
		f[1] = invT[4] * u[0] + invT[5] * u[1] + invT[6] * u[2] + invT[7] * u[3];
		f[2] = invT[8] * u[0] + invT[9] * u[1] + invT[10] * u[2] + invT[11] * u[3];
		f[3] = invT[12] * u[0] + invT[13] * u[1] + invT[14] * u[2]
			+ invT[15] * u[3];

		//limits min and max force from rotor
		//because motor can't make unlimited force

		float min_f = 0;
		float max_f = 10;

		if (f[0] < min_f) {
			f[0] = min_f;
		}
		if (f[1] < min_f) {
			f[1] = min_f;
		}
		if (f[2] < min_f) {
			f[2] = min_f;
		}
		if (f[3] < min_f) {
			f[3] = min_f;
		}

		if (f[0] > max_f) {
			f[0] = max_f;
		}
		if (f[1] > max_f) {
			f[1] = max_f;
		}
		if (f[2] > max_f) {
			f[2] = max_f;
		}
		if (f[3] > max_f) {
			f[3] = max_f;
		}

		return f;
	}

	
	//float* linspace(float* missionway1, float*missionway2){
	//float x[100];
	//for (i=0; i<100; i++)
	//{  
	//  x[i]=*missionway1+(*missionway2-*missionway1)*i/100;
	//}

	// return x; 
	//}


