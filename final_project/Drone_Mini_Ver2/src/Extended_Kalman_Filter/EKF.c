#include "EKF.h"


static Extended_Kalman_t ekf;

void kalman_init(void){
    ekf.z_predict = 0.0f;
    ekf.vz_predict = 0.0f;
    ekf.P[0][0] = 0.1f; 
    ekf.P[0][1] = 0.0f;
    ekf.P[1][0] = 0.0f; 
    ekf.P[1][1] = 0.1f;

    ekf.Q_accel = 0.05f;
    ekf.R_tof = 0.01f;
    ekf.R_bmp = 0.25f;

}

void kalman_predict_vl53(float acce_z_earth , float dt){
    // --- BƯỚC 1A: Dự đoán Trạng thái X ---
    ekf.z_predict += ekf.vz_predict*dt + 0.5f*acce_z_earth*dt*dt;
    ekf.vz_predict += acce_z_earth*dt;

    // --- BƯỚC 1B: Dự đoán Ma trận Sai số P (P = F*P*F' + Q) ---
    // Ma trận F = [1, dt; 0, 1]
    float P00_prev = ekf.P[0][0];
    float P01_prev = ekf.P[0][1];
    float P10_prev = ekf.P[1][0];
    float P11_prev = ekf.P[1][1];

    ekf.P[0][0] = P00_prev + dt*(P10_prev + P01_prev) + dt*dt*P11_prev + ekf.Q_accel*dt;
    ekf.P[0][1] = P01_prev + dt*P11_prev;
    ekf.P[1][0] = P10_prev + dt*P11_prev;
    ekf.P[1][1] = P11_prev + ekf.Q_accel*dt;

}
// 4. Bước UPDATE: Gọi bất cứ khi nào ToF có dữ liệu mới (Tần số 50Hz)

void kalman_update_vl53(float z_measure_tof, float roll, float pitch, float baro){

    float z_corrected = z_measure_tof * cosf(roll) * cosf(pitch);

    // --- BƯỚC 2A: Tính Innovation (Độ lệch thực tế - dự đoán) ---
    float y = z_measure_tof - ekf.z_predict; // measure - predict

    // --- BƯỚC 2B: Tính Kalman Gain K (K = P*H' / (H*P*H' + R)) ---
    // H = [1, 0]
    float S = ekf.P[0][0] + ekf.R_tof;
    float Innovation_sq = (y * y)/S; // Tranh loi do cao dot ngot

    if(Innovation_sq > 0.9f && fabsf(z_corrected - baro) > 0.3f) return;

    float K0 = ekf.P[0][0] / S;
    float K1 = ekf.P[1][0] / S;

    // --- BƯỚC 2C: Cập nhật Trạng thái X mới ---
    ekf.z_predict += K0 * y;
    ekf.vz_predict += K1 * y;

    // --- BƯỚC 2D: Cập nhật Ma trận P mới (P = (I - K*H)*P) ---
    float P00_new = (1.0f - K0) * ekf.P[0][0];
    float P01_new = (1.0f - K0) * ekf.P[0][1];
    float P10_new = ekf.P[1][0] - K1 * ekf.P[0][0];
    float P11_new = ekf.P[1][1] - K1 * ekf.P[0][1];

    ekf.P[0][0] = P00_new;
    ekf.P[0][1] = P01_new;
    ekf.P[1][0] = P10_new;
    ekf.P[1][1] = P11_new;
}

void kalman_update_baro(float baro){
    float y = baro - ekf.z_predict;
    float S = ekf.P[0][0] + ekf.R_bmp;
    float K0 = ekf.P[0][0] / S;
    float K1 = ekf.P[1][0] / S;

    ekf.z_predict += K0 * y;
    ekf.vz_predict += K1 * y;

    ekf.P[0][0] = (1.0f - K0) * ekf.P[0][0];
    ekf.P[0][1] = (1.0f - K0) * ekf.P[0][1];
    ekf.P[1][0] = ekf.P[1][0] - K1 * ekf.P[0][0];
    ekf.P[1][1] = ekf.P[1][1] - K1 * ekf.P[0][1];
}
