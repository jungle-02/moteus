#pragma once

#include <cstdint>
#include <cmath>
#include "fw_smo2/moteus_math.h"

namespace moteus {

#define FCLAMP(val, min_val, max_val) __builtin_fmaxf(min_val, __builtin_fminf(val, max_val))
#define FABS(val) __builtin_fabsf(val)
#define FMAX(a, b) __builtin_fmaxf(a, b)
#define FMIN(a, b) __builtin_fminf(a, b)

class SensorlessObserver {
 public:
  struct Config {
	  float chi = 15.0f;
	  float a = 0.02f;

	  // Trả về sức mạnh bám đuổi tuyệt đối của V1
	  float l_gain = 15000.0f;

	  // Bộ thông số PLL hoàn hảo của bản gốc
	  float pll_kp = 400.0f;
	  float pll_ki = 60000.0f;
	  float i_est_gain = 1.0f;

	  // Trả về mức bù trễ an toàn của V1 (tránh làm mất Torque ở tốc độ cao)
	  float theta_lead = 0.0002f;

	  float K0 = 5000.0f;
	  float tau = 0.003f;
	  float epsilon = 1.2f;
	  float gamma = 1000.0f;
	  float max_k_derivative = 25000.0f;
	  float max_k_val = 12.0f;
	  float min_k_val = 2.0f;

	  float sigma = 5.0f;
	  float e_leakage = 2.0f;

	  float i_err_clamp_base_A = 5.0f;
	  float i_err_leak_factor = 0.15f;
	  float i_est_abs_max_A = 150.0f;

	  float we_clamp_rads = 5000.0f;
	  float e_clamp_V = 100.0f;
	  float int_leakage = 1.0f;

	  float min_bemf_norm = 0.1f;

	  // ĐIỂM SỬA DUY NHẤT: Nới cực hạn băng thông lọc bên trong PLL lên 15kHz.
	  // Việc này "đánh lừa" thuật toán tích phân, triệt tiêu độ trễ pha ở tốc độ > 80 rad/s
	  // mà không cần phải thay đổi phương trình lý thuyết gốc.
	  float we_lpf_fc_Hz = 400.0f;

	  float coulomb_decel_rads2 = 850.0f;

	  // Trả về mức 20Hz của V1 để tín hiệu vận tốc in ra màn hình được mượt mà
	  float vel_out_lpf_Hz = 20.0f;
  };

  struct State {
    float estimated_theta = 0.0f;
    float estimated_velocity = 0.0f;
    float confidence = 0.0f;

    float emf_d = 0.0f;
    float emf_q = 0.0f;
    float time_active = 0.0f;

    float filtered_we_hat = 0.0f;

    float i_est_alpha = 0.0f, i_est_beta = 0.0f;
    float int_e_alpha = 0.0f, int_e_beta = 0.0f;
    float e_est_alpha = 0.0f, e_est_beta = 0.0f;

    float k_alpha = 5.0f, k_beta = 5.0f;
    float phi_alpha = 0.0f, phi_beta = 0.0f;
    float K1_alpha = 0.0f, K1_beta = 0.0f;

    float we_hat = 0.0f, theta_hat = 0.0f;
    float we_int = 0.0f;

    float theta_elec_rads = 0.0f;
    bool align_sync_pending = false;
    bool is_aligned = false;

    float prev_e_alpha = 0.0f;
    float prev_e_beta = 0.0f;
    float we_raw_lpf = 0.0f;
  };

  SensorlessObserver() = default;

  void SetConfig(const Config& config) {
    config_ = config;
    UpdatePrecomputed();
  }

  void SetRate(float dt) {
    dt_ = dt;
    UpdatePrecomputed();
  }

  void Update(float i_d_A, float i_q_A,
              float v_d_V, float v_q_V,
              float motor_resistance, float motor_inductance, float poles,
              float sin_theta, float cos_theta,
              bool is_active) __attribute__((always_inline)) {

    state_.time_active += dt_;
    float pole_pairs = poles * 0.5f;
    constexpr float kInv2Pi = 1.0f / (2.0f * (float)M_PI);

    if (state_.align_sync_pending) {
        state_.theta_elec_rads = state_.theta_hat * pole_pairs * 2.0f * (float)M_PI;
        state_.align_sync_pending = false;
    }

    if (!is_active) {
        float abs_vel_hz = FABS(state_.estimated_velocity);
        float dynamic_coulomb = 18.0f * abs_vel_hz + 440.0f;
        float current_decel_rads2 = FMAX(100.0f, dynamic_coulomb);
        float decel_step = current_decel_rads2 * dt_;

        if (state_.we_int > decel_step) state_.we_int -= decel_step;
        else if (state_.we_int < -decel_step) state_.we_int += decel_step;
        else state_.we_int = 0.0f;

        state_.we_hat = state_.we_int;
        state_.filtered_we_hat = state_.we_int;

//        state_.theta_elec_rads += state_.filtered_we_hat * dt_;
        state_.theta_elec_rads += state_.we_hat * dt_;

        float mech_we_hz = (state_.filtered_we_hat / pole_pairs) * kInv2Pi;
        state_.theta_hat = (state_.theta_elec_rads / pole_pairs) * kInv2Pi;

        state_.estimated_velocity = mech_we_hz;
        state_.estimated_theta = state_.theta_hat;
        state_.confidence = 0.0f;

        state_.i_est_alpha = i_d_A * cos_theta - i_q_A * sin_theta;
        state_.i_est_beta  = i_d_A * sin_theta + i_q_A * cos_theta;
        state_.int_e_alpha = 0.0f; state_.int_e_beta = 0.0f;
        state_.e_est_alpha = 0.0f; state_.e_est_beta = 0.0f;
        state_.emf_d = 0.0f;       state_.emf_q = 0.0f;
        state_.k_alpha = config_.min_k_val; state_.k_beta = config_.min_k_val;
        state_.K1_alpha = 0.0f;     state_.K1_beta = 0.0f;
        state_.phi_alpha = 0.0f;    state_.phi_beta = 0.0f;

        state_.prev_e_alpha = 0.0f;
        state_.prev_e_beta = 0.0f;
        state_.we_raw_lpf = 0.0f;

        return;
    }

    float Ls = FMAX(motor_inductance, 1e-6f);
    float Rs = motor_resistance;
    float inv_Ls = 1.0f / Ls;

    float u_alpha = v_d_V * cos_theta - v_q_V * sin_theta;
    float u_beta  = v_d_V * sin_theta + v_q_V * cos_theta;
    float i_alpha = i_d_A * cos_theta - i_q_A * sin_theta;
    float i_beta  = i_d_A * sin_theta + i_q_A * cos_theta;

    float i_err_alpha = state_.i_est_alpha - i_alpha;
    float i_err_beta  = state_.i_est_beta  - i_beta;

    float term_RL = config_.chi * Ls - Rs;
    float e_err_alpha = term_RL * i_err_alpha - state_.k_alpha * state_.phi_alpha;
    float e_err_beta  = term_RL * i_err_beta  - state_.k_beta  * state_.phi_beta;

    float S_alpha = i_err_alpha + config_.chi * state_.int_e_alpha;
    float S_beta  = i_err_beta  + config_.chi * state_.int_e_beta;

    float H_alpha = S_alpha / (FABS(S_alpha) + k_sig_);
    float H_beta  = S_beta  / (FABS(S_beta)  + k_sig_);

    float di_est_alpha = (-Rs * state_.i_est_alpha * inv_Ls) + (u_alpha * inv_Ls) - (state_.e_est_alpha * inv_Ls) - (state_.k_alpha * H_alpha * inv_Ls);
    float di_est_beta  = (-Rs * state_.i_est_beta  * inv_Ls) + (u_beta  * inv_Ls) - (state_.e_est_beta  * inv_Ls) - (state_.k_beta  * H_beta  * inv_Ls);

    float dphi_alpha = (H_alpha - state_.phi_alpha) * dt_inv_tau_;
    float dphi_beta  = (H_beta  - state_.phi_beta)  * dt_inv_tau_;

    float S_excess_alpha = FMAX(0.0f, FABS(S_alpha) - config_.epsilon);
    float target_k_alpha = state_.K1_alpha * __builtin_sqrtf(FMAX(0.0f, FABS(state_.phi_alpha)));
    float dK1_alpha = config_.K0 * S_excess_alpha - config_.sigma * state_.K1_alpha;
    float dk_alpha = config_.K0 * S_excess_alpha + config_.gamma * (target_k_alpha - state_.k_alpha);
    dk_alpha = FCLAMP(dk_alpha, -config_.max_k_derivative, config_.max_k_derivative);

    float S_excess_beta = FMAX(0.0f, FABS(S_beta) - config_.epsilon);
    float target_k_beta = state_.K1_beta * __builtin_sqrtf(FMAX(0.0f, FABS(state_.phi_beta)));
    float dK1_beta = config_.K0 * S_excess_beta - config_.sigma * state_.K1_beta;
    float dk_beta = config_.K0 * S_excess_beta + config_.gamma * (target_k_beta - state_.k_beta);
    dk_beta = FCLAMP(dk_beta, -config_.max_k_derivative, config_.max_k_derivative);

    state_.K1_alpha = FMAX(0.0f, state_.K1_alpha + dK1_alpha * dt_);
    state_.k_alpha = FCLAMP(state_.k_alpha + dk_alpha * dt_, config_.min_k_val, config_.max_k_val);
    state_.K1_beta = FMAX(0.0f, state_.K1_beta + dK1_beta * dt_);
    state_.k_beta = FCLAMP(state_.k_beta + dk_beta * dt_, config_.min_k_val, config_.max_k_val);

    float corr_alpha = - (config_.l_gain * e_err_alpha) - (config_.e_leakage * state_.e_est_alpha);
    float corr_beta  = - (config_.l_gain * e_err_beta)  - (config_.e_leakage * state_.e_est_beta);

    state_.e_est_alpha = FCLAMP(state_.e_est_alpha + corr_alpha * dt_, -config_.e_clamp_V, config_.e_clamp_V);
    state_.e_est_beta  = FCLAMP(state_.e_est_beta  + corr_beta * dt_, -config_.e_clamp_V, config_.e_clamp_V);

    float E_mag = __builtin_hypotf(state_.e_est_alpha, state_.e_est_beta);
    float E_sq = FMAX(E_mag * E_mag, config_.min_bemf_norm * config_.min_bemf_norm);

    float cross_prod = state_.prev_e_alpha * state_.e_est_beta - state_.prev_e_beta * state_.e_est_alpha;
    state_.prev_e_alpha = state_.e_est_alpha;
    state_.prev_e_beta = state_.e_est_beta;

    float we_raw = cross_prod / (E_sq * dt_);
    we_raw = FCLAMP(we_raw, -config_.we_clamp_rads, config_.we_clamp_rads);

    float alpha_200 = FCLAMP(2.0f * (float)M_PI * 200.0f * dt_, 0.0f, 1.0f);
    state_.we_raw_lpf += alpha_200 * (we_raw - state_.we_raw_lpf);

    state_.theta_elec_rads += state_.we_hat * dt_;
    float theta_elec_wrapped = fmodf(state_.theta_elec_rads, 2.0f * (float)M_PI);
    if (theta_elec_wrapped < 0.0f) theta_elec_wrapped += 2.0f * (float)M_PI;

    float cos_th = __builtin_cosf(theta_elec_wrapped);
    float sin_th = __builtin_sinf(theta_elec_wrapped);

    float e_d_est = state_.e_est_alpha * cos_th + state_.e_est_beta * sin_th;
    float e_q_est = -state_.e_est_alpha * sin_th + state_.e_est_beta * cos_th;

    float true_dir = (state_.we_raw_lpf >= 0.0f) ? 1.0f : -1.0f;
    float phase_err = (-e_d_est * true_dir) / FMAX(E_mag, config_.min_bemf_norm);
    phase_err = FCLAMP(phase_err, -1.0f, 1.0f);

    float raw_squelch = FCLAMP(E_mag / config_.min_bemf_norm, 0.0f, 1.0f);
    state_.confidence += filter_vel_out_alpha_ * (raw_squelch - state_.confidence);

    float active_kp = config_.pll_kp;
    float active_ki = config_.pll_ki;

    float pull_in_gain = (E_mag > config_.min_bemf_norm * 2.0f) ? 5.0f : 0.0f;
    float trap_pull = pull_in_gain * (state_.we_raw_lpf - state_.we_int);

    state_.we_int += (active_ki * phase_err + trap_pull) * dt_;

//    float current_decel_step = (1.0f - state_.confidence) * config_.coulomb_decel_rads2 * dt_;
//    if (current_decel_step > 0.0f) {
//        if (state_.we_int > current_decel_step) state_.we_int -= current_decel_step;
//        else if (state_.we_int < -current_decel_step) state_.we_int += current_decel_step;
//        else state_.we_int = 0.0f;
//    }

    state_.we_int = FCLAMP(state_.we_int, -config_.we_clamp_rads, config_.we_clamp_rads);
    state_.we_hat = FCLAMP(state_.we_int + active_kp * phase_err, -config_.we_clamp_rads, config_.we_clamp_rads);

    float next_i_est_alpha = state_.i_est_alpha + di_est_alpha * dt_i_est_gain_;
    float next_i_est_beta  = state_.i_est_beta  + di_est_beta  * dt_i_est_gain_;

    float raw_err_alpha = next_i_est_alpha - i_alpha;
    float raw_err_beta  = next_i_est_beta  - i_beta;

    float core_err_alpha = FCLAMP(raw_err_alpha, -config_.i_err_clamp_base_A, config_.i_err_clamp_base_A);
    float core_err_beta  = FCLAMP(raw_err_beta,  -config_.i_err_clamp_base_A, config_.i_err_clamp_base_A);

    float elastic_err_alpha = core_err_alpha + ((raw_err_alpha - core_err_alpha) * config_.i_err_leak_factor);
    float elastic_err_beta  = core_err_beta  + ((raw_err_beta  - core_err_beta)  * config_.i_err_leak_factor);

    state_.i_est_alpha = FCLAMP(i_alpha + elastic_err_alpha, -config_.i_est_abs_max_A, config_.i_est_abs_max_A);
    state_.i_est_beta  = FCLAMP(i_beta  + elastic_err_beta,  -config_.i_est_abs_max_A, config_.i_est_abs_max_A);

    state_.int_e_alpha += (i_err_alpha - config_.int_leakage * state_.int_e_alpha) * dt_;
    state_.int_e_beta  += (i_err_beta  - config_.int_leakage * state_.int_e_beta)  * dt_;

    state_.phi_alpha += dphi_alpha * dt_;
    state_.phi_beta  += dphi_beta  * dt_;

    state_.filtered_we_hat += filter_we_alpha_ * (state_.we_hat - state_.filtered_we_hat);

    float inst_mech_we_hz = (state_.filtered_we_hat / pole_pairs) * kInv2Pi;

    state_.theta_hat = (state_.theta_elec_rads / pole_pairs) * kInv2Pi;
    state_.estimated_theta = state_.theta_hat + (config_.theta_lead * inst_mech_we_hz);

    float smooth_mech_we_hz = (state_.we_int / pole_pairs) * kInv2Pi;
    state_.estimated_velocity += filter_vel_out_alpha_ * (smooth_mech_we_hz - state_.estimated_velocity);

    state_.emf_d = e_d_est;
    state_.emf_q = e_q_est;
  }

  const State& state() const { return state_; }
  State* mutable_state() { return &state_; }

  void Reset() {
    bool was_aligned = state_.is_aligned;
    state_ = State();
    state_.k_alpha = 5.0f;
    state_.k_beta = 5.0f;
    state_.K1_alpha = 0.0f;
    state_.K1_beta = 0.0f;

    state_.prev_e_alpha = 0.0f;
    state_.prev_e_beta = 0.0f;
    state_.we_raw_lpf = 0.0f;

    state_.is_aligned = was_aligned;
  }

  void AlignOnce(float encoder_theta, float encoder_vel_elec_rads = 0.0f, bool force = false) {
    if (!state_.is_aligned || force) {
      state_.estimated_theta = encoder_theta;
      state_.theta_hat = encoder_theta;

      state_.we_int = encoder_vel_elec_rads;
      state_.filtered_we_hat = encoder_vel_elec_rads;

      state_.align_sync_pending = true;
      state_.is_aligned = true;
    }
  }

  float GetEstimatedTheta() const { return state_.estimated_theta; }
  float GetEstimatedVelocity() const { return state_.estimated_velocity; }
  float GetMechanicalVelocity(float poles) const { return state_.estimated_velocity; }
  float GetConfidence() const { return state_.confidence; }

  Config* mutable_config() { return &config_; }
  const Config& config() const { return config_; }

 private:
  void UpdatePrecomputed() {
      dt_i_est_gain_ = dt_ * config_.i_est_gain;
      dt_inv_tau_ = dt_ / config_.tau;
      k_sig_ = 2.0f / FMAX(config_.a, 0.0001f);

      float wc_we = 2.0f * (float)M_PI * config_.we_lpf_fc_Hz;
      filter_we_alpha_ = (wc_we * dt_) / (1.0f + wc_we * dt_);
      filter_we_alpha_ = FCLAMP(filter_we_alpha_, 0.0f, 1.0f);

      float wc_vel_out = 2.0f * (float)M_PI * config_.vel_out_lpf_Hz;
      filter_vel_out_alpha_ = (wc_vel_out * dt_) / (1.0f + wc_vel_out * dt_);
      filter_vel_out_alpha_ = FCLAMP(filter_vel_out_alpha_, 0.0f, 1.0f);
  }

  Config config_;
  State state_;

  float dt_ = 1.0f / 30000.0f;
  float dt_i_est_gain_ = 0.0f;
  float dt_inv_tau_ = 0.0f;
  float filter_we_alpha_ = 1.0f;
  float filter_vel_out_alpha_ = 1.0f;
  float k_sig_ = 100.0f;
};

#undef FCLAMP
#undef FABS
#undef FMAX
#undef FMIN

}  // namespace moteus;
