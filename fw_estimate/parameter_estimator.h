#pragma once

#include <cmath>
#include <cstdint>

namespace moteus {

// bo uoc luong tham so dong co bang RLS
class ParameterEstimatorRLS {
 public:
  ParameterEstimatorRLS() {
    Reset(0.05f, 2e-5f, 0.002f, 25.0f);
  }

  // khoi tao tham so va gioi han
  void Reset(float init_R, float init_L, float init_flux, float init_temp_C) {
    init_R_ = init_R;
    init_L_ = init_L;
    init_flux_ = init_flux;

    kRMin_  = init_R_ * 0.7f;
    kRMax_  = init_R_ * 3.0f;
    kLMin_  = init_L_ * 0.1f;
    kLMax_  = init_L_ * 1.5f;
    kFxMin_ = init_flux_ * 0.5f;
    kFxMax_ = init_flux_ * 1.5f;

    params_.R      = init_R_;
    params_.Ld     = init_L_;
    params_.Lq     = init_L_;
    params_.flux_m = init_flux_;

    // filter output L de hien thi
    ld_output_ = init_L_;
    lq_output_ = init_L_;

    temp_lpf_ = init_temp_C;
    cached_dt_ = cached_inv_dt_ = did_dt_ = diq_dt_ = 0.0f;

    // filter dao ham dong
    did_smooth_ = diq_smooth_ = 0.0f;
    did_rms_est_ = diq_rms_est_ = 10.0f;

    // muc tieu P adaptive
    P_target_ld_ = P_target_lq_ = 1e-8f;
    high_speed_ctr_ld_ = high_speed_ctr_lq_ = 0;

    // bien loc tin hieu
    id_f_ = iq_f_ = ud_f_ = uq_f_ = we_f_ = 0.0f;
    prev_id_f_decim_ = prev_iq_f_decim_ = 0.0f;

    // tham so RLS cho L
    theta_ld_ = init_L_; P_ld_ = 1e-8f; ld_stable_ctr_ = 0;
    theta_lq_ = init_L_; P_lq_ = 1e-8f; lq_stable_ctr_ = 0;

    // tham so RLS cho R
    theta_R_       = init_R_; P_R_   = kPR_init;
    r_phi_filt_    = 0.0f;    r_y_filt_ = 0.0f;
    r_anchor_R_    = init_R_; r_anchor_T_ = init_temp_C;
    prev_iq_snap_  = prev_id_snap_ = prev_we_snap_ = 0.0f;
    r_filt_seeded_ = false;   r_absent_ctr_ = 0;

    // tham so RLS cho flux
    theta_flux_       = init_flux_; P_flux_ = kPFlux_init;
    flux_phi_filt_    = 0.0f;       flux_y_filt_ = 0.0f;
    flux_anchor_flux_ = init_flux_; flux_anchor_T_ = init_temp_C;
    flux_filt_seeded_ = false;      flux_absent_ctr_ = 0;

    // counter dieu khien chu ky cap nhat
    warmup_ctr_ = deriv_ctr_ = 0;
    ldq_ctr_ = ldq_turn_ = r_ctr_ = flux_ctr_ = 0;

    prev_id_ = prev_iq_ = 0.0f;
  }

  // ham cap nhat chinh
  void Update(float id, float iq, float ud, float uq, float we, float dt, float temp_C) {
    if (dt <= 0.0f) return;

    // loc nhiet do
    temp_lpf_ += kAlphaTemp * (temp_C - temp_lpf_);

    // bu deadtime va dien ap roi
    const float v_drop_q = SmoothDrop(iq, 0.055f, 0.010f);
    const float v_drop_d = SmoothDrop(id, 0.055f, 0.010f);
    const float uq_comp = uq - v_drop_q;
    const float ud_comp = ud - v_drop_d;

    // loc dong pha U I we
    id_f_ += kAlphaL * (id - id_f_);
    iq_f_ += kAlphaL * (iq - iq_f_);
    ud_f_ += kAlphaL * (ud_comp - ud_f_);
    uq_f_ += kAlphaL * (uq_comp - uq_f_);
    we_f_ += kAlphaL * (we - we_f_);

    // warmup ban dau
    if (warmup_ctr_ < kWarmupCycles) {
      ++warmup_ctr_;
      RunDerivEMA(id, iq, dt);
      r_anchor_T_    = temp_lpf_;
      flux_anchor_T_ = temp_lpf_;
      prev_id_ = id; prev_iq_ = iq;
      prev_we_snap_ = we;
      prev_id_f_decim_ = id_f_; prev_iq_f_decim_ = iq_f_;
      return;
    }

    // tinh dao ham dong
    RunDerivEMA(id, iq, dt);
    if (deriv_ctr_ < kDerivWarmup + kLSpikeLockout) ++deriv_ctr_;

    // kenh uoc luong Ld Lq
    if (++ldq_ctr_ >= kLdqDecim) {
      ldq_ctr_ = 0;
      const bool unlocked = (deriv_ctr_ >= kDerivWarmup + kLSpikeLockout);
      const float imag2_f = id_f_ * id_f_ + iq_f_ * iq_f_;

      const float dt_decim = dt * kLdqDecim;
      const float did_f = (id_f_ - prev_id_f_decim_) / dt_decim;
      const float diq_f = (iq_f_ - prev_iq_f_decim_) / dt_decim;
      prev_id_f_decim_ = id_f_;
      prev_iq_f_decim_ = iq_f_;

      // loc dao ham
      did_smooth_ += kDiLowPassAlpha * (did_f - did_smooth_);
      diq_smooth_ += kDiLowPassAlpha * (diq_f - diq_smooth_);

      // uoc luong rms de scale
      const float abs_did = AbsF(did_smooth_);
      const float abs_diq = AbsF(diq_smooth_);
      did_rms_est_ += 0.01f * (abs_did - did_rms_est_);
      diq_rms_est_ += 0.01f * (abs_diq - diq_rms_est_);

      if (unlocked && imag2_f > kImagMinSqL) {
        if (ldq_turn_ == 0) {
          // --- Ld CHANNEL ---
          UpdateLdAdaptive(abs_did, id_f_, iq_f_, ud_f_, uq_f_, we_f_,
                          high_speed_ctr_ld_, P_target_ld_, theta_ld_, P_ld_, did_smooth_);
        } else {
          // --- Lq CHANNEL ---
          UpdateLqAdaptive(abs_diq, id_f_, iq_f_, ud_f_, uq_f_, we_f_,
                          high_speed_ctr_lq_, P_target_lq_, theta_lq_, P_lq_, diq_smooth_);
        }
        ldq_turn_ ^= 1;
      }
    }

    // kenh uoc luong dien tro R
    if (++r_ctr_ >= kRDecim) {
      r_ctr_ = 0;
      const float dI = AbsF(iq - prev_iq_snap_), dId = AbsF(id - prev_id_snap_), dWe = AbsF(we - prev_we_snap_);
      prev_iq_snap_ = iq; prev_id_snap_ = id;

      const bool steady = (dI < kRSteadyDI) && (dId < kRSteadyDI) && (dWe < kRSteadyDWe);
      float phi_raw = 0.0f, y_raw = 0.0f;
      bool can_obs = false;

      if (AbsF(iq) > AbsF(id)) {
		  if (steady && AbsF(iq) > kIqMinForQAxis) {
			phi_raw = iq;
			y_raw   = uq_comp - we * params_.Ld * id - we * params_.flux_m;
			can_obs = true;
		  }
		} else {
		  if (steady && AbsF(id) > kIdMinForDAxis) {
			phi_raw = id;
			y_raw   = ud_comp + we * params_.Lq * iq;
			can_obs = true;
		  }
		}

      bool r_fired = false;
      if (can_obs) {
        if (!r_filt_seeded_) {
          r_phi_filt_ = phi_raw; r_y_filt_ = y_raw; r_filt_seeded_ = true;
          P_R_ = kPR_init;
        } else {
          r_phi_filt_ += kRFiltAlpha * (phi_raw - r_phi_filt_);
          r_y_filt_   += kRFiltAlpha * (y_raw   - r_y_filt_);
        }

        const float abs_phi = AbsF(r_phi_filt_);
        if (abs_phi > kRActMin && abs_phi < kRActMax) {
          RunScalarRLS(r_phi_filt_, r_y_filt_, theta_R_, P_R_, kLambdaR, 0.00005f);
          params_.R = ClampF(theta_R_, kRMin_, kRMax_);
          theta_R_ = params_.R; r_anchor_R_ = params_.R; r_anchor_T_ = temp_lpf_;
          r_fired = true; r_absent_ctr_ = 0;
        }
      } else {
        if (++r_absent_ctr_ > kRAbsentMax) { r_filt_seeded_ = false; r_absent_ctr_ = 0; }
      }

      if (!r_fired) {
        const float dT = temp_lpf_ - r_anchor_T_;
        params_.R = ClampF(r_anchor_R_ * (1.0f + kAlphaCu * dT), kRMin_, kRMax_);
        theta_R_ = params_.R;
      }
    }

    // kenh uoc luong flux
    if (++flux_ctr_ >= kFluxDecim) {
      flux_ctr_ = 0;
      const float dWe = AbsF(we - prev_we_snap_);
      prev_we_snap_ = we;

      if ((AbsF(did_dt_) < kSteadyDiMax) && (AbsF(diq_dt_) < kSteadyDiMax) && (dWe < kRSteadyDWe) &&
          AbsF(we) > kMinOmegaForFlux && (id * id + iq * iq) > kImagMinSqF) {

        const float phi_raw = we;
        const float y_raw = uq_comp - params_.R * iq - params_.Lq * diq_dt_ - we * params_.Ld * id;

        if (!flux_filt_seeded_) {
          flux_phi_filt_ = phi_raw; flux_y_filt_ = y_raw; flux_filt_seeded_ = true;
          P_flux_ = kPFlux_init;
        } else {
          flux_phi_filt_ += kFluxFiltAlpha * (phi_raw - flux_phi_filt_);
          flux_y_filt_   += kFluxFiltAlpha * (y_raw   - flux_y_filt_);
        }
        RunScalarRLS(flux_phi_filt_, flux_y_filt_, theta_flux_, P_flux_, kLambdaFlux, 0.000005f);

        params_.flux_m = ClampF(theta_flux_, kFxMin_, kFxMax_);
        theta_flux_ = params_.flux_m;
        flux_anchor_flux_ = params_.flux_m;
        flux_anchor_T_ = temp_lpf_;
      } else {
        const float dT = temp_lpf_ - flux_anchor_T_;
        params_.flux_m = ClampF(flux_anchor_flux_ * (1.0f + kAlphaFlux * dT), kFxMin_, kFxMax_);
        theta_flux_ = params_.flux_m;
      }
    }
    prev_id_ = id; prev_iq_ = iq;

  }

  // ham lay ket qua
  float GetR()   const { return params_.R; }
  float GetFai() const { return params_.flux_m; }
  float GetLd()  const { return ld_output_; }
  float GetLq()  const { return lq_output_; }

 private:

  // cac hang so cau hinh
  static constexpr float kAlphaTemp      = 0.0001f;
  static constexpr int   kWarmupCycles   = 2000;
  static constexpr int   kDerivWarmup    = 400;
  static constexpr float kMaxDiDt        = 5000.0f;

  // tham so adaptive L
  static constexpr float kDiLowPassAlpha = 0.05f;
  static constexpr float kDiHighSpeed    = 300.0f;
  static constexpr float kDiMediumSpeed  = 50.0f;
  static constexpr int   kHighSpeedDuration = 500;


  static constexpr float kPHighSpeed  = 1e-3f;
  static constexpr float kPMediumSpeed = 1e-5f;
  static constexpr float kPSlowSpeed  = 1e-8f;
  static constexpr float kPMinFloor   = 1e-9f;

  static constexpr float kDiRunMinSmooth = 15.0f;
  static constexpr float kLOutAlpha      = 0.001f;
  static constexpr float kLGUIAlpha      = 0.0005f;

  // tham so kenh L
  static constexpr int   kLdqDecim       = 10;
  static constexpr float kAlphaL         = 0.1f;
  static constexpr float kPhiMinSnrL     = 20.0f;
  static constexpr float kImagMinSqL     = 0.01f;
  const float kLambdaL                   = 0.98f;

  // tham so kenh R
  static constexpr int   kRDecim         = 50;
  static constexpr float kRFiltAlpha     = 0.08f;
  static constexpr int   kRAbsentMax     = 5;
  static constexpr float kRActMin        = 1.0f;
  static constexpr float kRActMax        = 25.0f;
  static constexpr float kRSteadyDI      = 0.05f;
  static constexpr float kRSteadyDWe     = 0.5f;
  static constexpr float kIqMinForQAxis  = 1.0f;
  static constexpr float kIdMinForDAxis  = 0.5f;
  static constexpr float kAlphaCu        = 0.00393f;
  static constexpr float kPR_init        = 1e-6f;

  // tham so kenh flux
  static constexpr int   kFluxDecim      = 500;
  static constexpr float kFluxFiltAlpha  = 0.05f;
  static constexpr int   kFluxAbsentMax  = 3;
  static constexpr float kSteadyDiMax    = 10.0f;
  static constexpr float kMinOmegaForFlux= 20.0f;
  static constexpr float kAlphaFlux      = -0.001f;
  static constexpr float kPFlux_init     = 1e-7f;
  static constexpr float kImagMinSqF     = 0.09f;

  const float kLambdaR    = 0.99999f;
  const float kLambdaFlux = 0.9999f;

  // bien trang thai
  float init_R_, init_L_, init_flux_;
  float kLMin_, kLMax_, kRMin_, kRMax_, kFxMin_, kFxMax_;

  struct EstimatedParams { float R, Ld, Lq, flux_m; } params_;

  float temp_lpf_, cached_dt_, cached_inv_dt_;
  float id_f_, iq_f_, ud_f_, uq_f_, we_f_;
  float prev_id_f_decim_, prev_iq_f_decim_;

  float did_dt_, diq_dt_;
  float did_smooth_, diq_smooth_;
  float did_rms_est_, diq_rms_est_;
  float P_target_ld_, P_target_lq_;
  int high_speed_ctr_ld_, high_speed_ctr_lq_;
  float ld_output_, lq_output_;
  float prev_id_, prev_iq_;

  float theta_ld_, P_ld_; int ld_stable_ctr_;
  float theta_lq_, P_lq_; int lq_stable_ctr_;
  float theta_R_, P_R_, r_phi_filt_, r_y_filt_, r_anchor_R_, r_anchor_T_;
  float prev_iq_snap_, prev_id_snap_, prev_we_snap_;
  bool r_filt_seeded_; int r_absent_ctr_;
  float theta_flux_, P_flux_, flux_phi_filt_, flux_y_filt_, flux_anchor_flux_, flux_anchor_T_;
  bool flux_filt_seeded_; int flux_absent_ctr_;
  int warmup_ctr_, deriv_ctr_, ldq_ctr_, ldq_turn_, r_ctr_, flux_ctr_;

  static constexpr int kLSpikeLockout = 2000;

  // ham adaptive cho Ld
  void UpdateLdAdaptive(float abs_did, float id_f, float iq_f, float ud_f,
                        float uq_f, float we_f, int& speed_ctr, float& P_target,
                        float& theta, float& P, float did_smooth) {

    if (abs_did > kDiHighSpeed) {
      P_target = kPHighSpeed;
      speed_ctr = kHighSpeedDuration;
    } else if (abs_did > kDiMediumSpeed) {
      P_target = kPMediumSpeed;
    } else {
      if (speed_ctr > 0) --speed_ctr;
      P_target = (speed_ctr > 0) ? kPMediumSpeed : kPSlowSpeed;
    }


    P = P + 0.01f * (P_target - P);
    P = ClampF(P, kPMinFloor, 1e-1f);


    if (AbsF(did_smooth) > kDiRunMinSmooth) {
      const float phi = did_smooth;
      const float y = ud_f - params_.R * id_f + we_f * params_.Lq * iq_f;


      const float clip = 0.2f * (AbsF(params_.R * id_f) + AbsF(we_f * params_.Lq * iq_f) + 0.1f);

      RunScalarRLS(phi, y, theta, P, kLambdaL, clip);
      theta = ClampF(theta, kLMin_, kLMax_);
      params_.Ld += kLOutAlpha * (theta - params_.Ld);
    }

    ld_output_ += kLGUIAlpha * (params_.Ld - ld_output_);
  }

  // ham adaptive cho Lq
  void UpdateLqAdaptive(float abs_diq, float id_f, float iq_f, float ud_f,
                        float uq_f, float we_f, int& speed_ctr, float& P_target,
                        float& theta, float& P, float diq_smooth) {

    if (abs_diq > kDiHighSpeed) {
      P_target = kPHighSpeed;
      speed_ctr = kHighSpeedDuration;
    } else if (abs_diq > kDiMediumSpeed) {
      P_target = kPMediumSpeed;
    } else {
      if (speed_ctr > 0) --speed_ctr;
      P_target = (speed_ctr > 0) ? kPMediumSpeed : kPSlowSpeed;
    }


    P = P + 0.01f * (P_target - P);
    P = ClampF(P, kPMinFloor, 1e-1f);


    if (AbsF(diq_smooth) > kDiRunMinSmooth) {
      const float phi = diq_smooth;
      const float y = uq_f - params_.R * iq_f - we_f * (params_.Ld * id_f + params_.flux_m);


      const float clip = 0.2f * (AbsF(params_.R * iq_f) + AbsF(we_f * (params_.Ld * id_f + params_.flux_m)) + 0.1f);

      RunScalarRLS(phi, y, theta, P, kLambdaL, clip);
      theta = ClampF(theta, kLMin_, kLMax_);
      params_.Lq += kLOutAlpha * (theta - params_.Lq);
    }

    lq_output_ += kLGUIAlpha * (params_.Lq - lq_output_);
  }

  // bu deadtime
  static inline float SmoothDrop(float current, float deadtime_v, float resistive_v_per_a) {
    const float threshold = 0.05f;
    float v_dt = 0.0f;
    if (current > threshold) v_dt = deadtime_v;
    else if (current < -threshold) v_dt = -deadtime_v;
    else v_dt = (current / threshold) * deadtime_v;
    return v_dt + current * resistive_v_per_a;
  }

  // tinh dao ham bang EMA
  void RunDerivEMA(float id, float iq, float dt) {
    if (dt != cached_dt_) { cached_dt_ = dt; cached_inv_dt_ = 1.0f / dt; }
    const float rdid = ClampF((id - prev_id_) * cached_inv_dt_, -kMaxDiDt, kMaxDiDt);
    const float rdiq = ClampF((iq - prev_iq_) * cached_inv_dt_, -kMaxDiDt, kMaxDiDt);
    did_dt_ += 0.01f * (rdid - did_dt_);
    diq_dt_ += 0.01f * (rdiq - diq_dt_);
  }

  // thuat toan RLS scalar
  static void RunScalarRLS(float phi, float y, float& theta, float& P, float lambda, float clip) {
    if (phi < 1e-9f && phi > -1e-9f) return;
    const float phiP = phi * P;
    const float denom = lambda + phi * phiP;
    if (denom < 1e-12f) return;
    const float K = phiP / denom;
    float err = y - theta * phi;
    if (err > clip) err = clip; else if (err < -clip) err = -clip;
    theta += K * err;
    P = (P - K * phiP) / lambda;
    if (P > 1e4f) P = 1e4f; else if (P < 1e-12f) P = 1e-12f;
  }

  static inline float AbsF(float x) { return x < 0.0f ? -x : x; }
  static inline float ClampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};

} // namespace moteus
