#pragma once

#include <cmath>
#include <cstdint>

namespace moteus {

// BO UOC LUONG THAM SO DONG CO BANG RLS
class ParameterEstimatorRLS {
 public:
  ParameterEstimatorRLS() {
    // KHOI TAO GIA TRI BAN DAU CHO R, L, FLUX VA NHIET DO
    Reset(0.0549f, 26.05e-6f, 0.0026f, 25.0f);
  }

  void Reset(float init_R, float init_L, float init_flux, float init_temp_C) {
    // LUU GIA TRI BAN DAU DE LAM MOC THAM CHIEU VA BU NHIET
    init_R_ = init_R; init_L_ = init_L; init_flux_ = init_flux; init_temp_C_ = init_temp_C;

    // THIET LAP GIOI HAN VAT LY CHO CAC THAM SO TRANH RLS BI VO
    kRMin_  = init_R_ * 0.7f; kRMax_  = init_R_ * 3.0f;
    kLMin_  = init_L_ * 0.1f; kLMax_  = init_L_ * 1.5f;
    kFxMin_ = init_flux_ * 0.5f; kFxMax_ = init_flux_ * 1.5f;

    // GAN GIA TRI BAN DAU CHO CAC THAM SO UOC LUONG
    params_.R = init_R_; params_.Ld = init_L_; params_.Lq = init_L_; params_.flux_m = init_flux_;
    ld_output_ = init_L_; lq_output_ = init_L_;

    // BO LOC NHIET DO
    temp_lpf_ = init_temp_C;
    cached_dt_ = 0.0f;

    // KHOI TAO STATE VARIABLE FILTER (SVF)
    id_svf_ = iq_svf_ = ud_svf_ = uq_svf_ = we_svf_ = 0.0f;
    did_dt_svf_ = diq_dt_svf_ = 0.0f;

    // KHOI TAO RLS CHO L VOI P RAT NHO DE TRANH NO
    theta_ld_ = init_L_; P_ld_ = 1e-10f;
    theta_lq_ = init_L_; P_lq_ = 1e-10f;

    // KHOI TAO RLS CHO R
    theta_R_       = init_R_; P_R_   = kPR_init;
    r_phi_filt_    = 0.0f;    r_y_filt_ = 0.0f;
    r_anchor_R_    = init_R_; r_anchor_T_ = init_temp_C;
    prev_iq_snap_  = prev_id_snap_ = prev_we_snap_ = 0.0f;
    r_filt_seeded_ = false;   r_absent_ctr_ = 0;

    // KHOI TAO RLS CHO FLUX
    theta_flux_       = init_flux_; P_flux_ = kPFlux_init;
    flux_phi_filt_    = 0.0f;       flux_y_filt_ = 0.0f;
    flux_anchor_flux_ = init_flux_; flux_anchor_T_ = init_temp_C;
    flux_filt_seeded_ = false;      flux_absent_ctr_ = 0;

    // CAC BO DEM DIEU KHIEN CAC GIAI DOAN
    warmup_ctr_ = deriv_ctr_ = ldq_turn_ = r_ctr_ = flux_ctr_ = 0;
  }

  void Update(float id, float iq, float ud, float uq, float we, float dt, float temp_C) {
    if (dt <= 0.0f) return;

    // LOC NHIET DO DE GIAM NHIEU
    temp_lpf_ += kAlphaTemp * (temp_C - temp_lpf_);
    if (dt != cached_dt_) cached_dt_ = dt;
    const float inv_dt = 1.0f / dt;

    // BU SUT AP DO DEADTIME VA DIEN TRO NOI
    const float v_drop_q = SmoothDrop(iq, 0.055f, 0.010f);
    const float v_drop_d = SmoothDrop(id, 0.055f, 0.010f);
    const float uq_comp = uq - v_drop_q;
    const float ud_comp = ud - v_drop_d;

    // ================================================================
    // STATE VARIABLE FILTER (SVF)
    // LOC VA DONG BO TIN HIEU DE LAY DAO HAM ON DINH
    // ================================================================
    did_dt_svf_ = (id - id_svf_) * (kAlphaSVF * inv_dt);
    diq_dt_svf_ = (iq - iq_svf_) * (kAlphaSVF * inv_dt);

    id_svf_ += kAlphaSVF * (id - id_svf_);
    iq_svf_ += kAlphaSVF * (iq - iq_svf_);
    ud_svf_ += kAlphaSVF * (ud_comp - ud_svf_);
    uq_svf_ += kAlphaSVF * (uq_comp - uq_svf_);
    we_svf_ += kAlphaSVF * (we - we_svf_);

    // GIAI DOAN WARMUP: KHONG CHAY RLS, CHI BU THEO NHIET DO
    if (warmup_ctr_ < kWarmupCycles) {
      ++warmup_ctr_;
      params_.R = init_R_ * (1.0f + kAlphaCu * (temp_lpf_ - init_temp_C_));
      params_.flux_m = init_flux_ * (1.0f + kAlphaFlux * (temp_lpf_ - init_temp_C_));
      r_anchor_T_ = temp_lpf_; flux_anchor_T_ = temp_lpf_;
      r_anchor_R_ = params_.R; flux_anchor_flux_ = params_.flux_m;
      theta_R_ = params_.R; theta_flux_ = params_.flux_m;
      return;
    }

    // MO KHOA DAO HAM SAU MOT THOI GIAN DE TRANH NHIEU BAN DAU
    if (deriv_ctr_ < kDerivWarmup + kLSpikeLockout) ++deriv_ctr_;
    const bool unlocked = (deriv_ctr_ >= kDerivWarmup + kLSpikeLockout);

    // ================================================================
    // CAP NHAT Ld, Lq (LUAN PHIEN DE GIAM TAI TINH TOAN)
    // ================================================================
    if (unlocked) {
      if (ldq_turn_ == 0) {
        UpdateLRobust(id_svf_, iq_svf_, ud_svf_, we_svf_,
                      theta_ld_, P_ld_, did_dt_svf_, params_.Ld, true);
      } else {
        UpdateLRobust(id_svf_, iq_svf_, uq_svf_, we_svf_,
                      theta_lq_, P_lq_, diq_dt_svf_, params_.Lq, false);
      }
      ldq_turn_ ^= 1;
    }

    // ================================================================
    // CAP NHAT DIEN TRO R (CHAY CHAM, CO LOC VA DIEU KIEN STEADY)
    // ================================================================
    if (++r_ctr_ >= kRDecim) {
      r_ctr_ = 0;

      // KIEM TRA DIEU KIEN ON DINH
      const float dI = AbsF(iq - prev_iq_snap_), dId = AbsF(id - prev_id_snap_), dWe = AbsF(we - prev_we_snap_);
      prev_iq_snap_ = iq; prev_id_snap_ = id;
      const bool steady = (dI < kRSteadyDI) && (dId < kRSteadyDI) && (dWe < kRSteadyDWe);

      float phi_raw = 0.0f, y_raw = 0.0f; bool can_obs = false;

      // LUA CHON TRUC D HOAC Q DE UOC LUONG
      if (AbsF(iq_svf_) > AbsF(id_svf_)) {
        if (steady && AbsF(we_svf_) > kMinOmegaForR) {
            phi_raw = iq_svf_;
            y_raw = uq_svf_ - we_svf_ * params_.Ld * id_svf_ - we_svf_ * params_.flux_m;
            can_obs = true;
        }
      } else {
        if (steady && AbsF(id_svf_) > kIdMinForDAxis) {
            phi_raw = id_svf_;
            y_raw = ud_svf_ + we_svf_ * params_.Lq * iq_svf_;
            can_obs = true;
        }
      }

      // CHAY RLS NEU DU DIEU KIEN
      if (can_obs) {
        if (!r_filt_seeded_) {
          r_phi_filt_ = phi_raw; r_y_filt_ = y_raw; r_filt_seeded_ = true; P_R_ = kPR_init;
        } else {
          r_phi_filt_ += kRFiltAlpha * (phi_raw - r_phi_filt_);
          r_y_filt_ += kRFiltAlpha * (y_raw - r_y_filt_);
        }

        if (AbsF(r_phi_filt_) > kRActMin && AbsF(r_phi_filt_) < kRActMax) {
          RunScalarRLS(r_phi_filt_, r_y_filt_, theta_R_, P_R_, kLambdaR, 0.00005f);
          theta_R_ = ClampF(theta_R_, kRMin_, kRMax_);
          r_anchor_R_ = theta_R_; r_anchor_T_ = temp_lpf_; r_absent_ctr_ = 0;
        }
      } else {
          if (++r_absent_ctr_ > kRAbsentMax) { r_filt_seeded_ = false; r_absent_ctr_ = 0; }
      }
    }

    // ================================================================
    // CAP NHAT FLUX (YEU CAU DIEU KIEN ON DINH VA TOC DO DU LON)
    // ================================================================
    if (++flux_ctr_ >= kFluxDecim) {
      flux_ctr_ = 0;

      const float dWe = AbsF(we - prev_we_snap_); prev_we_snap_ = we;

      if ((AbsF(did_dt_svf_) < kSteadyDiMax) && (AbsF(diq_dt_svf_) < kSteadyDiMax) && (dWe < kRSteadyDWe) &&
          AbsF(we_svf_) > kMinOmegaForFlux && (id_svf_ * id_svf_ + iq_svf_ * iq_svf_) > kImagMinSqF) {

        const float y_raw = uq_svf_ - params_.R * iq_svf_ - we_svf_ * params_.Ld * id_svf_;

        if (!flux_filt_seeded_) {
          flux_phi_filt_ = we_svf_; flux_y_filt_ = y_raw; flux_filt_seeded_ = true; P_flux_ = kPFlux_init;
        } else {
          flux_phi_filt_ += kFluxFiltAlpha * (we_svf_ - flux_phi_filt_);
          flux_y_filt_ += kFluxFiltAlpha * (y_raw - flux_y_filt_);
        }

        RunScalarRLS(flux_phi_filt_, flux_y_filt_, theta_flux_, P_flux_, kLambdaFlux, 0.000005f);
        theta_flux_ = ClampF(theta_flux_, kFxMin_, kFxMax_);
        flux_anchor_flux_ = theta_flux_; flux_anchor_T_ = temp_lpf_;
      }
    }

    // ================================================================
    // SLEW RATE LIMITER
    // GIOI HAN TOC DO THAY DOI DE TRANH NHAY THAM SO
    // ================================================================
    const float max_r_step = 0.005f * dt;
    float target_r = r_anchor_R_ * (1.0f + kAlphaCu * (temp_lpf_ - r_anchor_T_));
    target_r = ClampF(target_r, kRMin_, kRMax_);

    if (target_r - params_.R > max_r_step) params_.R += max_r_step;
    else if (target_r - params_.R < -max_r_step) params_.R -= max_r_step;
    else params_.R = target_r;

    const float max_flux_step = 0.00005f * dt;
    float target_flux = flux_anchor_flux_ * (1.0f + kAlphaFlux * (temp_lpf_ - flux_anchor_T_));
    target_flux = ClampF(target_flux, kFxMin_, kFxMax_);

    if (target_flux - params_.flux_m > max_flux_step) params_.flux_m += max_flux_step;
    else if (target_flux - params_.flux_m < -max_flux_step) params_.flux_m -= max_flux_step;
    else params_.flux_m = target_flux;
  }

  // CAP NHAT KHI KHONG CO DONG (STANDBY)
  // CHI BU NHIET VA GIU THAM SO ON DINH
  void UpdateStandby(float dt, float temp_C) {
    if (dt <= 0.0f) return;
    temp_lpf_ += kAlphaTemp * (temp_C - temp_lpf_);

    const float max_r_step = 0.005f * dt;
    float target_r = ClampF(r_anchor_R_ * (1.0f + kAlphaCu * (temp_lpf_ - r_anchor_T_)), kRMin_, kRMax_);
    if (target_r - params_.R > max_r_step) params_.R += max_r_step;
    else if (target_r - params_.R < -max_r_step) params_.R -= max_r_step;
    else params_.R = target_r;

    const float max_flux_step = 0.00005f * dt;
    float target_flux = ClampF(flux_anchor_flux_ * (1.0f + kAlphaFlux * (temp_lpf_ - flux_anchor_T_)), kFxMin_, kFxMax_);
    if (target_flux - params_.flux_m > max_flux_step) params_.flux_m += max_flux_step;
    else if (target_flux - params_.flux_m < -max_flux_step) params_.flux_m -= max_flux_step;
    else params_.flux_m = target_flux;
  }

  // CAC HAM LAY GIA TRI UOC LUONG
  float GetR()   const { return params_.R; }
  float GetFai() const { return params_.flux_m; }
  float GetLd()  const { return ld_output_; }
  float GetLq()  const { return lq_output_; }

 private:
  static constexpr float kAlphaTemp = 0.0001f;
  static constexpr int   kWarmupCycles = 2000;
  static constexpr int   kDerivWarmup = 400;
  static constexpr int   kLSpikeLockout = 2000;

  static constexpr float kAlphaSVF = 0.02f;
  static constexpr float kMacroDiDtThreshold = 500.0f;
  static constexpr float kWeDecoupleThreshold = 15.0f;

  static constexpr float kLOutAlpha   = 0.1f;
  static constexpr float kLGUIAlpha   = 0.05f;
  static constexpr float kLDesatAlpha = 0.001f;
  const float kLambdaL                = 0.99f;

  static constexpr int   kRDecim = 50; static constexpr float kRFiltAlpha = 0.08f; static constexpr int kRAbsentMax = 5;
  static constexpr float kRActMin = 1.0f; static constexpr float kRActMax = 25.0f; static constexpr float kRSteadyDI = 0.05f;
  static constexpr float kRSteadyDWe = 0.5f; static constexpr float kMinOmegaForR = 5.0f; static constexpr float kIdMinForDAxis = 0.5f;
  static constexpr float kAlphaCu = 0.00393f; static constexpr float kPR_init = 1e-6f;

  static constexpr int   kFluxDecim = 500; static constexpr float kFluxFiltAlpha = 0.05f; static constexpr int kFluxAbsentMax = 3;
  static constexpr float kSteadyDiMax = 50.0f; static constexpr float kMinOmegaForFlux = 20.0f; static constexpr float kAlphaFlux = -0.001f;
  static constexpr float kPFlux_init = 1e-7f; static constexpr float kImagMinSqF = 0.09f;

  const float kLambdaR = 0.99999f; const float kLambdaFlux = 0.9999f;

  float init_R_, init_L_, init_flux_, init_temp_C_;
  float kLMin_, kLMax_, kRMin_, kRMax_, kFxMin_, kFxMax_;
  struct EstimatedParams { float R, Ld, Lq, flux_m; } params_;

  float temp_lpf_, cached_dt_;

  float id_svf_, iq_svf_, ud_svf_, uq_svf_, we_svf_;
  float did_dt_svf_, diq_dt_svf_;

  float ld_output_, lq_output_;

  float theta_ld_, P_ld_, theta_lq_, P_lq_;
  float theta_R_, P_R_, r_phi_filt_, r_y_filt_, r_anchor_R_, r_anchor_T_;
  float prev_iq_snap_, prev_id_snap_, prev_we_snap_;
  bool r_filt_seeded_; int r_absent_ctr_;
  float theta_flux_, P_flux_, flux_phi_filt_, flux_y_filt_, flux_anchor_flux_, flux_anchor_T_;
  bool flux_filt_seeded_; int flux_absent_ctr_;
  int warmup_ctr_, deriv_ctr_, ldq_turn_, r_ctr_, flux_ctr_;

  // HAM CAP NHAT L VOI CO CHE ROBUST + SATURATION
  void UpdateLRobust(float id_sync, float iq_sync, float u_sync, float we_sync,
                     float& theta, float& P, float di_dt_sync,
                     float& param_L, bool is_d_axis) {

    float abs_di = AbsF(di_dt_sync);
    float i_mag_sq = id_sync * id_sync + iq_sync * iq_sync;

    // CHAY RLS KHI CO BIEN THIEN MANH (TRANSIENT)
    if (abs_di > kMacroDiDtThreshold) {
      const float phi = di_dt_sync;
      float y;

      if (is_d_axis) {
        y = u_sync - params_.R * id_sync + we_sync * params_.Lq * iq_sync;
      } else {
        y = u_sync - params_.R * iq_sync - we_sync * (params_.Ld * id_sync + params_.flux_m);
      }

      float we_penalty = 1.0f;
      float abs_we = AbsF(we_sync);
      if (abs_we > kWeDecoupleThreshold) {
          we_penalty = kWeDecoupleThreshold / abs_we;
      }

      float clip = 0.5f * (AbsF(u_sync) + 1.0f);

      if (phi > 1e-9f || phi < -1e-9f) {
          const float phiP = phi * P;
          const float denom = kLambdaL + phi * phiP;

          if (denom > 1e-12f) {
              const float K = phiP / denom;
              float err = y - theta * phi;

              if (err > clip) err = clip; else if (err < -clip) err = -clip;

              theta += K * err * we_penalty;
              theta = ClampF(theta, kLMin_, kLMax_);

              P = (P - K * phiP) / kLambdaL;

              // KHOA P TRONG VUNG AN TOAN TRANH NO SO HOC
              if (P < 1e-12f) P = 1e-12f;
              else if (P > 1e-9f) P = 1e-9f;
          }
      }

      param_L += kLOutAlpha * (theta - param_L);

    } else {
      // KHONG CO TRANSIENT -> GIU NGUYEN VA HOI PHUC THEO SATURATION
      theta = param_L;

      float saturation_factor = 1.0f - 0.02f * i_mag_sq;
      if (saturation_factor < 0.6f) saturation_factor = 0.6f;

      float target_L = init_L_ * saturation_factor;

      param_L += kLDesatAlpha * (target_L - param_L);
    }

    if (is_d_axis) ld_output_ += kLGUIAlpha * (param_L - ld_output_);
    else lq_output_ += kLGUIAlpha * (param_L - lq_output_);
  }

  // HAM BU SUT AP DO DEADTIME + DIEN TRO
  static inline float SmoothDrop(float current, float deadtime_v, float resistive_v_per_a) {
    const float threshold = 0.05f; float v_dt = 0.0f;
    if (current > threshold) v_dt = deadtime_v;
    else if (current < -threshold) v_dt = -deadtime_v;
    else v_dt = (current / threshold) * deadtime_v;
    return v_dt + current * resistive_v_per_a;
  }

  // RLS SCALAR CHUNG CHO R VA FLUX
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
  }

  static inline float AbsF(float x) { return x < 0.0f ? -x : x; }
  static inline float ClampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};

} // namespace moteus
