// Copyright 2024 moteus contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// Model Predictive Control + Kalman Filter state estimator for BLDC motors.
//
// Transliterated from the validated MATLAB reference implementation.
//
// Algorithm overview
// ------------------
//  1. Kalman Filter (3-state augmented model)
//       x = [omega_m (rad/s), theta_m (rad), Tm (Nm)]^T
//       Predicts one step ahead using motor dynamics, then corrects via
//       speed/angle measurements to estimate the unmeasured load torque Tm.
//
//  2. Reference Trajectory Generator
//       Generates (omega_ref, theta_ref) at each future prediction step.
//       Four modes: S-curve, Sinusoidal, Trapezoidal, Step.
//
//  3. MPC Controller (augmented incremental formulation)
//       Augmented state: xa = [omega, theta, u]^T  where u = Te_eff - Tm.
//       Computes the optimal torque increment DeltaU by minimising
//         J = ||Y - Rs||^2_Qy  +  ||DeltaU||^2_Ru
//       subject to:   -Te_max <= Te_k <= Te_max  for each prediction step k.
//
//  4. Hildreth QP Solver
//       Solves the inequality-constrained QP arising from the MPC cost using
//       the iterative Lagrange-multiplier (Hildreth) method.
//
// Usage
// -----
//   // Instantiate once (outside the ISR):
//   moteus::MPCKalmanEstimator estimator;
//   estimator.SetTrajectoryMode(moteus::MPCKalmanEstimator::TrajectoryMode::kSCurve);
//
//   // In the 30 kHz ISR:
//   estimator.Update(omega_meas, theta_e_meas, clk_now, clk_prev);
//   float iq_ref = estimator.output().iq_ref;
//   float id_ref = estimator.output().id_ref;
//
// Memory footprint (all fixed-size, zero dynamic allocation)
// ----------------------------------------------------------
//   Precomputed matrices (class members):
//     F      : 2*Np x Nx  = 50 x 3  =  150 float (600 B)
//     G      : 2*Np x Nc  = 50 x 8  =  400 float (1600 B)
//     H^-1   : Nc   x Nc  =  8 x 8  =   64 float (256 B)
//     HinvMT : Nc   x 2Nc =  8 x 16 =  128 float (512 B)
//     W      : 2Nc  x 2Nc = 16 x 16 =  256 float (1024 B)
//     Mineq  : 2Nc  x Nc  = 16 x 8  =  128 float (512 B)
//   Total precomputed: ~4.5 KB (reused every ISR call).
//
//   ISR stack (Update() local variables): ~1 KB.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "fw/ccm.h"

namespace moteus {

/// Combined MPC + Kalman Filter estimator / controller for a BLDC motor.
class MPCKalmanEstimator {
 public:
  // ─── Compile-time dimensions ────────────────────────────────────────────
  static constexpr int kNp   = 25;        ///< Prediction horizon (steps)
  static constexpr int kNc   = 8;         ///< Control horizon (steps)
  static constexpr int kNy   = 2;         ///< Outputs: [omega, theta]
  static constexpr int kNx   = 3;         ///< States:  [omega, theta, Tm]
  static constexpr int kNcon = 2 * kNc;   ///< Number of inequality constraints

  static constexpr float kPi  = 3.141592653589793f;
  static constexpr float k2Pi = 6.283185307179586f;

  // ─── Configuration structures ───────────────────────────────────────────

  /// Physical motor parameters.
  struct MotorConfig {
    int   poles   = 14;         ///< Number of motor poles
    float J       = 2.94e-5f;  ///< Rotor inertia [kg·m²]
    float B       = 1e-4f;     ///< Viscous friction coefficient [Nms/rad]
    float pm      = 0.0185f;   ///< Permanent-magnet flux linkage [Wb]
    float Iq_max  = 12.0f;     ///< Maximum q-axis current [A]
  };

  /// MPC tuning parameters.
  struct MPCConfig {
    float Qw       = 300.0f;   ///< Speed-error tracking weight
    float Qth      = 0.5f;     ///< Angle-error tracking weight
    float Ru       = 1.8f;     ///< Control-increment (ΔU) penalty weight
    float Ts       = 30e-6f;   ///< Sampling period [s]
    int   max_iter = 200;      ///< Maximum Hildreth QP iterations
    float tol      = 1e-9f;    ///< QP convergence tolerance (||Δλ||² < tol)
  };

  /// Kalman Filter noise configuration.
  struct KFConfig {
    float q_w   = 1.0f;    ///< Process noise – angular velocity
    float q_th  = 1e-6f;   ///< Process noise – angle
    float q_tm  = 0.5f;    ///< Process noise – load torque (random-walk)
    float r_w   = 5e-4f;   ///< Measurement noise – angular velocity
    float r_th  = 1e-4f;   ///< Measurement noise – angle
  };

  /// Reference trajectory mode.
  enum class TrajectoryMode : uint8_t {
    kSCurve      = 1,  ///< Jerk-limited S-curve to theta_max
    kSinusoidal  = 2,  ///< Constant-amplitude sinusoidal speed
    kTrapezoidal = 3,  ///< Piecewise-linear (trapezoidal) velocity profile
    kStep        = 4,  ///< Square-wave speed reversal
  };

  /// Reference trajectory tuning parameters.
  struct TrajectoryConfig {
    TrajectoryMode mode          = TrajectoryMode::kSCurve;
    float max_rpm                = 700.0f;        ///< Maximum speed [rpm]
    float alpha_max              = 200.0f;        ///< Max acceleration [rad/s²]
    float omega_max_limit        = 100.0f;        ///< Speed saturation cap [rad/s]
    float theta_max              = 5.0f * k2Pi;  ///< S-curve target angle [rad]
    float omega_sin              = k2Pi * 2.0f;  ///< Sinusoidal speed [rad/s]
    float wAref_limit            = 50.0f;         ///< Sinusoidal amplitude cap [rad/s]
    float t2_trap                = 0.5f;          ///< Trapezoidal constant-v duration [s]
    float t_step                 = 1.0f;          ///< Step transition period [s]
  };

  // ─── State and output ────────────────────────────────────────────────────

  /// Persistent state carried across Update() calls.
  struct State {
    float omega_hat = 0.0f;           ///< KF-estimated mechanical speed [rad/s]
    float theta_hat = 0.0f;           ///< KF-estimated mechanical angle [rad]
    float Tm_hat    = 0.0f;           ///< KF-estimated load torque [Nm]
    float P[kNx * kNx] = {};          ///< KF error covariance (3×3, row-major)
    float lambda[kNcon] = {};         ///< Hildreth dual variables (warm-start)
    float Te_prev   = 0.0f;           ///< Torque command applied last step [Nm]
  };

  /// Outputs produced by Update().
  struct Output {
    float iq_ref        = 0.0f;   ///< Q-axis current reference [A]
    float id_ref        = 0.0f;   ///< D-axis current reference [A] (= 0)
    float Te_cmd        = 0.0f;   ///< Torque command [Nm]
    float w_ref_rpm     = 0.0f;   ///< Current-step speed reference [rpm]
    float theta_ref_rad = 0.0f;   ///< Current-step angle reference [rad]
  };

  // ─── Constructor ─────────────────────────────────────────────────────────

  /// Construct with default or customised parameters.
  /// Precomputes all constant MPC matrices (F, G, H⁻¹, W, etc.) so that
  /// Update() can run in a time-bounded ISR budget.
  MPCKalmanEstimator()
      : MPCKalmanEstimator(MotorConfig{}, MPCConfig{}, KFConfig{},
                            TrajectoryConfig{}) {}

  MPCKalmanEstimator(const MotorConfig&      motor,
                     const MPCConfig&        mpc,
                     const KFConfig&         kf,
                     const TrajectoryConfig& traj)
      : motor_(motor), mpc_(mpc), kf_(kf), traj_(traj) {
    Reset();
    PrecomputeMPCMatrices();
  }

  // ─── Configuration API ───────────────────────────────────────────────────

  /// Reset all runtime state to safe initial values (keeps configuration).
  void Reset() {
    state_  = {};
    output_ = {};
    // Initialise KF covariance to identity.
    for (int i = 0; i < kNx; i++) {
      state_.P[i * kNx + i] = 1.0f;
    }
  }

  /// Change the reference trajectory mode at runtime.
  void SetTrajectoryMode(TrajectoryMode mode) {
    traj_.mode = mode;
  }

  // ─── Accessors ───────────────────────────────────────────────────────────

  const Output& output() const { return output_; }
  const State&  state()  const { return state_;  }

  /// Convenience accessor: latest torque command [Nm].
  float GetTorqueCommand() const { return output_.Te_cmd; }

  /// Retrieve the three KF state estimates.
  /// Any pointer may be nullptr if that value is not needed.
  void GetEstimatedState(float* omega, float* theta, float* Tm) const {
    if (omega) *omega = state_.omega_hat;
    if (theta) *theta = state_.theta_hat;
    if (Tm)    *Tm    = state_.Tm_hat;
  }

  // ─── Main ISR update ─────────────────────────────────────────────────────

  /// Run one full MPC+KF step.  Must be called once per sampling period.
  ///
  /// @param omega_m_meas  Measured mechanical angular velocity [rad/s]
  /// @param theta_e_meas  Measured electrical angle [rad]
  /// @param clk_sec       Current timestamp [s]
  /// @param clk_prev      Timestamp of the previous call [s]
  void Update(float omega_m_meas,
              float theta_e_meas,
              float clk_sec,
              float clk_prev) MOTEUS_CCM_ATTRIBUTE {
    output_ = {};

    const float Ts   = mpc_.Ts;
    const float J    = motor_.J;
    const float Bf   = motor_.B;
    const float p_f  = static_cast<float>(motor_.poles) / 2.0f;
    const float pm   = motor_.pm;
    const float Iq_max = motor_.Iq_max;
    const float Te_max = 1.5f * p_f * pm * Iq_max;

    // new_tick: true if this is a genuine new sample (guards the QP solve).
    const bool new_tick = (clk_sec > clk_prev) || (clk_sec == 0.0f);

    // =================================================================
    // 1. Kalman Filter
    // =================================================================

    // Convert electrical angle to mechanical angle.
    const float th_m_val = theta_e_meas / p_f;
    const float w_m_val  = omega_m_meas;
    const float Te_p     = state_.Te_prev;   // previous torque command [Nm]
    const float Tm_p     = state_.Tm_hat;    // previous load torque estimate [Nm]

    // KF system matrices.
    // A_kf = [1-Ts*B/J,  0,    -Ts/J;
    //          Ts,        1,     0;
    //          0,         0,     1   ]
    float A_kf[kNx * kNx];
    A_kf[0] = 1.0f - Ts * Bf / J;  A_kf[1] = 0.0f;  A_kf[2] = -Ts / J;
    A_kf[3] = Ts;                    A_kf[4] = 1.0f;  A_kf[5] = 0.0f;
    A_kf[6] = 0.0f;                  A_kf[7] = 0.0f;  A_kf[8] = 1.0f;

    // B_kf = [Ts/J, 0, 0]^T  (only the first row is nonzero)
    const float B_kf0 = Ts / J;

    // Previous KF state initialised from current measurements (per MATLAB reference).
    float xkf_prev[kNx] = { w_m_val, th_m_val, Tm_p };
    float Pkf_prev[kNx * kNx];
    for (int i = 0; i < kNx * kNx; i++) { Pkf_prev[i] = state_.P[i]; }

    // ── Prediction ──────────────────────────────────────────────────
    // x_pred = A_kf * xkf_prev + B_kf * Te_p
    float x_pred[kNx];
    Mul33Vec3(A_kf, xkf_prev, x_pred);
    x_pred[0] += B_kf0 * Te_p;  // B_kf[1] = B_kf[2] = 0

    // P_pred = A_kf * Pkf_prev * A_kf' + Q_kf  (Q_kf = diag)
    float tmp33[kNx * kNx];
    float A_kf_T[kNx * kNx];
    float P_pred[kNx * kNx];
    Mul33(A_kf, Pkf_prev, tmp33);
    Transpose3(A_kf, A_kf_T);
    Mul33(tmp33, A_kf_T, P_pred);
    P_pred[0] += kf_.q_w;    // Q_kf diagonal elements
    P_pred[4] += kf_.q_th;
    P_pred[8] += kf_.q_tm;

    // ── Update ──────────────────────────────────────────────────────
    // C_kf = [I₂ | 0] → C_kf * M extracts the first 2 rows of M.

    // S_kf = C_kf * P_pred * C_kf' + R_kf
    //      = top-left 2×2 block of P_pred + diag(r_w, r_th)
    float S_kf[kNy * kNy];
    S_kf[0] = P_pred[0] + kf_.r_w;
    S_kf[1] = P_pred[1];
    S_kf[2] = P_pred[3];
    S_kf[3] = P_pred[4] + kf_.r_th;

    float S_inv[kNy * kNy];
    Inv22(S_kf, S_inv);

    // K_kf = (P_pred * C_kf') * S_inv
    // P_pred * C_kf' = first 2 columns of P_pred  (3×2)
    float K_kf[kNx * kNy];
    for (int i = 0; i < kNx; i++) {
      for (int j = 0; j < kNy; j++) {
        float s = 0.0f;
        for (int k = 0; k < kNy; k++) {
          s += P_pred[i * kNx + k] * S_inv[k * kNy + j];
        }
        K_kf[i * kNy + j] = s;
      }
    }

    // Innovation: y_meas - C_kf * x_pred  (C_kf picks rows 0 and 1)
    const float innov0 = w_m_val  - x_pred[0];
    const float innov1 = th_m_val - x_pred[1];

    // x_hat = x_pred + K_kf * innov
    float x_hat[kNx];
    for (int i = 0; i < kNx; i++) {
      x_hat[i] = x_pred[i]
               + K_kf[i * kNy + 0] * innov0
               + K_kf[i * kNy + 1] * innov1;
    }

    // P_hat = (I - K_kf * C_kf) * P_pred
    // K_kf * C_kf (3×3): since C_kf = [I₂|0], columns 2 are zero.
    float KC[kNx * kNx] = {};
    for (int i = 0; i < kNx; i++) {
      for (int j = 0; j < kNy; j++) {
        KC[i * kNx + j] = K_kf[i * kNy + j];
      }
    }
    float I_KC[kNx * kNx];
    for (int i = 0; i < kNx * kNx; i++) {
      I_KC[i] = ((i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f) - KC[i];
    }
    float P_hat[kNx * kNx];
    Mul33(I_KC, P_pred, P_hat);

    // Store updated KF state.
    state_.omega_hat = x_hat[0];
    state_.theta_hat = x_hat[1];
    state_.Tm_hat    = x_hat[2];
    for (int i = 0; i < kNx * kNx; i++) { state_.P[i] = P_hat[i]; }

    // =================================================================
    // 2. Reference Trajectory Generation
    // =================================================================

    const float OMEGA_MAX = traj_.max_rpm * (k2Pi / 60.0f);
    const float omega_max = (traj_.omega_max_limit < OMEGA_MAX)
                            ? traj_.omega_max_limit : OMEGA_MAX;
    const float wAref     = (traj_.wAref_limit < OMEGA_MAX)
                            ? traj_.wAref_limit : OMEGA_MAX;
    const float t1_trap   = omega_max / traj_.alpha_max;

    // Current-step reference (for telemetry).
    {
      float w_now = 0.0f, th_now = 0.0f;
      RefAtTime(clk_sec, omega_max, wAref, t1_trap, &w_now, &th_now);
      w_now = Clamp(w_now, -OMEGA_MAX, OMEGA_MAX);
      output_.w_ref_rpm     = w_now * 60.0f / k2Pi;
      output_.theta_ref_rad = th_now;
    }

    // Future reference vector Rs (2*Np × 1).
    float Rs[kNy * kNp];
    for (int i = 0; i < kNp; i++) {
      const float ti = clk_sec + static_cast<float>(i + 1) * Ts;
      float wref_i = 0.0f, thref_i = 0.0f;
      RefAtTime(ti, omega_max, wAref, t1_trap, &wref_i, &thref_i);
      wref_i = Clamp(wref_i, -OMEGA_MAX, OMEGA_MAX);
      Rs[2 * i]     = wref_i;
      Rs[2 * i + 1] = thref_i;
    }

    // =================================================================
    // 3. MPC Controller
    // =================================================================

    // Clamp previous torque for constraint initialisation.
    const float Te_p_clip = Clamp(Te_p, -Te_max, Te_max);

    // Augmented initial state: xa₀ = [omega_hat, theta_hat, u_prev]^T
    // where u_prev = Te_p - Tm_hat  (net effective torque excl. load).
    const float u_prev = Te_p_clip - x_hat[2];
    float x0[kNx] = { x_hat[0], x_hat[1], u_prev };

    // ── Prediction error vector  e = F * x0 - Rs  (2*Np × 1) ───────
    float e_vec[kNy * kNp];
    for (int i = 0; i < kNp; i++) {
      float Fx0_0 = 0.0f, Fx0_1 = 0.0f;
      for (int k = 0; k < kNx; k++) {
        Fx0_0 += F_[(2 * i)     * kNx + k] * x0[k];
        Fx0_1 += F_[(2 * i + 1) * kNx + k] * x0[k];
      }
      e_vec[2 * i]     = Fx0_0 - Rs[2 * i];
      e_vec[2 * i + 1] = Fx0_1 - Rs[2 * i + 1];
    }

    // ── Gradient vector  f = G' * Qy * e  (Nc × 1) ─────────────────
    float f_vec[kNc];
    for (int j = 0; j < kNc; j++) {
      float s = 0.0f;
      for (int i = 0; i < kNp; i++) {
        s += mpc_.Qw  * G_[(2 * i)     * kNc + j] * e_vec[2 * i];
        s += mpc_.Qth * G_[(2 * i + 1) * kNc + j] * e_vec[2 * i + 1];
      }
      f_vec[j] = s;
    }

    // ── Inequality RHS  Nineq (2*Nc × 1) ────────────────────────────
    // Nineq = [Te_max - Te_p_clip  (×Nc); Te_max + Te_p_clip  (×Nc)]
    float Nineq[kNcon];
    for (int i = 0; i < kNc; i++) {
      Nineq[i]        =  Te_max - Te_p_clip;
      Nineq[i + kNc]  =  Te_max + Te_p_clip;
    }

    // ── Hildreth QP Solver ───────────────────────────────────────────
    float Te_next_val = Te_p_clip;

    if (new_tick) {
      // H_reg_inv * f  (Nc)
      float Hinv_f[kNc];
      MulNcNcVec(H_reg_inv_, f_vec, Hinv_f);

      // Mineq * Hinv_f  (2*Nc)
      float M_Hinv_f[kNcon];
      MulConNcVec(Mineq_, Hinv_f, M_Hinv_f);

      // kvec = Nineq + Mineq * Hinv_f
      float kvec[kNcon];
      for (int i = 0; i < kNcon; i++) {
        kvec[i] = Nineq[i] + M_Hinv_f[i];
      }

      // Warm-start from previous lambda.
      float lambda[kNcon];
      for (int i = 0; i < kNcon; i++) { lambda[i] = state_.lambda[i]; }

      // Hildreth iterations (Gauss-Seidel on the dual problem).
      for (int it = 0; it < mpc_.max_iter; it++) {
        float diff_sq = 0.0f;

        for (int ii = 0; ii < kNcon; ii++) {
          float sumi = 0.0f;
          for (int jj = 0; jj < kNcon; jj++) {
            if (jj != ii) {
              sumi += W_[ii * kNcon + jj] * lambda[jj];
            }
          }
          const float wi = (-kvec[ii] - sumi) / W_[ii * kNcon + ii];
          const float new_lam = (wi > 0.0f) ? wi : 0.0f;
          const float dlam = new_lam - lambda[ii];
          diff_sq += dlam * dlam;
          lambda[ii] = new_lam;
        }

        if (diff_sq < mpc_.tol) { break; }
      }

      // Save lambda for warm-starting next step.
      for (int i = 0; i < kNcon; i++) { state_.lambda[i] = lambda[i]; }

      // Optimal DeltaU = -(H_reg_inv * (f + Mineq' * lambda))
      //                = -(Hinv_f + HinvMT * lambda)
      float HinvMT_lam[kNc];
      MulNcConVec(HinvMT_, lambda, HinvMT_lam);

      const float delta_u1 = -(Hinv_f[0] + HinvMT_lam[0]);
      const float u_next   = u_prev + delta_u1;
      Te_next_val = u_next + x_hat[2];   // = u_next + Tm_hat
    }

    // Clamp torque and store for next step.
    Te_next_val      = Clamp(Te_next_val, -Te_max, Te_max);
    state_.Te_prev   = Te_next_val;
    output_.Te_cmd   = Te_next_val;

    // Convert torque to q-axis current reference.
    // Te = (3/2) * p * pm * iq   →   iq = 2 * Te / (3 * p * pm)
    const float iq_raw = (2.0f / (3.0f * p_f * pm)) * output_.Te_cmd;
    output_.iq_ref = Clamp(iq_raw, -Iq_max, Iq_max);
    output_.id_ref = 0.0f;
  }

 private:
  // ─── Precomputed MPC matrices (populated in constructor) ─────────────────

  /// Prediction matrix:   F  (2*Np × Nx = 50 × 3, row-major)
  float F_[kNy * kNp * kNx] = {};
  /// Impulse-response matrix: G  (2*Np × Nc = 50 × 8, row-major)
  float G_[kNy * kNp * kNc] = {};
  /// Inverse of (G'*Qy*G + Ru*I + ε*I): H_reg⁻¹  (Nc × Nc)
  float H_reg_inv_[kNc * kNc] = {};
  /// H_reg⁻¹ * Mineq'  (Nc × 2*Nc)  — used to form DeltaU without re-solve
  float HinvMT_[kNc * kNcon] = {};
  /// W = Mineq * H_reg⁻¹ * Mineq'  (2*Nc × 2*Nc) — Hildreth kernel
  float W_[kNcon * kNcon] = {};
  /// Mineq  (2*Nc × Nc)  — constraint matrix
  float Mineq_[kNcon * kNc] = {};

  // ─── Configuration ───────────────────────────────────────────────────────
  MotorConfig     motor_;
  MPCConfig       mpc_;
  KFConfig        kf_;
  TrajectoryConfig traj_;

  // ─── Runtime state / output ──────────────────────────────────────────────
  State  state_;
  Output output_;

  // =========================================================================
  // PrecomputeMPCMatrices()
  //
  // Builds F, G, H, Mineq, H_reg_inv, HinvMT, and W from the motor/MPC
  // configuration.  All of these are state-independent and therefore computed
  // once in the constructor.
  // =========================================================================
  void PrecomputeMPCMatrices() {
    const float Ts = mpc_.Ts;
    const float J  = motor_.J;
    const float Bf = motor_.B;

    // Augmented plant matrices (same as in Update, but motor-param-dependent).
    // A_aug = [1-Ts*B/J,  0,   Ts/J;
    //           Ts,        1,   0;
    //           0,         0,   1  ]
    float A_aug[kNx * kNx];
    A_aug[0] = 1.0f - Ts * Bf / J;  A_aug[1] = 0.0f;  A_aug[2] = Ts / J;
    A_aug[3] = Ts;                    A_aug[4] = 1.0f;  A_aug[5] = 0.0f;
    A_aug[6] = 0.0f;                  A_aug[7] = 0.0f;  A_aug[8] = 1.0f;

    // B_aug = [Ts/J, 0, 1]^T
    const float B_aug[kNx] = { Ts / J, 0.0f, 1.0f };

    // C_aug = [[1,0,0],[0,1,0]]  →  selects rows 0 and 1 of any 3×_ matrix.

    // ──────────────────────────────────────────────────────────────────
    // Build Markov-parameter vector h and prediction matrix F_.
    //
    //   h[:,i] = C_aug * A_aug^i * B_aug   (i = 0 … Np-1)
    //   F_[2*i : 2*(i+1), :] = C_aug * A_aug^(i+1)
    //
    // Matches the MATLAB loop:
    //   Apow_h = eye(3);  Apow_F = A_aug;
    //   for i = 1:Np
    //     h(:,i)          = C_aug * Apow_h * B_aug;
    //     Apow_h          = Apow_h * A_aug;
    //     F(2i-1:2i, :)   = C_aug * Apow_F;
    //     Apow_F          = Apow_F * A_aug;
    //   end
    // ──────────────────────────────────────────────────────────────────
    float h[kNy * kNp] = {};    // Markov parameters: h[2*i], h[2*i+1]

    float Apow_h[kNx * kNx];
    float Apow_F[kNx * kNx];
    Mat3Identity(Apow_h);   // starts as A_aug^0 = I
    Mat3Copy(A_aug, Apow_F); // starts as A_aug^1

    for (int i = 0; i < kNp; i++) {
      // h[:,i] = C_aug * Apow_h * B_aug  = first 2 rows of (Apow_h * B_aug)
      for (int ky = 0; ky < kNy; ky++) {
        float val = 0.0f;
        for (int kx = 0; kx < kNx; kx++) {
          val += Apow_h[ky * kNx + kx] * B_aug[kx];
        }
        h[2 * i + ky] = val;
      }
      // F_[2*i : 2*(i+1), :] = C_aug * Apow_F  = first 2 rows of Apow_F
      for (int ky = 0; ky < kNy; ky++) {
        for (int kx = 0; kx < kNx; kx++) {
          F_[(2 * i + ky) * kNx + kx] = Apow_F[ky * kNx + kx];
        }
      }
      // Advance power matrices.
      float tmp[kNx * kNx];
      Mul33(Apow_h, A_aug, tmp);
      Mat3Copy(tmp, Apow_h);
      Mul33(Apow_F, A_aug, tmp);
      Mat3Copy(tmp, Apow_F);
    }

    // ──────────────────────────────────────────────────────────────────
    // Build G_ (2*Np × Nc).
    //
    //   G_[2*i : 2*(i+1), j] = h[:, i-j]   for j = 0…Nc-1, i = j…Np-1
    // ──────────────────────────────────────────────────────────────────
    for (int k = 0; k < kNy * kNp * kNc; k++) { G_[k] = 0.0f; }
    for (int j = 0; j < kNc; j++) {
      for (int i = j; i < kNp; i++) {
        const int hcol = i - j;
        G_[(2 * i)     * kNc + j] = h[2 * hcol];
        G_[(2 * i + 1) * kNc + j] = h[2 * hcol + 1];
      }
    }

    // ──────────────────────────────────────────────────────────────────
    // Build H = G' * Qy * G + (Ru + ε) * I  (Nc × Nc, SPD).
    // ──────────────────────────────────────────────────────────────────
    float H_mat[kNc * kNc] = {};
    for (int j1 = 0; j1 < kNc; j1++) {
      for (int j2 = 0; j2 < kNc; j2++) {
        float s = 0.0f;
        for (int i = 0; i < kNp; i++) {
          s += mpc_.Qw  * G_[(2 * i)     * kNc + j1] * G_[(2 * i)     * kNc + j2];
          s += mpc_.Qth * G_[(2 * i + 1) * kNc + j1] * G_[(2 * i + 1) * kNc + j2];
        }
        if (j1 == j2) { s += mpc_.Ru + 1e-9f; }
        H_mat[j1 * kNc + j2] = s;
      }
    }

    // ──────────────────────────────────────────────────────────────────
    // Invert H (Cholesky): store in H_reg_inv_.
    // ──────────────────────────────────────────────────────────────────
    CholeskyInvertNc(H_mat, H_reg_inv_);

    // ──────────────────────────────────────────────────────────────────
    // Build Mineq_ (2*Nc × Nc) = [tril(1); -tril(1)].
    // ──────────────────────────────────────────────────────────────────
    for (int k = 0; k < kNcon * kNc; k++) { Mineq_[k] = 0.0f; }
    for (int i = 0; i < kNc; i++) {
      for (int j = 0; j <= i; j++) {
        Mineq_[i * kNc + j]         =  1.0f;   //  S
        Mineq_[(i + kNc) * kNc + j] = -1.0f;   // -S
      }
    }

    // ──────────────────────────────────────────────────────────────────
    // Build HinvMT_ = H_reg_inv * Mineq'  (Nc × 2*Nc).
    // Mineq' [kNc × kNcon]:  Mineq_T[k, j] = Mineq_[j * kNc + k]
    // ──────────────────────────────────────────────────────────────────
    for (int i = 0; i < kNc; i++) {
      for (int j = 0; j < kNcon; j++) {
        float s = 0.0f;
        for (int k = 0; k < kNc; k++) {
          s += H_reg_inv_[i * kNc + k] * Mineq_[j * kNc + k];
        }
        HinvMT_[i * kNcon + j] = s;
      }
    }

    // ──────────────────────────────────────────────────────────────────
    // Build W_ = Mineq * HinvMT_  (2*Nc × 2*Nc).
    // ──────────────────────────────────────────────────────────────────
    for (int i = 0; i < kNcon; i++) {
      for (int j = 0; j < kNcon; j++) {
        float s = 0.0f;
        for (int k = 0; k < kNc; k++) {
          s += Mineq_[i * kNc + k] * HinvMT_[k * kNcon + j];
        }
        W_[i * kNcon + j] = s;
      }
    }
  }

  // =========================================================================
  // RefAtTime() — reference trajectory generator
  //
  // Returns the reference speed [rad/s] and angle [rad] at time t.
  // Faithfully mirrors the MATLAB ref_at_time() helper.
  // =========================================================================
  void RefAtTime(float t,
                 float omega_max,
                 float wAref,
                 float t1_trap,
                 float* wref,
                 float* thref) const MOTEUS_CCM_ATTRIBUTE {
    const float alpha_max = traj_.alpha_max;
    const float theta_max = traj_.theta_max;
    const float omega_sin = traj_.omega_sin;
    const float t2_trap   = traj_.t2_trap;
    const float t_step    = traj_.t_step;

    switch (traj_.mode) {

      // ── S-curve ─────────────────────────────────────────────────────
      case TrajectoryMode::kSCurve: {
        const float tc   = omega_max / alpha_max;
        const float jmax = (tc > 0.0f) ? (alpha_max / tc) : 0.0f;
        const float denom = (omega_max > 0.0f) ? omega_max : 1.0f;
        const float tm   = (theta_max - 2.0f * jmax * tc * tc * tc) / denom;
        const float tm_c = (tm > 0.0f) ? tm : 0.0f;
        const float tf   = 4.0f * tc + tm_c;

        if (t < tc) {
          *wref  = 0.5f * jmax * t * t;
          *thref = (1.0f / 6.0f) * jmax * t * t * t;
        } else if (t < 2.0f * tc) {
          const float tau = t - tc;
          *wref  = 0.5f * jmax * tc * tc
                 + jmax * tc * tau
                 - 0.5f * jmax * tau * tau;
          *thref = (1.0f / 6.0f) * jmax * tc * tc * tc
                 + 0.5f * jmax * tc * tc * tau
                 + 0.5f * jmax * tc  * tau * tau
                 - (1.0f / 6.0f) * jmax * tau * tau * tau;
        } else if (t < 2.0f * tc + tm_c) {
          *wref  = omega_max;
          *thref = jmax * tc * tc * tc + omega_max * (t - 2.0f * tc);
        } else if (t < 3.0f * tc + tm_c) {
          const float tau = t - 2.0f * tc - tm_c;
          *wref  = omega_max - 0.5f * jmax * tau * tau;
          *thref = jmax * tc * tc * tc
                 + omega_max * tm_c
                 + omega_max * tau
                 - (1.0f / 6.0f) * jmax * tau * tau * tau;
        } else if (t < tf) {
          const float tau = tf - t;
          *wref  = 0.5f * jmax * tau * tau;
          *thref = theta_max - (1.0f / 6.0f) * jmax * tau * tau * tau;
        } else {
          *wref  = 0.0f;
          *thref = theta_max;
        }
        break;
      }

      // ── Sinusoidal ──────────────────────────────────────────────────
      case TrajectoryMode::kSinusoidal: {
        *wref  = wAref * std::cos(omega_sin * t);
        *thref = (omega_sin > 0.0f)
               ? (wAref / omega_sin) * std::sin(omega_sin * t)
               : 0.0f;
        break;
      }

      // ── Trapezoidal ─────────────────────────────────────────────────
      case TrajectoryMode::kTrapezoidal: {
        if (t < t1_trap) {
          *wref  = alpha_max * t;
          *thref = 0.5f * alpha_max * t * t;
        } else if (t < t1_trap + t2_trap) {
          *wref  = omega_max;
          *thref = 0.5f * alpha_max * t1_trap * t1_trap
                 + omega_max * (t - t1_trap);
        } else if (t < 2.0f * t1_trap + t2_trap) {
          const float tau = t - (t1_trap + t2_trap);
          *wref  = omega_max - alpha_max * tau;
          *thref = 0.5f * alpha_max * t1_trap * t1_trap
                 + omega_max * t2_trap
                 + omega_max * tau
                 - 0.5f * alpha_max * tau * tau;
        } else {
          *wref  = 0.0f;
          *thref = 0.0f;
        }
        break;
      }

      // ── Step ────────────────────────────────────────────────────────
      case TrajectoryMode::kStep: {
        if (t < t_step) {
          *wref  = omega_max;
          *thref = omega_max * t;
        } else if (t < 2.0f * t_step) {
          *wref  = -omega_max;
          // Angle at end of first phase: omega_max * t_step
          // Then decreasing at omega_max rad/s for (t - t_step) seconds.
          *thref = omega_max * t_step - omega_max * (t - t_step);
        } else {
          *wref  = 0.0f;
          *thref = 0.0f;
        }
        break;
      }

      default:
        *wref  = 0.0f;
        *thref = 0.0f;
        break;
    }
  }

  // =========================================================================
  // Low-level fixed-size matrix utilities
  // =========================================================================

  /// Saturate x to [lo, hi].
  static float Clamp(float x, float lo, float hi) MOTEUS_CCM_ATTRIBUTE {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  // ── 3×3 operations (row-major, stride 3) ─────────────────────────────────

  /// C = A * B  (3×3 × 3×3)
  static void Mul33(const float A[9], const float B[9], float C[9]) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        float s = 0.0f;
        for (int k = 0; k < 3; k++) { s += A[i * 3 + k] * B[k * 3 + j]; }
        C[i * 3 + j] = s;
      }
    }
  }

  /// y = A * x  (3×3 · 3)
  static void Mul33Vec3(const float A[9], const float x[3], float y[3]) {
    for (int i = 0; i < 3; i++) {
      y[i] = A[i*3] * x[0] + A[i*3+1] * x[1] + A[i*3+2] * x[2];
    }
  }

  /// B = Aᵀ  (3×3 transpose)
  static void Transpose3(const float A[9], float B[9]) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        B[j * 3 + i] = A[i * 3 + j];
      }
    }
  }

  /// A = I₃
  static void Mat3Identity(float A[9]) {
    for (int i = 0; i < 9; i++) { A[i] = 0.0f; }
    A[0] = A[4] = A[8] = 1.0f;
  }

  /// dst = src  (copy 9 floats)
  static void Mat3Copy(const float src[9], float dst[9]) {
    for (int i = 0; i < 9; i++) { dst[i] = src[i]; }
  }

  // ── 2×2 operations ───────────────────────────────────────────────────────

  /// Ainv = A⁻¹  for a 2×2 matrix.
  /// If the matrix is singular (|det| < eps), Ainv is zeroed to avoid
  /// propagating invalid values; the Kalman gain will then be zero and
  /// the filter coasts on the prediction.
  static void Inv22(const float A[4], float Ainv[4]) {
    const float det = A[0] * A[3] - A[1] * A[2];
    constexpr float kEps = 1e-30f;
    if (det > kEps || det < -kEps) {
      const float inv_det = 1.0f / det;
      Ainv[0] =  A[3] * inv_det;
      Ainv[1] = -A[1] * inv_det;
      Ainv[2] = -A[2] * inv_det;
      Ainv[3] =  A[0] * inv_det;
    } else {
      Ainv[0] = Ainv[1] = Ainv[2] = Ainv[3] = 0.0f;
    }
  }

  // ── Nc×Nc Cholesky inversion ──────────────────────────────────────────────

  /// Compute Ainv = A⁻¹ for a symmetric positive-definite kNc×kNc matrix
  /// using Cholesky factorisation (A = L·Lᵀ), solving A·X = I column-wise.
  static void CholeskyInvertNc(const float A[kNc * kNc],
                                float Ainv[kNc * kNc]) {
    float L[kNc * kNc] = {};

    // Cholesky decomposition.
    for (int i = 0; i < kNc; i++) {
      for (int j = 0; j <= i; j++) {
        float s = A[i * kNc + j];
        for (int k = 0; k < j; k++) {
          s -= L[i * kNc + k] * L[j * kNc + k];
        }
        if (i == j) {
          // If s <= 0 the matrix is not positive-definite (numerical drift).
          // Clamping to a tiny positive value keeps the factorisation
          // well-defined; the resulting approximate inverse degrades
          // gracefully — H is SPD by construction so this should not
          // occur in normal operation.
          L[i * kNc + i] = (s > 0.0f) ? std::sqrt(s) : 1e-12f;
        } else {
          L[i * kNc + j] = s / L[j * kNc + j];
        }
      }
    }

    // Solve A · x_col = e_col for each column of the identity.
    for (int col = 0; col < kNc; col++) {
      float y[kNc] = {};

      // Forward substitution: L · y = e_col
      y[col] = 1.0f / L[col * kNc + col];
      for (int i = col + 1; i < kNc; i++) {
        float s = 0.0f;
        for (int k = col; k < i; k++) { s += L[i * kNc + k] * y[k]; }
        y[i] = -s / L[i * kNc + i];
      }

      // Backward substitution: Lᵀ · x = y
      float x[kNc] = {};
      x[kNc - 1] = y[kNc - 1] / L[(kNc - 1) * kNc + (kNc - 1)];
      for (int i = kNc - 2; i >= 0; i--) {
        float s = y[i];
        for (int k = i + 1; k < kNc; k++) { s -= L[k * kNc + i] * x[k]; }
        x[i] = s / L[i * kNc + i];
      }

      for (int i = 0; i < kNc; i++) { Ainv[i * kNc + col] = x[i]; }
    }
  }

  // ── Precomputed-matrix·vector products ───────────────────────────────────

  /// y = H_reg_inv · x   (Nc × Nc matrix times Nc vector)
  static void MulNcNcVec(const float M[kNc * kNc],
                          const float x[kNc],
                          float y[kNc]) MOTEUS_CCM_ATTRIBUTE {
    for (int i = 0; i < kNc; i++) {
      float s = 0.0f;
      for (int j = 0; j < kNc; j++) { s += M[i * kNc + j] * x[j]; }
      y[i] = s;
    }
  }

  /// y = Mineq · x   (2*Nc × Nc matrix times Nc vector → 2*Nc vector)
  static void MulConNcVec(const float M[kNcon * kNc],
                           const float x[kNc],
                           float y[kNcon]) MOTEUS_CCM_ATTRIBUTE {
    for (int i = 0; i < kNcon; i++) {
      float s = 0.0f;
      for (int j = 0; j < kNc; j++) { s += M[i * kNc + j] * x[j]; }
      y[i] = s;
    }
  }

  /// y = HinvMT · λ   (Nc × 2*Nc matrix times 2*Nc vector → Nc vector)
  static void MulNcConVec(const float M[kNc * kNcon],
                           const float x[kNcon],
                           float y[kNc]) MOTEUS_CCM_ATTRIBUTE {
    for (int i = 0; i < kNc; i++) {
      float s = 0.0f;
      for (int j = 0; j < kNcon; j++) { s += M[i * kNcon + j] * x[j]; }
      y[i] = s;
    }
  }
};

}  // namespace moteus
