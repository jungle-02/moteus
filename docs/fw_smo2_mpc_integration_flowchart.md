# Lưu đồ Giải Thuật Hệ Thống Motor Control với MPC+Kalman+SMO+RLS

## Quy trình thực thi SVPWM thực tế trong firmware moteus

Phần này mô tả đúng luồng chạy thực tế trong firmware (`fw/bldc_servo.cc`):

1. Timer PWM phát sinh update interrupt, vào `GlobalInterrupt()` rồi `ISR_HandleTimer()`.
2. Trong `ISR_DoTimer()`:
   - Trigger ADC đồng bộ bằng `TriggerAllAdcs()`.
   - Chia nhịp điều khiển bằng `phase_ = (phase_ + 1) & rate_config_.interrupt_mask`.
   - Gọi `ISR_DoSenseCritical()` để chốt mẫu quan trọng.
   - Set `PendSV` để chạy phần điều khiển mức ưu tiên thấp.
3. Trong `ISR_DoTimerLowerPriority()` (PendSV):
   - Gọi `ISR_DoSense()` để hoàn tất đọc cảm biến.
   - Tính `electrical_theta`, rồi lấy `sin/cos` bằng `cordic_`.
   - Gọi `ISR_CalculateCurrentState(sin_cos)` để tính dòng dq.
   - Gọi `ISR_DoControl(sin_cos, data)` để chọn và chạy mode điều khiển.
4. Trong `ISR_DoControl()`:
   - Kiểm tra mode/fault/timeout.
   - Chọn nhánh tương ứng (`kVoltageFoc`, `kVoltageDq`, `kCurrent`, `kPosition`, ...).
5. Với nhánh điều khiển điện áp dq:
   - `ISR_CalculatePhaseVoltage()` thực hiện biến đổi ngược dq->abc (`InverseDqTransform`).
6. Thực hiện SVPWM tại `ISR_DoBalancedVoltageControl()` theo min/max injection:
   - Chuẩn hóa điện áp pha: `pwm_in = phase_voltage / bus_V`.
   - Tìm `pwmmin`, `pwmmax`.
   - Tính offset: `offset = 0.5f * (pwmmin + pwmmax) - 0.5f`.
   - Dịch đều 3 pha theo offset (tương đương SVPWM).
7. Xuất PWM phần cứng trong `ISR_DoPwmControl()`:
   - Giới hạn duty bằng `LimitPwm()` theo `min_pwm/max_pwm`.
   - Đổi sang CCR (`pwm_counts_`) và ghi `CCR1/CCR2/CCR3`.
   - Gọi `motor_driver_->PowerOn()`.
8. Kết thúc chu kỳ:
   - `ISR_DoTimerLowerPriority()` enable lại IRQ PWM.
   - Chu kỳ kế tiếp lặp lại theo tần số PWM.

## Tổng Quan Hệ Thống (System Overview)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          MOTEUS FIRMWARE CONTROL SYSTEM                      │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                    Initialization Phase (Startup)                    │   │
│  │                                                                      │   │
│  │  ┌──────────────────┐                                              │   │
│  │  │      Start       │                                              │   │
│  │  │   (Power Up)     │                                              │   │
│  │  └────────┬─────────┘                                              │   │
│  │           │                                                         │   │
│  │           ↓                                                         │   │
│  │  ┌─────────────────────────────────────────────────────┐           │   │
│  │  │      Cấu Hình Ngoại Vi (Configure Peripherals)     │           │   │
│  │  │  - Khởi tạo TIMER2 (20 kHz)                        │           │   │
│  │  │  - Khởi tạo ADC (đo Vbus, I_abc)                   │           │   │
│  │  │  - Khởi tạo Encoder (lấy θ, ω)                     │           │   │
│  │  │  - Khởi tạo PWM (3-phase FOC)                      │           │   │
│  │  └────────┬────────────────────────────────────────────┘           │   │
│  │           │                                                         │   │
│  │           ↓                                                         │   │
│  │  ┌─────────────────────────────────────────────────────┐           │   │
│  │  │   Khởi Tạo Bộ Ước Lượng và Điều Khiển             │           │   │
│  │  │  - Kalman Filter (KF)                              │           │   │
│  │  │  - SMO (Sliding Mode Observer)                     │           │   │
│  │  │  - RLS (Recursive Least Squares)                   │           │   │
│  │  │  - MPC (Model Predictive Controller)               │           │   │
│  │  └────────┬────────────────────────────────────────────┘           │   │
│  │           │                                                         │   │
│  │           ↓                                                         │   │
│  │  ┌─────────────────────────────────────────────────────┐           │   │
│  │  │   Đọc Cấu Hình Từ Bộ Nhớ Lưu Trữ (Load Config)    │           │   │
│  │  │  - MPC tuning parameters (Qw, Qth, Ru)           │           │   │
│  │  │  - Kalman Filter noise matrices                   │           │   │
│  │  │  - SMO gain parameters (k1, k2)                   │           │   │
│  │  │  - RLS forgetting factor (λ)                      │           │   │
│  │  │  - Motor parameters (J, B, flux, poles)           │           │   │
│  │  └────────┬────────────────────────────────────────────┘           │   │
│  │           │                                                         │   │
│  │           └──────────────────────┬──────────────────────┘           │   │
│  │                                  │                                  │   │
│  └──────────────────────────────────┼──────────────────────────────────┘   │
│                                     │                                      │
│                                     ↓                                      │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │              Main Control Loop (20 kHz) - TIMER2 ISR                │  │
│  │                 [Hình vẽ chi tiết bên dưới]                        │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Chi Tiết Vòng Lặp Điều Khiển Chính (20 kHz - TIMER2 ISR)

```
┌────────────────────────────────────────────────────────────────┐
│              Start ngắt TIMER2 (20 kHz)                        │
│              ↓                                                 │
│  ┌────────────────────────────────────────────────────────┐   │
│  │     Đọc Dữ Liệu Cảm Biến                             │   │
│  │     ↓                                                 │   │
│  │  ┌─────────────────────────────────────────────────┐ │   │
│  │  │ 1. Đọc 3-Phase Current: Ia, Ib, Ic [A]         │ │   │
│  │  │    (từ ADC - Current Sense)                     │ │   │
│  │  └────────────┬────────────────────────────────────┘ │   │
│  │              │                                       │   │
│  │              ↓                                       │   │
│  │  ┌─────────────────────────────────────────────────┐ │   │
│  │  │ 2. Đọc Bus Voltage: Vbus [V]                   │ │   │
│  │  │    (từ ADC - Voltage Sense)                     │ │   │
│  │  └────────────┬────────────────────────────────────┘ │   │
│  │              │                                       │   │
│  │              ↓                                       │   │
│  │  ┌─────────────────────────────────────────────────┐ │   │
│  │  │ 3. Đọc Encoder & Lộc Nhiễu                     │ │   │
│  │  │    - ω_measured [rad/s]                        │ │   │
│  │  │    - θ_measured [rad] (Electrical Angle)       │ │   │
│  │  │    - Position [rev]                            │ │   │
│  │  │                                                 │ │   │
│  │  │    Kiểm tra: Quá Động? Quá Tốc?              │ │   │
│  │  └────────────┬────────────────────────────────────┘ │   │
│  │              │                                       │   │
│  │              ↓                                       │   │
│  │              ◇────────────────────────────────◇      │   │
│  │         Quá Động?   Quá Tốc?             Y          │   │
│  │         /                  \                │        │   │
│  │    Y  /                      \N           │        │   │
│  │      /                        \           ↓        │   │
│  │   Fault ◄──────────────────── ┘       Tiếp tục    │   │
│  │                                          │        │   │
│  │                                          ↓        │   │
│  └────────────────────────────────────────────────────┘   │
│                                                            │
│  ┌────────────────────────────────────────────────────────┐   │
│  │        MPC Enable?                                    │   │
│  │        (Cờ điều khiển)                               │   │
│  │        │                                              │   │
│  │   Y    │                                    N         │   │
│  │  ╱─────◇─────╲                                        │   │
│  │             ╱──────────────────────────────╲         │   │
│  │            │                                 │        │   │
│  │            ↓                                 ↓        │   │
│  │   ┌────────────────┐            ┌────────────────┐   │   │
│  │   │ BLOCK A:       │            │ BLOCK B:       │   │   │
│  │   │ MPC+KF+SMO+RLS │            │ Standard FOC   │   │   │
│  │   │ (5 kHz)        │            │ Control Mode   │   │   │
│  │   │                │            │ (20 kHz)       │   │   │
│  │   │ Chạy mỗi 4x    │            │                │   │   │
│  │   │ chu kỳ         │            │ - PI Controller│   │   │
│  │   │                │            │ - Clarke/Park  │   │   │
│  │   └────┬───────────┘            │ - SVPWM        │   │   │
│  │        │                        └────┬───────────┘   │   │
│  │        ↓                            │               │   │
│  └─────────┴────────────────────────────┴─────────────┘   │
│            │                                   │           │
│            ↓                                   ↓           │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  [BLOCK A - Chi Tiết bên dưới]                     │  │
│  └──────────────────────────────────────────────────────┘  │
│            │                                                │
│            ↓                                                │
│  ┌──────────────────────────────────────────────────────┐  │
│  │          Chuẩn Bị Lệnh Điều Khiển                   │  │
│  │          (Prepare Control Command)                  │  │
│  │          ↓                                           │  │
│  │  ┌────────────────────────────────────────────────┐ │  │
│  │  │ • Lấy Iq_ref từ MPC (nếu MPC Enable)          │ │  │
│  │  │ • Lấy Id_ref = 0 (Field-weakening nếu cần)   │ │  │
│  │  │ • Tính điện áp tương ứng (Park → abc)          │ │  │
│  │  │ • Giới hạn PWM duty cycle [0, 1]              │ │  │
│  │  └────────┬───────────────────────────────────────┘ │  │
│  │           │                                         │  │
│  │           ↓                                         │  │
│  │  ┌────────────────────────────────────────────────┐ │  │
│  │  │   Ghi PWM Duty Cycle Vào Thanh Ghi             │ │  │
│  │  │   (Update PWM Registers)                        │ │  │
│  │  │   - PWM1, PWM2, PWM3 (3-phase)                │ │  │
│  │  └────────┬───────────────────────────────────────┘ │  │
│  │           │                                         │  │
│  │           ↓                                         │  │
│  │  ┌────────────────────────────────────────────────┐ │  │
│  │  │   Chuẩn Bị Dữ Liệu Telemetry                  │ │  │
│  │  │   - Tốc độ ước lượng từ MPC                    │ │  │
│  │  │   - Góc ước lượng từ SMO                       │ │  │
│  │  │   - Mô-men xoắn ước lượng từ RLS              │ │  │
│  │  │   - Trạng thái MPC flag & status               │ │  │
│  │  └────────┬───────────────────────────────────────┘ │  │
│  │           │                                         │  │
│  └───────────┴─────────────────────────────────────────┘  │
│              │                                             │
│              ↓                                             │
│              ◇──────────────────────────────────◇         │
│         Cần gửi dữ liệu?                  Y           │
│              │                            │              │
│              └────────────────────────────┘              │
│                                           │              │
│              N                            ↓              │
│              │                 ┌──────────────────┐     │
│              │                 │ Gửi Telemetry   │     │
│              │                 │ qua UART/CAN    │     │
│              │                 └────┬─────────────┘     │
│              │                      │                  │
│              └──────────────────────┤─────────────────┘  │
│                                     │                   │
│                                     ↓                   │
│                                ┌──────────┐            │
│                                │   End    │            │
│                                │  ISR     │            │
│                                └──────────┘            │
└────────────────────────────────────────────────────────────┘
```

---

## BLOCK A - Chi Tiết MPC+Kalman+SMO+RLS (5 kHz / mỗi 4 chu kỳ)

```
┌──────────────────────────────────────────────────────────────────┐
│           MPC Enable = 1  &&  MPC_Flag++ >= 4?                  │
│           (Chạy mỗi 4 chu kỳ 20kHz = 5kHz)                      │
│                                                                  │
│                          Y                                       │
│                          ↓                                       │
│           ┌──────────────────────────────┐                      │
│           │   MPC_Flag = 0               │                      │
│           │   (Reset counter)            │                      │
│           └────────────┬─────────────────┘                      │
│                        │                                        │
│                        ↓                                        │
│        ╔═══════════════════════════════════════════════════╗   │
│        ║      BLOCK A.1: SMO - Sliding Mode Observer      ║   │
│        ║      (5 kHz - State Estimator)                    ║   │
│        ║                                                  ║   │
│        ║  ┌────────────────────────────────────────────┐ ║   │
│        ║  │ Input:                                     │ ║   │
│        ║  │  - ω_measured [rad/s] (từ encoder)       │ ║   │
│        ║  │  - θ_measured [rad]   (từ encoder)       │ ║   │
│        ║  │  - Ia, Ib, Ic [A]      (từ ADC)          │ ║   │
│        ║  │  - Vdc [V]             (từ ADC)          │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Process:                                   │ ║   │
│        ║  │ 1. Chuyển đổi abc → dq (Park Transform) │ ║   │
│        ║  │    - Iq_actual = (2/3)*p*flux*Te/Nm     │ ║   │
│        ║  │    - Id_actual (từ điều khiển)          │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 2. Tính BEMF (Back EMF) từ mô hình       │ ║   │
│        ║  │    - e_d = -ω*Lq*Iq + Vd (mô hình)     │ ║   │
│        ║  │    - e_q = ω*Ld*Id + flux*ω + Vq       │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 3. Tính Signal Switching (Relay Output) │ ║   │
│        ║  │    - s_d = sign(e_d - e_d_est)         │ ║   │
│        ║  │    - s_q = sign(e_q - e_q_est)         │ ║   │
│        ║  │    K = [k1, k2] (SMO gain)             │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 4. Cập Nhật Trạng Thái SMO              │ ║   │
│        ║  │    dx_est/dt = Ax_est + B*u + K*s      │ ║   │
│        ║  │    (Euler integration)                  │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Output:                                   │ ║   │
│        ║  │  - ω_smo [rad/s]  (Speed from SMO)      │ ║   │
│        ║  │  - Te_est [Nm]    (Torque estimate)     │ ║   │
│        ║  │  - θ_back_emf [rad] (BEMF-based angle)  │ ║   │
│        ║  └────────────────────────────────────────┘ ║   │
│        ║                                               ║   │
│        └────────────────┬────────────────────────────┘   │   │
│                         │                                │   │
│                         ↓                                │   │
│        ╔═══════════════════════════════════════════════════╗   │
│        ║    BLOCK A.2: RLS - Recursive Least Squares      ║   │
│        ║    (5 kHz - Load Torque Estimation)              ║   │
│        ║                                                  ║   │
│        ║  ┌────────────────────────────────────────────┐ ║   │
│        ║  │ Input:                                     │ ║   │
│        ║  │  - ω_smo [rad/s]    (từ SMO)             │ ║   │
│        ║  │  - ω_filtered (lowpass)                   │ ║   │
│        ║  │  - Iq [A]           (current command)     │ ║   │
│        ║  │  - Ts [s]           (sampling time)       │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Process:                                   │ ║   │
│        ║  │ Mô hình tuyến tính:                       │ ║   │
│        ║  │  J*dω/dt = Kt*Iq - B*ω - Te_load        │ ║   │
│        ║  │  dω/dt = (Kt/J)*Iq - (B/J)*ω - Te/J     │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Dạng regression:                         │ ║   │
│        ║  │  y = [a1, a2, a3] * [Iq, ω, 1]ᵀ        │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ RLS Update:                               │ ║   │
│        ║  │  K(k) = P(k-1)*φ/(λ + φᵀ*P(k-1)*φ)     │ ║   │
│        ║  │  θ(k) = θ(k-1) + K(k)*[y(k) - φᵀ*θ]    │ ║   │
│        ║  │  P(k) = (1/λ)*[P(k-1) - K(k)*φᵀ*P(k-1)]│ ║   │
│        ║  │  λ ∈ [0.95, 0.99] (forgetting factor)  │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Output:                                   │ ║   │
│        ║  │  - θ_rls = [Kt/J, B/J, Te/J]            │ ║   │
│        ║  │  - Te_load_est [Nm] (từ: J*a3)          │ ║   │
│        ║  │  - Kt_est, B_est (system parameters)    │ ║   │
│        ║  └────────────────────────────────────────┘ ║   │
│        ║                                               ║   │
│        └────────────────┬────────────────────────────┘   │   │
│                         │                                │   │
│                         ↓                                │   │
│        ╔═══════════════════════════════════════════════════╗   │
│        ║    BLOCK A.3: Kalman Filter State Fusion        ║   │
│        ║    (5 kHz - Fuse all estimates)                 ║   │
│        ║                                                  ║   │
│        ║  ┌────────────────────────────────────────────┐ ║   │
│        ║  │ Input:                                     │ ║   │
│        ║  │  - ω_measured   (từ encoder)              │ ║   │
│        ║  │  - ω_smo        (từ SMO)                  │ ║   │
│        ║  │  - θ_measured   (từ encoder)              │ ║   │
│        ║  │  - θ_back_emf   (từ SMO)                  │ ║   │
│        ║  │  - Te_load_est  (từ RLS)                  │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Kalman Filter Formulation:                 │ ║   │
│        ║  │ State vector: x = [ω, θ, Te_load]ᵀ       │ ��   │
│        ║  │                                           │ ║   │
│        ║  │ Prediction Step:                           │ ║   │
│        ║  │  x_pred(k) = A*x_est(k-1) + B*u         │ ║   │
│        ║  │  P_pred = A*P_est*Aᵀ + Q                │ ║   │
│        ║  │  where: A = [1-B*Ts/J,  0,    -Ts/J;  │ ║   │
│        ║  │              Ts,         1,     0;     │ ║   │
│        ║  │              0,          0,     1]     │ ║   │
│        ║  │         B = [Ts*Kt/(J), 0, 0]ᵀ         │ ║   │
│        ║  │         Q = diag([q_ω, q_θ, q_load])   │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Update Step:                               │ ║   │
│        ║  │  Measurement: z = [ω_measured;           │ ║   │
│        ║  │                    θ_measured]           │ ║   │
│        ║  │  Innovation: y = z - C*x_pred            │ ║   │
│        ║  │  S = C*P_pred*Cᵀ + R                    │ ║   │
│        ║  │  K_gain = P_pred*Cᵀ/S  (Kalman gain)   │ ║   │
│        ║  │  x_est = x_pred + K_gain*y              │ ║   │
│        ║  │  P_est = (I - K_gain*C)*P_pred          │ ║   │
│        ║  │  where: C = [1 0 0; 0 1 0]             │ ║   │
│        ║  │         R = diag([r_ω, r_θ])           │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Tuning Parameters:                        │ ║   │
│        ║  │  q_ω = 1.0         (state process noise) │ ║   │
│        ║  │  q_θ = 1e-6        (angle process noise) │ ║   │
│        ║  │  q_load = 0.5      (load process noise)  │ ║   │
│        ║  │  r_ω = 5e-4        (speed meas. noise)   │ ║   │
│        ║  │  r_θ = 1e-4        (angle meas. noise)   │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Output:                                   │ ║   │
│        ║  │  - ω_fused [rad/s]  (Fused speed)        │ ║   │
│        ║  │  - θ_fused [rad]    (Fused angle)        │ ║   │
│        ║  │  - Te_load_fused [Nm] (Fused load torque)│ ║   │
│        ║  │  - x_hat, P_est     (State & covariance) │ ║   │
│        ║  └────────────────────────────────────────┘ ║   │
│        ║                                               ║   │
│        └────────────────┬────────────────────────────┘   │   │
│                         │                                │   │
│                         ↓                                │   │
│        ╔═══════════════════════════════════════════════════╗   │
│        ║      BLOCK A.4: MPC - Model Predictive Controller║   │
│        ║      (5 kHz - Optimal Control Law)              ║   │
│        ║                                                  ║   │
│        ║  ┌────────────────────────────────────────────┐ ║   │
│        ║  │ Input:                                     │ ║   │
│        ║  │  - ω_fused [rad/s]       (from KF)        │ ║   │
│        ║  │  - θ_fused [rad]         (from KF)        │ ║   │
│        ║  │  - ω_ref [rad/s]         (setpoint)       │ ║   │
│        ║  │  - θ_ref [rad]           (setpoint)       │ ║   │
│        ║  │  - Te_load_fused [Nm]    (from KF)        │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Process:                                   │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 1. Build Augmented System Model            │ ║   │
│        ║  │    Augmented State: x_aug = [ω, θ, u]     │ ║   │
│        ║  │    A_aug = [1-B*Ts/J, 0, -Ts/J;          │ ║   │
│        ║  │            Ts, 1, 0;                      │ ║   │
│        ║  │            0, 0, 1]                       │ ║   │
│        ║  │    B_aug = [Ts*Kt/J, 0, 1]ᵀ              │ ║   │
│        ║  │    C = [1, 0, 0; 0, 1, 0]                │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 2. Build Prediction & Cost Matrices        │ ║   │
│        ║  │    Prediction Horizon: Np = 25 steps      │ ║   │
│        ║  │    Control Horizon: Nc = 8 steps          │ ║   │
│        ║  │    Ts = 200μs (5 kHz)                    │ ║   │
│        ║  │                                           │ ║   │
│        ║  │    Build F, G matrices:                   │ ║   │
│        ║  │    Ŷ = F*x_aug + G*ΔU                    │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 3. Build Reference Trajectory             │ ║   │
│        ║  │    Reference vector Rs (2*Np x 1):       │ ║   │
│        ║  │    Rs = [ω_ref(1), θ_ref(1),             │ ║   │
│        ║  │          ω_ref(2), θ_ref(2),             │ ║   │
│        ║  │          ...,                            │ ║   │
│        ║  │          ω_ref(Np), θ_ref(Np)]           │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 4. Build Cost Function                     │ ║   │
│        ║  │    J = ||Y - Rs||²_Q + ||ΔU||²_R         │ ║   │
│        ║  │    Q = diag(Qw*ones(Np,1), Qth*ones())   │ ║   │
│        ║  │    R = Ru*eye(Nc)                        │ ║   │
│        ║  │                                           │ ║   │
│        ║  │    Quadratic form:                        │ ║   │
│        ║  │    J = (1/2)*ΔUᵀ*H*ΔU + fᵀ*ΔU + const   │ ║   │
│        ║  │    H = 2*(Gᵀ*Q*G + R)                    │ ║   │
│        ║  │    f = 2*Gᵀ*Q*(F*x_aug - Rs)            │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 5. Build Constraints (Torque Saturation)  │ ║   │
│        ║  │    -Te_max ≤ Te_cmd ≤ Te_max            │ ║   │
│        ║  │    -Iq_max ≤ Iq_cmd ≤ Iq_max            │ ║   │
│        ║  │                                           │ ║   │
│        ║  │    Constraint matrices:                   │ ║   │
│        ║  │    M*ΔU ≤ N (inequality constraints)     │ ║   │
│        ║  │    M = [I_Nc; -I_Nc]  (upper & lower)   │ ║   │
│        ║  │    N = [Te_max*ones; Te_max*ones]        │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 6. Solve QP Problem (Hildreth Algorithm)  │ ║   │
│        ║  │    min (1/2)*ΔUᵀ*H*ΔU + fᵀ*ΔU          │ ║   │
│        ║  │    s.t. M*ΔU ≤ N                         │ ║   │
│        ║  │                                           │ ║   │
│        ║  │    Algorithm:                             │ ║   │
│        ║  │    while not converged:                   │ ║   │
│        ║  │      for i = 1 to 2*Nc:                  │ ║   │
│        ║  │        λ(i) = max(0, λ(i) + (W(i,:)*λ) + │ ║   │
│        ║  │                   (N(i) - M(i,:)*ΔU))    │ ║   │
│        ║  │      ΔU = -H⁻¹*(f + Mᵀ*λ)               │ ║   │
│        ║  │      if ||Δλ|| < tol: break              │ ║   │
│        ║  │    max_iter = 200, tolerance = 1e-9      │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ 7. Extract First Control Action           │ ║   │
│        ║  │    Te_cmd = Te_prev + ΔU(1)             │ ║   │
│        ║  │    Saturate: Te_cmd ∈ [-Te_max, Te_max] │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ MPC Parameters:                           │ ║   │
│        ║  │  Np = 25, Nc = 8                        │ ║   │
│        ║  │  Qw = 300    (speed tracking weight)     │ ║   │
│        ║  │  Qth = 0.5   (angle tracking weight)     │ ║   │
│        ║  │  Ru = 1.8    (input effort weight)       │ ║   │
│        ║  │  Iq_max = 12 A (max current)             │ ║   │
│        ║  │  Te_max = 0.49 Nm (max torque)           │ ║   │
│        ║  │                                           │ ║   │
│        ║  │ Output:                                   │ ║   │
│        ║  │  - Iq_ref [A]         (Q-axis current)   │ ║   │
│        ║  │  - Id_ref = 0 [A]     (D-axis current)   │ ║   │
│        ║  │  - Te_mpc [Nm]        (Commanded torque) │ ║   │
│        ║  │  - mpc_status flags                      │ ║   │
│        ║  └────────────────────────────────────────┘ ║   │
│        ║                                               ║   │
│        └────────────────┬────────────────────────────┘   │   │
│                         │                                │   │
│                         ↓                                │   │
│           ┌──────────────────────────────┐              │   │
│           │  Lưu Trữ Kết Quả Cho Vòng   │              │   │
│           │  Lặp 20 kHz Tiếp Theo       │              │   │
│           │                              │              │   │
│           │ - ω_fused, θ_fused         │              │   │
│           │ - Iq_ref, Id_ref           │              │   │
│           │ - Te_mpc                   │              │   │
│           │ - SMO/KF state estimates   │              │   │
│           │ - Flags & diagnostic data  │              │   │
│           └────────────┬─────────────────┘              │   │
│                        │                                │   │
│                        ↓                                │   │
│                   [Kết Thúc BLOCK A]                   │   │
│                   (Quay lại Main Loop)                 │   │
│                                                        │   │
└──────────────────────────────────────────────────────────┘
```

---

## Bảng So Sánh Các Bộ Ước Lượng

| Bộ Ước Lượng | Tần Suất | Đầu Vào | Đầu Ra | Ưu Điểm | Nhược Điểm |
|---|---|---|---|---|---|
| **SMO** | 5 kHz | ω_meas, θ_meas, I_abc, Vdc | ω_smo, θ_back_emf, Te_est | Nhanh, không cần mô hình chính xác | Nhiễu từ switching |
| **RLS** | 5 kHz | ω, Iq, Ts | Kt, B, Te_load | Tự thích nghi, ước lượng tải | Yêu cầu regressor tốt |
| **KF** | 5 kHz | ω_measured, θ_measured, ω_smo, Te_load | ω_fused, θ_fused, Te_load_fused | Tối ưu, lý thuyết vững | Tính toán phức tạp |
| **MPC** | 5 kHz | ω_fused, θ_fused, ω_ref, θ_ref | Iq_ref, Id_ref, Te_mpc | Tối ưu, xử lý ràng buộc | Tính toán hạn chế MCU |

---

## Cấu Hình Tham Số Mặc Định

### Motor Parameters
```
poles = 14
J = 2.94e-5 kg⋅m²
B = 1e-4 N⋅m⋅s
flux = 0.0185 Wb
Kt = (2/3)*p*flux = 0.0868 N⋅m/A
```

### MPC Configuration
```
Np = 25 steps (prediction horizon)
Nc = 8 steps (control horizon)
Ts = 200 μs (sampling time = 1/5kHz)
Qw = 300 (speed tracking weight)
Qth = 0.5 (angle tracking weight)
Ru = 1.8 (input effort weight)
Iq_max = 12 A
Te_max = 1.5 * p * flux * Iq_max = 0.49 Nm
```

### Kalman Filter Tuning
```
q_ω = 1.0 (state process noise - speed)
q_θ = 1e-6 (state process noise - angle)
q_load = 0.5 (state process noise - load torque)
r_ω = 5e-4 (measurement noise - speed)
r_θ = 1e-4 (measurement noise - angle)
```

### SMO Tuning
```
k1 = 100 (sliding surface gain)
k2 = 10 (integral gain)
```

### RLS Tuning
```
λ = 0.98 (forgetting factor)
P_init = 100*I (initial covariance)
```

---

## Chế Độ Hoạt Động

### Mode 1: MPC Enable (MPC_Flag++ >= 4)
- **Tần suất:** 5 kHz (chạy mỗi 4 chu kỳ 20 kHz)
- **Bộ ước lượng:** SMO + RLS + KF
- **Điều khiển:** MPC
- **Lợi ích:** Hiệu năng cao, tối ưu hóa

### Mode 2: Standard FOC (MPC Disable)
- **Tần suất:** 20 kHz
- **Bộ ước lượng:** Chỉ sử dụng encoder
- **Điều khiển:** PI thông thường
- **Lợi ích:** Tính toán đơn giản, backup khi MPC lỗi

---

## Yêu Cầu Thời Gian Thực (Real-Time Constraints)

```
TIMER2 ISR (20 kHz):
  - ADC read: ~50 μs
  - Encoder processing: ~20 μs
  - Fault check: ~10 μs
  - Standard FOC control: ~500 μs (available)
  - Total: ~580 μs (deadline: 50 μs)

MPC Block (5 kHz sub-loop):
  - SMO computation: ~200 μs
  - RLS update: ~150 μs
  - KF prediction/update: ~300 μs
  - MPC QP solve: ~1000 μs (critical)
  - Total: ~1650 μs (deadline: 200 μs = 4*50μs)
  - Must run in background or reduced update rate
```

---

## Cơ Chế Failsafe

```
┌─────────────────────────┐
│  Fault Detection        │
└────────────┬────────────┘
             │
             ↓
    ◇─ Motor Overspeed?
    /│\
   / │ \
  Y  N  \
  │     └─ Over Current?
  │        /│\
  ├─N─────/ │ \
  │  ◇ KF Divergence?
  │  /│\
  ├─Y │ \──N───┐
  │    \        │
  ↓     └───┐   ↓
 Fault      ↓  Pass
         Fault

  Result: Disable MPC, switch to Standard FOC mode
```

---

## Tổng Kết Kiến Trúc

- **Tầng 1 (20 kHz):** Sampling & Basic FOC control
- **Tầng 2 (5 kHz):** State estimation (SMO + RLS + KF)
- **Tầng 3 (5 kHz):** Optimal control (MPC)
- **Failsafe:** Automatic degradation to FOC if error detected

Cấu trúc này cho phép hệ thống:
✓ Hoạt động thời gian thực
✓ Ước lượng trạng thái chính xác
✓ Điều khiển tối ưu với ràng buộc
✓ Suy giảm nhẹ khi xảy ra lỗi
