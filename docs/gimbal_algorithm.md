# Gimbal Algorithm — Giải thích chi tiết thuật toán

**Project:** STM32G4_Lighting — Gimbal 2 trục (Pitch + Yaw)  
**MCU:** STM32G431CBUx | **Control Rate:** 500 Hz | **Date:** 2026-05-20

---

## Mục lục

1. [Tổng quan kiến trúc điều khiển](#1-tổng-quan-kiến-trúc-điều-khiển)
2. [Tại sao dùng 2 IMU?](#2-tại-sao-dùng-2-imu)
3. [Bộ lọc Complementary Filter](#3-bộ-lọc-complementary-filter)
4. [Relative Error — Sai số tương đối](#4-relative-error--sai-số-tương-đối)
5. [Feedforward Control](#5-feedforward-control)
6. [Cascaded PID — Trục Pitch](#6-cascaded-pid--trục-pitch)
7. [Rate PID — Trục Yaw](#7-rate-pid--trục-yaw)
8. [Servo Output Mapping](#8-servo-output-mapping)
9. [Vòng lặp 500 Hz — Cơ chế Flag-based](#9-vòng-lặp-500-hz--cơ-chế-flag-based)
10. [Luồng xử lý tổng thể](#10-luồng-xử-lý-tổng-thể)
11. [Bảng thông số mặc định](#11-bảng-thông-số-mặc-định)
12. [Hướng dẫn Tuning PID](#12-hướng-dẫn-tuning-pid)

---

## 1. Tổng quan kiến trúc điều khiển

```mermaid
graph TD
    subgraph Hardware["PHẦN CỨNG"]
        FR_HW["Frame IMU (MPU6500)\nI2C addr: 0x68\nGắn trên khung/body"]
        CA_HW["Camera IMU (MPU6050)\nI2C addr: 0x69\nGắn trên platform camera"]
        SRV_P["Servo Pitch\nTIM3 CH2 - PA4"]
        SRV_Y["Servo Yaw\nTIM3 CH3 - PB0"]
    end

    subgraph Fusion["LỌC & SENSOR FUSION"]
        CF_F["Complementary Filter\nFrame: pitch_f, roll_f"]
        CF_C["Complementary Filter\nCamera: pitch_c, roll_c"]
    end

    subgraph Control["BỘ ĐIỀU KHIỂN"]
        REL["Relative Error\nerr_p = pitch_c - pitch_f - SP\nerr_y = gyro_z_c - gyro_z_f"]
        FF["Feedforward\nKff × frame_gyro"]
        PID_A["Angle PID\nVòng ngoài Pitch"]
        PID_R["Rate PID\nVòng trong Pitch"]
        PID_Y["Rate PID\nYaw"]
        SUM_P["Σ Pitch"]
        SUM_Y["Σ Yaw"]
    end

    FR_HW -->|Accel + Gyro| CF_F
    CA_HW -->|Accel + Gyro| CF_C
    CF_F --> REL
    CF_C --> REL
    REL -->|err_pitch| PID_A
    PID_A -->|rate_sp| PID_R
    CA_HW -.->|cam_gyro_x| PID_R
    FR_HW -.->|frame_gyro| FF
    PID_R --> SUM_P
    FF --> SUM_P
    FF --> SUM_Y
    REL -->|err_yaw_rate| PID_Y
    PID_Y --> SUM_Y
    SUM_P -->|offset µs| SRV_P
    SUM_Y -->|offset µs| SRV_Y
```

---

## 2. Tại sao dùng 2 IMU?

### Với 1 IMU (trên camera) — Phương pháp cũ
```mermaid
sequenceDiagram
    participant Frame
    participant Camera
    participant PID

    Frame->>Camera: Rung lắc truyền lên
    Note over Camera: Camera đã bị lệch!
    Camera->>PID: Gửi sai số (muộn)
    PID->>Camera: Điều chỉnh servo (càng muộn hơn)
    Note over PID: Latency = I2C + PID + Servo slew
```

**Nhược điểm:** PID phải **đợi camera bị lệch** mới biết để phản ứng → luôn chậm 1 bước.

### Với 2 IMU — Phương pháp hiện tại
```mermaid
sequenceDiagram
    participant Frame as Frame IMU (0x68)
    participant Camera as Camera IMU (0x69)
    participant FF as Feedforward
    participant PID as Cascaded PID

    Frame->>FF: frame_gyro_x (NGAY LẬP TỨC!)
    FF->>FF: servo_offset += Kff × frame_gyro
    Note over FF: Phản ứng tức thì, không đợi error
    Frame->>Camera: Rung lắc (nếu có)
    Camera->>PID: Gửi sai số tương đối
    PID->>PID: Bù phần dư còn lại
    FF-->>Frame: Đã bù phần lớn trước rồi
```

**Ưu điểm:** Feedforward từ Frame IMU bù **trước** khi camera bị lệch → hệ thống nhanh hơn và mượt hơn.

---

## 3. Bộ lọc Complementary Filter

### Vấn đề với từng cảm biến đơn lẻ

```mermaid
graph LR
    subgraph Gyro["GYROSCOPE ✅ Nhanh ❌ Drift"]
        G1["Tốc độ góc\n(°/s)"] --> G2["Tích phân\n× dt"] --> G3["Góc tức thì\n(chính xác ngắn hạn)"]
        G3 -.->|"Sai số tích lũy\ntheo thời gian"| G4["❌ DRIFT"]
    end

    subgraph Accel["ACCELEROMETER ✅ Ổn định ❌ Nhiễu"]
        A1["Gia tốc\n(g)"] --> A2["atan2(ax, √ay²+az²)"] --> A3["Góc tuyệt đối\n(ổn định dài hạn)"]
        A3 -.->|"Bị nhiễu khi\nrung cơ học"| A4["❌ NOISE"]
    end
```

### Công thức Complementary Filter

```
angle[t] = α × (angle[t-1] + gyro × dt)  +  (1 - α) × accel_angle[t]
            \_______________________/         \_____________________/
               Phần High-Pass                    Phần Low-Pass
               (tin Gyro, phản ứng nhanh)       (tin Accel, chống drift)
```

### Flowchart `CompFilter_Update()`

```mermaid
flowchart TD
    START(["CompFilter_Update(ax, ay, az, gx, gy, dt)"])
    --> ACCEL["Tính góc từ Accelerometer:\naccel_pitch = atan2(ax, √ay²+az²) × 57.3\naccel_roll  = atan2(ay, az) × 57.3"]
    --> INIT_CHK{initialized == 0?}

    INIT_CHK -- "Lần đầu tiên\n(tránh bump khi bật)" --> SET_INIT["pitch = accel_pitch\nroll  = accel_roll\ninitialized = 1"]
    SET_INIT --> RETURN_INIT(["return"])

    INIT_CHK -- "Đã chạy rồi" --> CF["Complementary Filter:\npitch = α×(pitch + gx×dt) + (1-α)×accel_pitch\nroll  = α×(roll  + gy×dt) + (1-α)×accel_roll"]
    CF --> RETURN_CF(["return — pitch, roll đã cập nhật"])
```

> **Giá trị α = 0.98** được dùng trong project:
> - Time constant: τ = α / ((1-α) × f) = 0.98 / (0.02 × 500Hz) ≈ **0.098 giây**
> - Nghĩa là drift của gyro được Accel bù lại sau ~0.1 giây

---

## 4. Relative Error — Sai số tương đối

### Tại sao dùng sai số tương đối thay vì sai số tuyệt đối?

```mermaid
graph TD
    subgraph Scenario["Kịch bản: Frame nghiêng +10°"]
        F["Frame: pitch_f = +10°"]
        C["Camera: pitch_c = +2° (bị kéo nhẹ)"]
        SP["Setpoint: 0° (muốn camera nhìn thẳng)"]
    end

    subgraph Absolute["❌ Sai số TUYỆT ĐỐI (1 IMU)"]
        ERR_ABS["err = setpoint - cam_pitch\n    = 0 - 2 = -2°\n→ Servo chỉ bù 2°, bỏ qua frame rung!"]
    end

    subgraph Relative["✅ Sai số TƯƠNG ĐỐI (2 IMU)"]
        ERR_REL["err = (cam_pitch - frame_pitch) - setpoint\n    = (2 - 10) - 0 = -8°\n→ Servo bù đúng 8° để camera về 0° tuyệt đối"]
    end

    Scenario --> Absolute
    Scenario --> Relative
```

### Công thức tính Error trong code

```c
// Pitch: dùng góc tuyệt đối từ Complementary Filter
err_pitch = (cf_camera.pitch - cf_frame.pitch) - pitch_setpoint

// Yaw: dùng gyro rate (không có accel reference cho trục Z)
err_yaw = (cam_gyro_z - frame_gyro_z) - yaw_rate_setpoint
```

> **Lưu ý Yaw:** Không thể tính góc Yaw từ Accelerometer (trọng lực thẳng đứng, không có tham chiếu ngang). Vì vậy Yaw chỉ dùng **Rate PID** (lock tốc độ quay = 0).

---

## 5. Feedforward Control

### Nguyên lý

```mermaid
graph LR
    subgraph FB["Feedback Only (PID thuần)"]
        ERR1["Sai số\n(muộn)"] --> PID1["PID"] --> OUT1["Servo\n(phản ứng chậm)"]
    end

    subgraph FF_diagram["Feedback + Feedforward"]
        FRAME["Frame gyro\n(tức thì!)"] -->|"Kff × gyro_rate"| SUM2["Σ"]
        ERR2["Sai số\n(bù phần dư)"] --> PID2["PID"] --> SUM2
        SUM2 --> OUT2["Servo\n(phản ứng nhanh)"]
    end
```

### Tại sao dùng Gyro Rate cho Feedforward thay vì Angle?

| | Gyro Rate | Angle (từ CF) |
|---|---|---|
| **Độ trễ** | Tức thì (0 trễ) | ~0.1s (time constant của CF) |
| **Drift** | Không drift ngắn hạn | Không drift (CF đã bù) |
| **Phù hợp FF?** | ✅ Tốt nhất | ❌ Có trễ |

---

## 6. Cascaded PID — Trục Pitch

Trục Pitch dùng **2 vòng PID lồng nhau** (Cascaded) để đạt phản ứng mượt và chính xác hơn 1 vòng PID đơn.

```mermaid
graph LR
    SP["Setpoint\n0° (mặc định)"]
    ERR_P["err_pitch\n= cam_p - frame_p - SP"]
    SP --> ERR_P

    subgraph OUTER["VÒNG NGOÀI — Angle PID"]
        ANG_PID["Angle PID\nKp=3.5 Ki=0.02 Kd=0.05\nOutput clamp: ±120 °/s"]
    end

    subgraph INNER["VÒNG TRONG — Rate PID"]
        RATE_ERR["rate_error\n= rate_sp - cam_gyro_x"]
        RATE_PID["Rate PID\nKp=1.8 Ki=0.15 Kd=0.015\nOutput clamp: ±800 µs"]
    end

    FF_P["Feedforward\n+ Kff_pitch × frame_gyro_x"]

    SUM(["Σ"])
    SERVO["Servo Pitch\n1500 ± offset µs"]

    ERR_P --> ANG_PID
    ANG_PID -->|"rate_sp (°/s)"| RATE_ERR
    RATE_ERR --> RATE_PID
    RATE_PID -->|"offset µs"| SUM
    FF_P --> SUM
    SUM --> SERVO
```

### Lý do dùng Cascaded thay vì 1 PID đơn

| | 1 PID đơn | Cascaded PID |
|---|---|---|
| Input → Output | Angle → Servo µs | Angle → Rate → Servo µs |
| **Ổn định** | Khó tuning, dễ dao động | Dễ tuning từng vòng riêng |
| **Phản ứng** | Có thể overshoot góc | Rate PID giới hạn tốc độ servo |
| **Chống nhiễu** | Kém | Tốt hơn — Rate feedback triệt nhiễu cơ học |

### Công thức PID rời rạc (triển khai trong `pid.c`)

```
error[k]   = setpoint - measurement

P = Kp × error[k]
I = I_prev + Ki × error[k] × dt        ← anti-windup: clamp I
D = Kd × (error[k] - error[k-1]) / dt  ← lọc EMA: D_filt = α×D_filt + (1-α)×D

output = P + I + D_filt                 ← clamp output_min..output_max
```

---

## 7. Rate PID — Trục Yaw

Yaw chỉ dùng **1 vòng Rate PID** (không có Angle PID) vì:
- Không có tham chiếu góc Yaw tuyệt đối từ gia tốc kế
- Mục tiêu: **lock yaw rate = 0** (kháng lại xoay ngang)

```mermaid
graph LR
    SP_Y["Setpoint\n0 °/s\n(không xoay)"]
    ERR_Y["err_yaw_rate\n= cam_gyro_z - frame_gyro_z - SP"]
    SP_Y --> ERR_Y

    subgraph SINGLE["VÒNG ĐƠN — Rate PID"]
        YPID["Rate PID\nKp=1.5 Ki=0.08 Kd=0.01\nOutput clamp: ±800 µs"]
    end

    FF_Y["Feedforward\n+ Kff_yaw × frame_gyro_z"]
    SUM_Y(["Σ"])
    SRV_Y["Servo Yaw\n1500 ± offset µs"]

    ERR_Y --> YPID
    YPID -->|"offset µs"| SUM_Y
    FF_Y --> SUM_Y
    SUM_Y --> SRV_Y
```

---

## 8. Servo Output Mapping

```mermaid
graph LR
    OFFSET["PID + FF output\n(float, đơn vị µs)"]
    SIGN["× SIGN\n+1 hoặc -1\n(đảo chiều servo)"]
    CENTER["+ CENTER\n1500 µs"]
    CLAMP["Clamp\n500 – 2500 µs"]
    CCR["Ghi TIM3_CCR\n(TIM3_CH2 hoặc CH3)"]
    SERVO["Servo Motor\n50 Hz PWM"]

    OFFSET --> SIGN --> CENTER --> CLAMP --> CCR --> SERVO
```

### Bảng ánh xạ góc → pulse width

| Góc | Pulse Width | CCR Value |
|---|---|---|
| -90° (cực tiểu) | 500 µs | 500 |
| 0° (trung tâm) | 1500 µs | 1500 |
| +90° (cực đại) | 2500 µs | 2500 |

> **Ghi chú:** Nếu servo quay ngược chiều, đặt `GIMBAL_SERVO_PITCH_SIGN = -1.0f` trong `gimbal_control.h`

---

## 9. Vòng lặp 500 Hz — Cơ chế Flag-based

### Tại sao không gọi thẳng từ TIM6 ISR?

```mermaid
sequenceDiagram
    participant TIM6 as TIM6 ISR
    participant I2C as I2C3 EV IRQ
    participant MAIN as Main Loop

    Note over TIM6,I2C: ❌ Cách SAI: gọi I2C từ ISR
    TIM6->>TIM6: Bắt đầu HAL_I2C_Mem_Read()
    TIM6->>I2C: Chờ I2C event (EV IRQ)
    Note over I2C: I2C EV có priority THẤP HƠN TIM6
    I2C--xTIM6: Không thể preempt TIM6!
    TIM6->>TIM6: TIMEOUT / DEADLOCK ❌

    Note over TIM6,MAIN: ✅ Cách ĐÚNG: flag-based
    TIM6->>MAIN: g_gimbal_tick_flag = 1 (< 1µs)
    MAIN->>MAIN: Kiểm tra flag
    MAIN->>I2C: HAL_I2C_Mem_Read() bình thường
    I2C-->>MAIN: OK (~400µs)
    MAIN->>MAIN: Chạy PID, ghi servo
```

### Flowchart cơ chế Flag

```mermaid
flowchart TD
    TIM6_ISR(["TIM6 ISR\n@ mỗi 2ms"])
    SET_FLAG["g_gimbal_tick_flag = 1\nThời gian: < 1µs"]
    TIM6_ISR --> SET_FLAG --> EXIT_ISR(["Thoát ISR"])

    MAIN_LOOP(["Main Loop\nwhile 1"])
    CHECK_MODE{Mode ==\nGIMBAL?}
    CHECK_FLAG{flag == 1?}
    CLEAR["flag = 0"]

    MAIN_LOOP --> CHECK_MODE
    CHECK_MODE -- No --> OTHER["Demo / Normal\nlogic"]
    CHECK_MODE -- Yes --> CHECK_FLAG
    CHECK_FLAG -- No --> UPDATE["Gimbal_Update\nIn telemetry 200ms"]
    CHECK_FLAG -- Yes --> CLEAR --> TICK["Gimbal_Tick()"]
    TICK --> UPDATE
    OTHER --> MAIN_LOOP
    UPDATE --> MAIN_LOOP
```

---

## 10. Luồng xử lý tổng thể

Đây là flowchart chi tiết của **hàm `Gimbal_Tick()`** — tim mạch của toàn bộ hệ thống:

```mermaid
flowchart TD
    START(["Gimbal_Tick() được gọi từ Main Loop"])

    INIT_CHK{initialized?}
    START --> INIT_CHK
    INIT_CHK -- No --> RET0(["return — chưa init"])

    MEAS_DT["Đo dt thực tế\ndt = HAL_GetTick - last_tick"]
    INIT_CHK -- Yes --> MEAS_DT

    DT_CHK{0 < dt < 10ms?}
    MEAS_DT --> DT_CHK
    DT_CHK -- No --> RET1(["return — dt bất thường\nbỏ tick này"])

    READ_IMU["ImuDual_Read()\nI2C3 tuần tự: Frame → Camera\n~400µs"]
    DT_CHK -- Yes --> READ_IMU

    I2C_OK{I2C OK?}
    READ_IMU --> I2C_OK
    I2C_OK -- FAIL --> RET2(["return — giữ servo cũ"])

    CF_UP["CompFilter_Update()\nFrame IMU → pitch_f, roll_f\nCamera IMU → pitch_c, roll_c\nCông thức: α×gyro_int + 1-α×accel_angle"]
    I2C_OK -- OK --> CF_UP

    REL_ERR["Tính Relative Error:\nerr_pitch = pitch_c - pitch_f - SP_pitch\nerr_yaw = gyro_z_cam - gyro_z_frame - SP_yaw"]
    CF_UP --> REL_ERR

    ANG_PID["Angle PID (Pitch outer loop):\nrate_sp = PID pitch_angle err_pitch"]
    REL_ERR --> ANG_PID

    RATE_PID["Rate PID (Pitch inner loop):\nrate_err = rate_sp - cam_gyro_x\noffset_p = PID pitch_rate rate_err"]
    ANG_PID --> RATE_PID

    FF_PITCH["Feedforward Pitch:\noffset_p += Kff_pitch × frame_gyro_x"]
    RATE_PID --> FF_PITCH

    YAW_PID["Rate PID (Yaw):\noffset_y = PID yaw_rate err_yaw\noffset_y += Kff_yaw × frame_gyro_z"]
    FF_PITCH --> YAW_PID

    SERVO_CALC["Tính Servo Pulse Width:\npitch_us = 1500 + SIGN × offset_p\nyaw_us   = 1500 + SIGN × offset_y\nClamp: 500 – 2500 µs"]
    YAW_PID --> SERVO_CALC

    WRITE_TIM["Ghi TIM3 CCR:\nTIM3_CH2 ← pitch_us\nTIM3_CH3 ← yaw_us"]
    SERVO_CALC --> WRITE_TIM

    TELEM["Copy Telemetry Struct\n(để main loop in log 200ms)"]
    WRITE_TIM --> TELEM

    DONE(["return — done! Tiếp tục main loop"])
    TELEM --> DONE
```

---

## 11. Bảng thông số mặc định

### Complementary Filter

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| α (alpha) | 0.98 | 98% Gyro + 2% Accel |
| Time constant | ~0.098s | Drift bù sau ~100ms |
| Sample rate | 500 Hz | dt = 2ms |
| DLPF | 42 Hz BW | Lọc rung cơ học > 42Hz |

### PID Gains (điểm khởi đầu, cần tuning thực tế)

| Controller | Kp | Ki | Kd | Output Range |
|---|---|---|---|---|
| Pitch Angle (outer) | 3.5 | 0.02 | 0.05 | ±120 °/s |
| Pitch Rate (inner) | 1.8 | 0.15 | 0.015 | ±800 µs |
| Yaw Rate | 1.5 | 0.08 | 0.01 | ±800 µs |

### Anti-windup & Derivative Filter

| Tham số | Giá trị |
|---|---|
| Integral limit (Angle PID) | ±40 |
| Integral limit (Rate PID) | ±80 |
| D-term EMA filter (Angle) | α = 0.15 |
| D-term EMA filter (Rate) | α = 0.12 |

---

## 12. Hướng dẫn Tuning PID

### Quy trình tuning từng bước

```mermaid
flowchart TD
    S1["Bước 1: Chỉ Kp\nKi=0, Kd=0, Kff=0\nTăng Kp từ 1.0"]
    S2{"Servo phản ứng\nchưa dao động?"}
    S3["Bước 2: Thêm Ki nhỏ\nKi = 0.01 → 0.05\nGiảm steady-state error"]
    S4{"Còn error khi\ngiữ tĩnh?"}
    S5["Bước 3: Thêm Kd\nKd = 0.005 → 0.02\nGiảm overshoot"]
    S6{"Servo mượt,\nít rung?"}
    S7["Bước 4: Thêm Feedforward\nKff = 0.0 → 0.3 → 0.6\nBù chuyển động frame nhanh"]
    S8{"Frame rung\nđược bù tốt?"}
    S9["Fine-tune Alpha\n0.99 = mượt, drift ít\n0.95 = nhanh, nhiều nhiễu"]
    DONE(["✅ Hoàn thành!"])

    S1 --> S2
    S2 -- No: Tăng thêm Kp --> S1
    S2 -- Yes --> S3
    S3 --> S4
    S4 -- Yes: Tăng Ki --> S3
    S4 -- No --> S5
    S5 --> S6
    S6 -- No: Giảm Kd --> S5
    S6 -- Yes --> S7
    S7 --> S8
    S8 -- No: Tăng Kff --> S7
    S8 -- Yes --> S9
    S9 --> DONE
```

### CLI Commands để tuning (qua USB CDC Terminal)

```
p Kp Ki Kd    → Pitch Angle PID   ví dụ: p 3.5 0.02 0.05
P Kp Ki Kd    → Pitch Rate PID    ví dụ: P 1.8 0.15 0.015
y Kp Ki Kd    → Yaw Rate PID      ví dụ: y 1.5 0.08 0.01
f kff_p kff_y → Feedforward       ví dụ: f 0.4 0.3
a alpha       → Filter alpha      ví dụ: a 0.97
s deg         → Pitch setpoint    ví dụ: s -5.0
r             → Reset PIDs (xóa integral)
d             → Dump thông số hiện tại
```

### Dấu hiệu nhận biết vấn đề

| Triệu chứng | Nguyên nhân | Cách sửa |
|---|---|---|
| Servo rung liên tục | Kp quá cao | Giảm Kp |
| Camera không giữ thẳng | Kp quá thấp hoặc Ki=0 | Tăng Kp, thêm Ki |
| Servo giật khi frame rung | Kff quá cao | Giảm Kff |
| Camera lắc sau khi bị chạm | Kd thiếu | Tăng Kd nhẹ |
| Góc bị drift theo thời gian | alpha quá cao | Giảm alpha (0.97–0.98) |

---

*Tài liệu được tạo tự động từ source code — Last updated: 2026-05-20 (v3.0)*
