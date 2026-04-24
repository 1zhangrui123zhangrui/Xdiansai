# Core/App — 应用层模块说明

> 所有不确定的硬件参数统一在 `platform_config.h` 中修改，不要直接改各模块源码中的数值。

---

## 模块一览

| 文件 | 职责 | 对外接口 |
|------|------|----------|
| `platform_config.h` | 所有宏参数 | — |
| `motor.h/.c` | ZDT X42S Emm 固件驱动 | `Motor_Init / Enable / Stop / MoveAbsolute / MultiPositionCmd` |
| `kinematics.h/.c` | 绳长↔坐标换算 | `Kinematics_Init / LaserToCam / CamToAngles` |
| `task.h/.c` | 任务状态机 | `Task_Init / Tick / StartHome / StartAreaPatrol ...` |
| `screen.h/.c` | 串口屏通信 | `Screen_Init / SetCoord / RecordFire / RegisterCallback` |
| `nrf_app.h/.c` | NRF 应用层 | `NrfApp_Init / Poll / IsFire / GetDxCm / GetDyCm` |
| `buzzer.h/.c` | 蜂鸣器 | `Buzzer_Init / Beep / BeepAsync / Tick` |
| `motor_test.h/.c` | 电机验证测试 | `MotorTest_Run` |

---

## platform_config.h — 关键参数

| 宏 | 含义 | 默认值 | 说明 |
|----|------|--------|------|
| `ROPE_H_CM` | 滑轮到平台竖直距离 (cm) | `0.0` | 滑轮与平台水平时为 0 |
| `SPOOL_RADIUS_CM` | 绕线轮半径 (cm) | `3.5` | 实测 35mm |
| `LASER_OFFSET_X_CM` | 激光笔距摄像头中心 X 偏移 (cm) | `3.5` | 向正 X 方向 |
| `MOTOR_SPEED_RPM` | 运动速度 (RPM) | `300` | 调低可更稳 |
| `MOTOR_ACCEL_LEVEL` | 加速档位 0-255 | `50` | 0=直接起速 |
| `MOTOR_MOVE_TIMEOUT_MS` | 运动超时 (ms) | `8000` | 按实际调整 |
| `MOTOR_ID_1~4` | 电机地址 | `1~4` | 需与电机拨码一致 |

---

## motor.h/.c — 电机驱动

### 固件说明
ZDT X42S V1.0 出厂默认 **Emm 固件**。

Emm FD 位置命令格式 (13 字节):
```
Addr  FD  dir  spd_h spd_l  acc  p3 p2 p1 p0  mode  sync  6B
```
- `dir`: `0x00`=CW(收线/正转)，`0x01`=CCW(放线/反转)
- `speed`: 0~3000 RPM，2 字节大端
- `acc`: 加速档位 0~255（0=直接起速，越大越快）
- `pulses`: 脉冲数，**3200 脉冲 = 1 圈**（1.8°步进 × 16 细分）
- `mode`: `0x01`=绝对零点，`0x02`=相对当前位置

### 主要接口
```c
// 初始化
Motor_Init(&huart1);

// 使能 / 停止
Motor_EnableAll();
Motor_StopAll();

// 标定: 平台放到中心后调用, 将所有电机角度清零
Motor_ZeroAllPositions();

// 绝对位置运动 (以标定点为零)
// angle_deg > 0 → CW 收线, < 0 → CCW 放线
Motor_MoveAbsolute(MOTOR_ID_1, 360.0f, 300.0f, 50, 0);

// 四电机多机原子命令
Motor_MultiPositionCmd(angles, 300.0f, 50);
```

---

## kinematics.h/.c — 运动学

将激光目标坐标转换为四电机所需的绝对旋转角度。

```
激光目标 (lx, ly)
      ↓ LaserToCam: cam_x = lx - LASER_OFFSET_X_CM
      ↓ CamToAngles: 绳长差分 → 角度
angles[4] → Motor_MultiPositionCmd
```

绳长公式 (H≈0 时退化为 2D):
```
L_i = sqrt( (cam_x - EA_i_x)^2 + (cam_y - EA_i_y)^2 )
angle_i (°) = (L_i_at_zero - L_i_at_target) / (2π×R) × 360
```

**必须先标定 (Motor_ZeroAllPositions) 再调用 Kinematics_Init()。**

---

## task.h/.c — 任务状态机

| 状态 | 触发函数 | 描述 |
|------|---------|------|
| `TASK_IDLE` | — | 空闲 |
| `TASK_HOME` | `Task_StartHome()` | 激光回中心圆 |
| `TASK_AREA_PATROL` | `Task_StartAreaPatrol()` | 四角圆依次巡逻 |
| `TASK_SEQ_PATROL` | `Task_StartSeqPatrol(seq)` | 用户指定 5 圆顺序 |
| `TASK_AUTO_PATROL` | `Task_StartAutoPatrol()` | 蛇形 + 火源检测 |
| `TASK_CALIBRATE` | `Task_StartCalibrate()` | 清零电机 + 更新运动学 |
| `TASK_E_STOP` | `Task_EStop()` | 立即停止 |

主循环调用 `Task_Tick()` 驱动状态机。

---

## nrf_app.h/.c — NRF 应用层

接收 F103 (平台摄像头) 发来的 8 字节数据包:

```c
typedef struct {
    uint8_t  flags;       // bit0=火源检测, bit1=位置有效
    uint8_t  seq;         // 序列号
    int16_t  dx_0p1mm;   // 摄像头偏离指令中心 X (0.1mm)
    int16_t  dy_0p1mm;   // Y (0.1mm)
    int8_t   fire_x_cm;  // 火源 X 坐标 (cm, 相对当前激光位置的偏差)
    int8_t   fire_y_cm;  // 火源 Y 坐标 (cm)
} NRF_CamData_t;
```

F103 端对接要点:
- 使用相同地址 `{0x34,0x43,0x10,0x10,0x01}`
- 载荷长度 8 字节
- 发现火源时置 `flags |= 0x01`, 填写 `fire_x_cm / fire_y_cm`

---

## buzzer.h/.c — 蜂鸣器

```c
Buzzer_Beep(3);          // 阻塞响 3 声
Buzzer_BeepAsync(3);     // 非阻塞, 需在主循环调用 Buzzer_Tick()
```

> ⚠️ 需要在 CubeMX IOC 中为 PB8 (或修改 `BUZZER_GPIO_PIN`) 新增 GPIO Output。

---

## motor_test.h/.c — 电机测试

验证 F407 → USART1 → 电机通路是否正常。

**开启方法**: 在 `main.c` 的 `USER CODE BEGIN PD` 处取消注释:
```c
#define MOTOR_TEST_ENABLE
```

**测试内容**: 电机 1 以 200 RPM 正转 1 圈，反转 1 圈，重复 3 次。

**验证完成后务必重新注释掉**，否则系统不会进入正常任务循环。
