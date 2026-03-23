# GO2 Driver Message

由LowState重发布，因为没有时间戳，高级没有关节信息了

go2_driver/msg/LegSensor.msg

```shell
# 时间戳 (纳秒，替代 Header 以去字符串化)
uint64 timestamp_ns

# 12个关节的位置 [FL_hip, FL_thigh, FL_calf, FR_hip, ..., RL..., RR...]
float32[12] q

# 12个关节的速度
float32[12] dq

# 12个关节的力矩
float32[12] tau

# 4个足端的接触力 (从 float32 改回与 LowState 一致的 int16) [FL, FR, RL, RR]，有效范围 -32768~32767
int16[4] foot_force

# --- 展开的 IMU 数据 (移除嵌套结构) ---
float32[4] imu_quaternion
float32[3] imu_gyroscope
float32[3] imu_accelerometer
```

```shell
ros2 interface show unitree_go/msg/LowState
uint8[2] head # 非标准 header  无法使用
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
IMUState imu_state
        float32[4] quaternion
        float32[3] gyroscope
        float32[3] accelerometer
        float32[3] rpy
        int8 temperature
MotorState[20] motor_state
        uint8 mode
        float32 q
        float32 dq
        float32 ddq
        float32 tau_est
        float32 q_raw
        float32 dq_raw
        float32 ddq_raw
        int8 temperature
        uint32 lost
        uint32[2] reserve
BmsState bms_state
        uint8 version_high
        uint8 version_low
        uint8 status
        uint8 soc
        int32 current
        uint16 cycle
        int8[2] bq_ntc
        int8[2] mcu_ntc
        uint16[15] cell_vol
int16[4] foot_force
int16[4] foot_force_est
uint32 tick # 非标准  无法使用
uint8[40] wireless_remote
uint8 bit_flag
float32 adc_reel
int8 temperature_ntc1
int8 temperature_ntc2
float32 power_v
float32 power_a
uint16[4] fan_frequency
uint32 reserve
uint32 crc
```
