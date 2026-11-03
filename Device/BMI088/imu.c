#include "imu.h"
#include <math.h>

float imu_angle[3];

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float last_yaw_deg;
static bool attitude_initialized;
static const float rad_to_deg = 57.2957795131f;
static const float correction_gain = 2.0f;

static bool normalize_sample(const float gyro[3], const float accel[3],
                             float dt_s, float unit_accel[3]);
static void initialize_attitude(const float accel[3]);
static void integrate_attitude(const float gyro[3], const float accel[3], float dt_s);
static void update_angles(void);

/**
************************************************************************
* @brief:      imu_data_process - 使用一组 BMI088 测量值更新姿态角
* @param:      gyro - 三轴角速度，单位 rad/s，按机身 X、Y、Z 排列
* @param:      accel - 同一机身坐标系的三轴加速度，单位 m/s²
* @param:      dt_s - 相邻有效采样的实际时间间隔，单位秒
* @retval:     true - 已更新 imu_angle；false - 输入无效，保留上次结果
* @details:    首次静止采样初始化横滚和俯仰，偏航从零开始；后续采样
*              通过四元数积分和重力方向校正更新姿态。每组采样调用一次。
*              本函数接收驱动已经换算好的数值，不负责 SPI 读取或零偏标定。
************************************************************************
**/
bool imu_data_process(const float gyro[3], const float accel[3], float dt_s)
{
    float unit_accel[3];

    if (!normalize_sample(gyro, accel, dt_s, unit_accel))
        return false;

    if (!attitude_initialized)
    {
        initialize_attitude(unit_accel);
        attitude_initialized = true;
    }
    else
    {
        integrate_attitude(gyro, unit_accel, dt_s);
    }

    update_angles();
    return true;
}

/**
************************************************************************
* @brief:      get_imu_angle - 获取指定轴的最新姿态角
* @param:      axis - Roll、Pitch 或 Yaw，调用方必须传入有效枚举值
* @retval:     对应姿态角，单位度；Yaw 为连续相对角，不是绝对航向
* @details:    读取 imu_angle 缓存，不读取传感器、不推进解算。
*              应在 imu_data_process 首次返回 true 后使用。
************************************************************************
**/
float get_imu_angle(IMU axis)
{
    return imu_angle[axis];
}

/**
************************************************************************
* @brief:      Euler_Update - 将最新解算结果发布到控制层 Euler 结构体
* @param:      euler - 要更新的姿态结构体指针，Update_Yaw 保存软件航向零点
* @retval:     void
* @details:    Roll、Pitch 直接采用机身轴定义；Yaw 扣除 Update_Yaw。
*              Last_Yaw、Last_Pitch 保存本次调用前的结构体值，因此其含义是
*              上次发布值，不一定是上次采样值。不会重新积分或改写 Target_Yaw。
************************************************************************
**/
void Euler_Update(Euler *euler)
{
    euler->Last_Yaw = euler->Yaw;
    euler->Last_Pitch = euler->Pitch;
    euler->Roll = imu_angle[Roll];
    euler->Pitch = imu_angle[Pitch];
    euler->Yaw = imu_angle[Yaw] - euler->Update_Yaw;
}

/**
************************************************************************
* @brief:      Yaw_Reset - 将当前朝向设置为控制层的偏航零点
* @param:      euler - 要设置航向零点的姿态结构体指针
* @retval:     void
* @details:    记录当前连续偏航角到 Update_Yaw，并清零 Yaw、Last_Yaw。
*              不改变内部四元数、imu_angle 或 Target_Yaw，也不执行传感器标定。
************************************************************************
**/
void Yaw_Reset(Euler *euler)
{
    euler->Update_Yaw = imu_angle[Yaw];
    euler->Yaw = 0.0f;
    euler->Last_Yaw = 0.0f;
}

/**
************************************************************************
* @brief:      normalize_sample - 统一检查输入采样并提取加速度方向
* @param:      gyro - 三轴角速度输入，单位 rad/s
* @param:      accel - 三轴加速度输入，单位 m/s²
* @param:      dt_s - 采样时间间隔，单位秒，必须大于零
* @param:      unit_accel - 输出归一化加速度方向，三个元素，无量纲
* @retval:     true - 输入有效；false - 非有限值、零加速度或时间间隔无效
* @details:    外部采样只在此处校验；成功后内部数学函数直接使用已验证数据。
*              仅归一化方向，不能区分重力和机身运动产生的加速度。
************************************************************************
**/
static bool normalize_sample(const float gyro[3], const float accel[3],
                             float dt_s, float unit_accel[3])
{
    unsigned i;
    float norm;

    if (!isfinite(dt_s) || dt_s <= 0.0f)
        return false;

    for (i = 0; i < 3; ++i)
    {
        if (!isfinite(gyro[i]) || !isfinite(accel[i]))
            return false;
    }

    norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (!isfinite(norm) || norm == 0.0f)
        return false;

    for (i = 0; i < 3; ++i)
        unit_accel[i] = accel[i] / norm;

    return true;
}

/**
************************************************************************
* @brief:      initialize_attitude - 由静止加速度方向建立初始姿态
* @param:      accel - 已校验的单位加速度向量，按机身 X、Y、Z 排列
* @retval:     void
* @details:    采用 X 向前、Y 向左、Z 向上的右手坐标系，静止水平时
*              加速度沿 +Z。由重力参考求横滚和俯仰，初始偏航设为零，
*              生成从机身坐标到参考坐标的四元数 q，顺序为 w、x、y、z。
************************************************************************
**/
static void initialize_attitude(const float accel[3])
{
    float roll = atan2f(accel[1], accel[2]);
    float pitch = atan2f(-accel[0],
                         sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
    float cr = cosf(0.5f * roll);
    float sr = sinf(0.5f * roll);
    float cp = cosf(0.5f * pitch);
    float sp = sinf(0.5f * pitch);

    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

/**
************************************************************************
* @brief:      integrate_attitude - 融合角速度和重力参考并更新四元数
* @param:      gyro - 已校验的机身角速度，单位 rad/s
* @param:      accel - 同一坐标系内的单位加速度向量，无量纲
* @param:      dt_s - 本次积分时间间隔，单位秒
* @retval:     void
* @details:    加速度方向误差按 correction_gain 比例反馈到角速度，再进行
*              四元数积分和归一化。要求初始姿态已建立；明显的运动加速度
*              会影响重力校正。本函数不包含自动零偏估计或绝对航向校正。
************************************************************************
**/
static void integrate_attitude(const float gyro[3], const float accel[3], float dt_s)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float vx = 2.0f * (x * z - w * y);
    float vy = 2.0f * (w * x + y * z);
    float vz = w * w - x * x - y * y + z * z;
    float gx = gyro[0] + correction_gain * (accel[1] * vz - accel[2] * vy);
    float gy = gyro[1] + correction_gain * (accel[2] * vx - accel[0] * vz);
    float gz = gyro[2] + correction_gain * (accel[0] * vy - accel[1] * vx);
    float half_dt = 0.5f * dt_s;
    float norm;
    unsigned i;

    q[0] = w + (-x * gx - y * gy - z * gz) * half_dt;
    q[1] = x + ( w * gx + y * gz - z * gy) * half_dt;
    q[2] = y + ( w * gy - x * gz + z * gx) * half_dt;
    q[3] = z + ( w * gz + x * gy - y * gx) * half_dt;

    norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (i = 0; i < 4; ++i)
        q[i] /= norm;
}

/**
************************************************************************
* @brief:      update_angles - 将当前四元数转换为角度并展开连续偏航
* @param:      void
* @retval:     void，结果写入 imu_angle[Roll/Pitch/Yaw]
* @details:    按 ZYX 约定输出欧拉角，单位度；Pitch 范围为 [-90, 90]，
*              Roll 为 atan2 主值。偏航跨越 ±180° 时累加最短角差，要求
*              相邻有效采样的偏航变化小于 180°。连续角处理不能消除漂移。
************************************************************************
**/
static void update_angles(void)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float sin_pitch = 2.0f * (w * y - z * x);
    float yaw_deg = atan2f(2.0f * (w * z + x * y),
                           1.0f - 2.0f * (y * y + z * z)) * rad_to_deg;
    float yaw_delta = yaw_deg - last_yaw_deg;

    /* 浮点舍入可能让正弦值略超出 asin 的定义域。 */
    sin_pitch = fminf(1.0f, fmaxf(-1.0f, sin_pitch));
    imu_angle[Roll] = atan2f(2.0f * (w * x + y * z),
                            1.0f - 2.0f * (x * x + y * y)) * rad_to_deg;
    imu_angle[Pitch] = asinf(sin_pitch) * rad_to_deg;

    if (yaw_delta > 180.0f)
        yaw_delta -= 360.0f;
    else if (yaw_delta < -180.0f)
        yaw_delta += 360.0f;

    imu_angle[Yaw] += yaw_delta;
    last_yaw_deg = yaw_deg;
}
