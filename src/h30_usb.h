#pragma once
#include "h30_protocol.h"

// 不等待设备接入；初始化失败不影响其他业务。
/**
 * @brief 初始化H30 USB接收，不等待设备插入；失败不阻断其他业务。
 * @return true 初始化成功。
 */
bool H30UsbBegin();
// 返回 nullptr 表示成功；否则 INIT/OFFLINE/NO_DATA/STALE。
/**
 * @brief 读取最新原始姿态和数据年龄，不进行地图航向标定。
 * @param sample 输出H30原始姿态。
 * @param ageMs 输出数据年龄(ms)。
 * @return nullptr 成功，否则为初始化、离线、缺数据或过期原因。
 */
const char *H30UsbGetAttitude(H30Attitude &sample, uint32_t &ageMs);

// 仅在车体实际朝向被确认时调用。失败会清除旧参考，绝不延迟绑定未来样本。
/**
 * @brief 将当前有效原始Yaw绑定到已知车体地图航向；失败清除旧参考。
 * @param mapHeading 已确认的地图航向(deg)，0~360。
 * @return nullptr 成功，否则为不可标定原因。
 */
const char *H30UsbCalibrateHeading(float mapHeading);
// 地图航向，0~360 deg；与 currentPose 的理想航向独立，不驱动电机。
// 除原数据错误外，还可能返回 NOT_CALIBRATED。
/**
 * @brief 读取标定后的地图航向及数据年龄，不驱动电机；成功返回 nullptr。
 */
const char *H30UsbGetHeading(float &heading, uint32_t &ageMs);
// 标定并输出 REFERENCE 事件；调用时不持有 USB 临界区。
/**
 * @brief 建立地图航向参考并输出REFERENCE事件；heading为已确认的车体航向(deg)。
 */
void CalibrateH30MapHeading(float heading);
// 一致快照：采样时间与参考代次可用于闭环新鲜度及重标定检查。
/**
 * @brief 读取同一快照中的地图航向、采样毫秒数和参考代次，供闭环判断新鲜度及重标定。
 */
const char *H30UsbGetHeadingSnapshot(float &heading,uint32_t &sampleMs,uint32_t &referenceVersion);
// 同一帧的完整惯性数据与地图航向；停稳确认必须具备加速度和角速度。
/**
 * @brief 读取同一帧完整惯性数据、地图航向及参考代次；停稳检查需要加速度和角速度字段。
 */
const char *H30UsbGetMotionSnapshot(H30Attitude &sample,float &heading,uint32_t &referenceVersion);

/** @brief 收臂后静置3秒，再用3秒窗口采集静止Yaw并剔除离群值，绑定地图航向；不阻塞。 */
void H30UsbBeginHeadingAverage(float mapHeading);
/** @brief 推进开局标定：0采集中，1成功，-1无有效静止均值或数据不可用；不延长窗口。 */
int H30UsbPollHeadingAverage();
/** @brief 查询开局静置或采集窗口是否仍在运行，期间不执行开局斜移。 */
bool H30UsbHeadingAverageActive();
