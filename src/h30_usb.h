#pragma once
#include "h30_protocol.h"

// 不等待设备接入；初始化失败不影响其他业务。
bool H30UsbBegin();
// 返回 nullptr 表示成功；否则 INIT/OFFLINE/NO_DATA/STALE。
const char *H30UsbGetAttitude(H30Attitude &sample, uint32_t &ageMs);

// 仅在车体实际朝向被确认时调用。失败会清除旧参考，绝不延迟绑定未来样本。
const char *H30UsbCalibrateHeading(float mapHeading);
// 地图航向，0~360 deg；与 currentPose 的理想航向独立，不驱动电机。
// 除原数据错误外，还可能返回 NOT_CALIBRATED。
const char *H30UsbGetHeading(float &heading, uint32_t &ageMs);
// 标定并输出 REFERENCE 事件；调用时不持有 USB 临界区。
void CalibrateH30MapHeading(float heading);
// 一致快照：采样时间与参考代次可用于闭环新鲜度及重标定检查。
const char *H30UsbGetHeadingSnapshot(float &heading,uint32_t &sampleMs,uint32_t &referenceVersion);
