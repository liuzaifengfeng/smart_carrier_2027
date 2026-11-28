// 圆盘三个抓取入口共用同一个抓放过程，并分别传入物理位置编号。
// 颜色和货位随调用传递，动作实现见物料转运模块。

#include "disc_position_grab.h"  // 引入 disc_position_grab.h 中的声明和定义。
#include "motion_executor.h"  // 引入 motion_executor.h 中的声明和定义。
int armSpeed=500;  // 将 500 写入或初始化 armSpeed（机械臂速度）。
namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
bool grab(uint8_t position,uint8_t color,uint8_t cargo) {  // 定义函数或成员 的入口 grab，参数由此处的签名确定。
    MotionRequest r;r.kind=MotionKind::DISC_GRAB;r.position=position;r.color=color;r.cargo=cargo;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::DISC_GRAB 写入或初始化 r.kind。然后将 position 写入或初始化 r.position。然后将 color 写入或初始化 r.color。然后将 cargo 写入或初始化 r.cargo。
    return WaitMotion(SubmitMotion(r));  // 结束当前函数并返回 WaitMotion(SubmitMotion(r))。
}  // 结束此函数或控制块。
}  // 结束命名空间。
/**
 * @brief 同步提交并等待圆盘位置1的夹取、抬升及载物台放置动作。
 * @param color 物料颜色编号。
 * @param cargo 目标载物台编号1~3。
 * @return true 作业完成并登记，不代表传感器确认实物。
 */
bool GrabDiscPosition1(uint8_t color,uint8_t cargo){return grab(1,color,cargo);}  // 定义函数或成员 GrabDiscPosition1（Grab圆盘位置1），参数由此处的签名确定。
/**
 * @brief 同步提交并等待圆盘位置2的夹取、抬升及载物台放置动作；color为颜色，cargo为载物台1~3。
 */
bool GrabDiscPosition2(uint8_t color,uint8_t cargo){return grab(2,color,cargo);}  // 定义函数或成员 GrabDiscPosition2（Grab圆盘位置2），参数由此处的签名确定。
/**
 * @brief 同步提交并等待圆盘位置3的夹取、抬升及载物台放置动作；color为颜色，cargo为载物台1~3。
 */
bool GrabDiscPosition3(uint8_t color,uint8_t cargo){return grab(3,color,cargo);}  // 定义函数或成员 GrabDiscPosition3（Grab圆盘位置3），参数由此处的签名确定。
