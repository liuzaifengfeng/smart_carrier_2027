#include "disc_position_grab.h"
#include "motion_executor.h"
int armSpeed=500;
namespace {
bool grab(uint8_t position,uint8_t color,uint8_t cargo) {
    MotionRequest r;r.kind=MotionKind::DISC_GRAB;r.position=position;r.color=color;r.cargo=cargo;
    return WaitMotion(SubmitMotion(r));
}
}
/**
 * @brief 同步提交并等待圆盘位置1的夹取、抬升及载物台放置动作。
 * @param color 物料颜色编号。
 * @param cargo 目标载物台编号1~3。
 * @return true 作业完成并登记，不代表传感器确认实物。
 */
bool GrabDiscPosition1(uint8_t color,uint8_t cargo){return grab(1,color,cargo);}
/**
 * @brief 同步提交并等待圆盘位置2的夹取、抬升及载物台放置动作；color为颜色，cargo为载物台1~3。
 */
bool GrabDiscPosition2(uint8_t color,uint8_t cargo){return grab(2,color,cargo);}
/**
 * @brief 同步提交并等待圆盘位置3的夹取、抬升及载物台放置动作；color为颜色，cargo为载物台1~3。
 */
bool GrabDiscPosition3(uint8_t color,uint8_t cargo){return grab(3,color,cargo);}
