#pragma once

enum RobotState {
    STATE_WAIT_START,    // 待机,等一键启动
    STATE_READ_TASK,     // 读取任务码(二维码板 / 机载电脑)
    STATE_SCAN_FAILED,   // 扫码重试耗尽，保持停车，不进入抓取流程
    STATE_ROUTE_FAILED,  // 路径失败，不进入后续用户代码
    STATE_ALIGN_FAILED,  // 自动对齐失败，停车等待处理
    STATE_TIMEOUT_FAILED, // 总任务超时，停止并等待人工处理
    STATE_TRANSFER_FAILED, // 搬运动作失败，保持停车等待处理
    STATE_GRAB_ROUND1,   // 第一批: 转盘抓取 3 个物料
    STATE_PLACE_COARSE1, // 第一批: 放到粗加工区
    STATE_PLACE_TEMP1,   // 第一批: 放到暂存区
    STATE_GRAB_ROUND2,   // 第二批: 转盘抓取
    STATE_PLACE_COARSE2, // 第二批: 放到粗加工区
    STATE_STACK_TEMP2,   // 第二批: 码垛到暂存区(叠在第一批上)
    STATE_RETURN_HOME,   // 回启停区,上报完成统计
    STATE_DONE
};

enum StartZone {
    START_ZONE_UNKNOWN = 0,
    START_ZONE_1 = 1,
    START_ZONE_2 = 2
};

enum class AutoAlignmentState { IDLE, WAITING, DONE, FAILED };
