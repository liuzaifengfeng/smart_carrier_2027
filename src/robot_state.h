// 比赛对外状态定义。
// 状态用于日志和上位机进度，内部每个状态可对应 competition_flow 中的多个子阶段。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

enum RobotState {  // 定义 RobotState（Robot状态） 的类型成员或作用域。
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
    STATE_DONE  // 续写当前声明或运算，提供 STATE_DONE。
};  // 结束此类型定义。

enum StartZone {  // 定义 StartZone（起点Zone） 的类型成员或作用域。
    START_ZONE_UNKNOWN = 0,  // 将 0, 写入或初始化 START_ZONE_UNKNOWN（起点ZONEUNKNOWN）。
    START_ZONE_1 = 1,  // 将 1, 写入或初始化 START_ZONE_1（起点ZONE1）。
    START_ZONE_2 = 2  // 将 2 写入或初始化 START_ZONE_2（起点ZONE2）。
};  // 结束此类型定义。

enum class AutoAlignmentState { IDLE, WAITING, DONE, FAILED };  // 定义 AutoAlignmentState（AutoAlignment状态） 的类型成员或作用域。
