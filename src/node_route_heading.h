// 节点线路的航向选择辅助逻辑。
// 路径的平移方向和车体朝向分别计算，查看终点行为时应核对调用方传入的目标航向。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。

constexpr float HOME_ZONE_HEADING = 270.0f; // 两个启停区返家时的地图航向。

// 输入航向已归一化到 [0,360)，只规划车身前后直行及原地转向。
/**
 * @brief 计算节点航向规划使用的绝对值。
 */
constexpr float NodeRouteAbs(float value) { return value < 0 ? -value : value; }  // 定义函数或成员 NodeRouteAbs（节点路线Abs），参数由此处的签名确定。
/**
 * @brief 计算from到to的最短有符号转角(deg)，输入航向已归一化到0~360。
 */
constexpr float NodeRouteTurn(float from, float to) {  // 定义函数或成员 的入口 NodeRouteTurn（节点路线Turn），参数由此处的签名确定。
    return to - from >= 180 ? to - from - 360  // 结束当前函数并返回 to - from >= 180 ? to - from - 360。
        : to - from < -180 ? to - from + 360 : to - from;  // 续写上一行表达式的条件取值分支。
}  // 结束此函数或控制块。
/**
 * @brief 计算指定行进方向对应的倒车车头航向(deg)。
 */
constexpr float NodeRouteBackward(float travel) { return travel >= 180 ? travel - 180 : travel + 180; }  // 定义函数或成员 NodeRouteBackward（节点路线Backward），参数由此处的签名确定。
// 最后一段先最小化终点转角；等角时选进入本段转动较少的方案，再等角时前进。
/**
 * @brief 选择本段是否倒车，末段优先减少终点转角，再比较起始转角。
 */
constexpr bool NodeRouteReverse(float current, float travel, bool finish, float finalHeading) {  // 定义函数或成员 的入口 NodeRouteReverse（节点路线Reverse），参数由此处的签名确定。
    return finish && NodeRouteAbs(NodeRouteTurn(travel, finalHeading)) + 0.01f  // 结束当前函数并返回 finish && NodeRouteAbs(NodeRouteTurn(travel, finalHead…。
            < NodeRouteAbs(NodeRouteTurn(NodeRouteBackward(travel), finalHeading)) ? false  // 定义函数或成员 NodeRouteAbs（节点路线Abs），参数由此处的签名确定。
        : finish && NodeRouteAbs(NodeRouteTurn(NodeRouteBackward(travel), finalHeading)) + 0.01f  // 定义函数或成员 NodeRouteAbs（节点路线Abs），参数由此处的签名确定。
            < NodeRouteAbs(NodeRouteTurn(travel, finalHeading)) ? true  // 定义函数或成员 NodeRouteAbs（节点路线Abs），参数由此处的签名确定。
        : NodeRouteAbs(NodeRouteTurn(current, NodeRouteBackward(travel)))  // 定义函数或成员 NodeRouteAbs（节点路线Abs），参数由此处的签名确定。
            < NodeRouteAbs(NodeRouteTurn(current, travel));  // 声明接口 NodeRouteAbs（节点路线Abs），参数由此处的签名确定。
}  // 结束此函数或控制块。
/**
 * @brief 按前进或倒车选择本段车头目标航向(deg)。
 */
constexpr float NodeRouteHeading(float current, float travel, bool finish, float finalHeading) {  // 定义函数或成员 的入口 NodeRouteHeading（节点路线航向），参数由此处的签名确定。
    return NodeRouteReverse(current, travel, finish, finalHeading) ? NodeRouteBackward(travel) : travel;  // 结束当前函数并返回 NodeRouteReverse(current, travel, finish, finalHeading…。
}  // 结束此函数或控制块。

struct NodeSegmentHeading {  // 定义 NodeSegmentHeading（节点Segment航向） 的类型成员或作用域。
    float heading;  // 声明 heading（航向），类型和可用范围由所在声明决定。
    float turn;  // 声明 turn，类型和可用范围由所在声明决定。
    float finishTurn;  // 声明 finishTurn，类型和可用范围由所在声明决定。
    bool reverse;  // 声明 reverse，类型和可用范围由所在声明决定。
};  // 结束此类型定义。

/**
 * @brief 规划本段车头航向、起始转角、终点转角及倒车标志；仅计算，不下发命令。
 */
constexpr NodeSegmentHeading PlanNodeSegmentHeading(float current, float travel, bool finish, float finalHeading) {  // 定义函数或成员 的入口 PlanNodeSegmentHeading（Plan节点Segment航向），参数由此处的签名确定。
    return {NodeRouteHeading(current, travel, finish, finalHeading),  // 结束当前函数并返回 {NodeRouteHeading(current, travel, finish, finalHeadin…。
            NodeRouteTurn(current, NodeRouteHeading(current, travel, finish, finalHeading)),  // 调用 NodeRouteTurn（节点路线Turn），传入 current, NodeRouteHeading(current, travel, finish, fin…。
            finish ? NodeRouteTurn(NodeRouteHeading(current, travel, finish, finalHeading), finalHeading) : 0.0f,  // 定义函数或成员 NodeRouteTurn（节点路线Turn），参数由此处的签名确定。
            NodeRouteReverse(current, travel, finish, finalHeading)};  // 调用 NodeRouteReverse（节点路线Reverse），传入 current, travel, finish, finalHeading)}。
}  // 结束此函数或控制块。
