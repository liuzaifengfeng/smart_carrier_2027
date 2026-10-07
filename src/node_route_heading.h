#pragma once

// 输入航向已归一化到 [0,360)，只规划车身前后直行及原地转向。
constexpr float NodeRouteAbs(float value) { return value < 0 ? -value : value; }
constexpr float NodeRouteTurn(float from, float to) {
    return to - from >= 180 ? to - from - 360
        : to - from < -180 ? to - from + 360 : to - from;
}
constexpr float NodeRouteBackward(float travel) { return travel >= 180 ? travel - 180 : travel + 180; }
// 最后一段先最小化终点转角；等角时选进入本段转动较少的方案，再等角时前进。
constexpr bool NodeRouteReverse(float current, float travel, bool finish, float finalHeading) {
    return finish && NodeRouteAbs(NodeRouteTurn(travel, finalHeading)) + 0.01f
            < NodeRouteAbs(NodeRouteTurn(NodeRouteBackward(travel), finalHeading)) ? false
        : finish && NodeRouteAbs(NodeRouteTurn(NodeRouteBackward(travel), finalHeading)) + 0.01f
            < NodeRouteAbs(NodeRouteTurn(travel, finalHeading)) ? true
        : NodeRouteAbs(NodeRouteTurn(current, NodeRouteBackward(travel)))
            < NodeRouteAbs(NodeRouteTurn(current, travel));
}
constexpr float NodeRouteHeading(float current, float travel, bool finish, float finalHeading) {
    return NodeRouteReverse(current, travel, finish, finalHeading) ? NodeRouteBackward(travel) : travel;
}

struct NodeSegmentHeading {
    float heading;
    float turn;
    float finishTurn;
    bool reverse;
};

constexpr NodeSegmentHeading PlanNodeSegmentHeading(float current, float travel,
                                                    bool finish, float finalHeading) {
    return {NodeRouteHeading(current, travel, finish, finalHeading),
            NodeRouteTurn(current, NodeRouteHeading(current, travel, finish, finalHeading)),
            finish ? NodeRouteTurn(NodeRouteHeading(current, travel, finish, finalHeading), finalHeading) : 0.0f,
            NodeRouteReverse(current, travel, finish, finalHeading)};
}
