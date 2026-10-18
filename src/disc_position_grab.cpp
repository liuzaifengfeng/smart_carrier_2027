#include "disc_position_grab.h"
#include "chassis.h"
#include "material_transfer.h"
#include <Arduino.h>

//圆盘上三个位置从左到右是1、2、3

//机械臂速度
int armSpeed = 300;

bool GrabDiscPosition1(uint8_t color, uint8_t cargo) {
    
    if (!ValidateDiscPositionTransfer(color, cargo)) return false;
    if (!MoveArm(120, 150, 71, 80, armSpeed)) return false;
    delay(300);
    if (!MoveArm(80, 150, 71, 80, armSpeed)) return false;
    delay(400);
    if (!MoveArm(80, 150, 71, -8, armSpeed)) return false;
    delay(300);
    if (!MoveArm(170, 150, 71, -8, armSpeed)) return false;
    delay(300);
    return PlaceHeldDiscMaterialToCargo(color, cargo);
}

bool GrabDiscPosition2(uint8_t color, uint8_t cargo) {
    if (!ValidateDiscPositionTransfer(color, cargo)) return false;
    // 从物料观察姿态开始：先在安全高度收回并张爪，再垂直下探。
    if (!MoveArm(120, 0, 90, 80, armSpeed)) return false;
    delay(300);
    if (!MoveArm(80, 0, 90, 80, armSpeed)) return false;
    delay(400);
    // 实车确认的抓到物料瞬间，保持该动作不变。
    if (!MoveArm(80, 0, 90, -8, armSpeed)) return false;
    delay(300);
    // 保持夹紧并垂直抬升到安全高度。
    if (!MoveArm(170, 0, 90, -8, armSpeed)) return false;
    delay(300);
    return PlaceHeldDiscMaterialToCargo(color, cargo);
}

bool GrabDiscPosition3(uint8_t color, uint8_t cargo) {
    if (!ValidateDiscPositionTransfer(color, cargo)) return false;
    if (!MoveArm(120, 150, 102, 80, armSpeed)) return false;
    delay(300);
    if (!MoveArm(80, 150, 102, 80, armSpeed)) return false;
    delay(400);
    if (!MoveArm(80, 150, 102, -8, armSpeed)) return false;
    delay(300);
    if (!MoveArm(170, 150, 102, -8, armSpeed)) return false;
    delay(300);
    return PlaceHeldDiscMaterialToCargo(color, cargo);
}
