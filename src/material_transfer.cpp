#include "material_transfer.h"
#include "turret_path.h"
#include "motion_executor.h"

#include <math.h>

// ================= 实车标定区（主要修改这里） =================
// 每一项的填写顺序都是：{升降高度, 伸出长度, 转台角度}。
// 固定位置由参数配置提供，调用搬运动作前应完成实车标定。
MaterialTransferLayout materialTransferLayout = {
    {150, 50, 90}, // 圆盘取料位置
    {
        {102, 20, 242}, // 1 号载物台
        {102, 20, 271}, // 2 号载物台
        {102, 21, 300}, // 3 号载物台
    },
    {
        {2.5, 61, 48}, // 粗加工区/暂存区的 1 号位置
        {2.5, 0, 86}, // 粗加工区/暂存区的 2 号位置
        {2.5, 39, 125.3}, // 粗加工区/暂存区的 3 号位置
    },
    170.0f, // approachHeight，按物料实际高度修改
    60.0f,  // secondLayerOffset，按物料实际高度修改
    250.0f, // moveSpeed
    50.0f,   // clawOpenAngle
    -5.0f,  // clawClosedAngle
    1,      // motionWaitMs，不传等待参数时的默认毫秒数
};

MaterialMotionSteps materialMotionSteps = {
    {80.0f, 0.0f, 90.0f}, // 圆盘预抓：高度、伸出、转台
    90.0f,  // 圆盘预抓张爪角度
    150.0f, // 圆盘抓后提起高度
    250.0f, 250.0f, 250.0f, // 圆盘预抓、夹紧、提起
    250.0f, 250.0f,         // 移至工位上方：平移、转向
    250.0f, 250.0f, 250.0f, // 取料：下降、夹紧、提起
    250.0f, 250.0f, 250.0f, // 放料：下降、松爪、提起
};

// ACTION_GROUP_DELAYS_BEGIN
// 上位机只改写本区域的延时数值；单位 ms，0 表示不等待。
// 每行顺序：取料平移/稳定/转向，下降/夹紧/抬升，放料平移/稳定/转向，下降/松爪/抬升。
MaterialTransferDelays commonMaterialDelays =
    {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300};
MaterialTransferDelays cargoToWorkAreaDelays[MATERIAL_STATION_COUNT][MATERIAL_STATION_COUNT] = {
    {
        {{100, 0, 800}, 600, 500, 400, {300, 0, 800}, 1100, 300, 700}, // 1 -> 1
        {{100, 0, 800}, 600, 500, 400, {200, 0, 600}, 1100, 300, 700}, // 1 -> 2
        {{100, 0, 800}, 600, 500, 400, {300, 0, 200}, 1200, 300, 700}, // 1 -> 3
    },
    {
        {{100, 0, 800}, 600, 500, 400, {300, 0, 800}, 1100, 300, 700}, // 2 -> 1
        {{100, 0, 800}, 600, 500, 400, {200, 0, 900}, 1100, 300, 700}, // 2 -> 2
        {{100, 0, 800}, 600, 500, 400, {300, 0, 800}, 800, 300, 700}, // 2 -> 3
    },
    {
        {{100, 0, 800}, 600, 500, 400, {300, 0, 600}, 800, 300, 700}, // 3 -> 1
        {{100, 0, 800}, 600, 500, 400, {300, 0, 700}, 800, 300, 700}, // 3 -> 2
        {{100, 0, 800}, 600, 500, 300, {200, 0, 900}, 800, 300, 700}, // 3 -> 3
    },
};
MaterialAuxiliaryDelays materialAuxiliaryDelays = {300, 340, 300, 1360, 680, 1360, 680};
// ACTION_GROUP_DELAYS_END

// REVERSE_ACTION_GROUP_DELAYS_BEGIN
// 工作区 -> 载物台，按 [工作区编号-1][载物台编号-1] 选择；单位 ms。
// 每行与正向相同，共12项；取料抬升表示同时抬升、缩回到0之后的等待。
// 九组初值复制当前公共默认值，之后各自独立调整、保存。
MaterialTransferDelays workAreaToCargoDelays[MATERIAL_STATION_COUNT][MATERIAL_STATION_COUNT] = {
    {
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区1 -> 载物台1
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区1 -> 载物台2
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区1 -> 载物台3
    },
    {
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区2 -> 载物台1
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区2 -> 载物台2
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区2 -> 载物台3
    },
    {
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区3 -> 载物台1
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区3 -> 载物台2
        {{1000, 0, 700}, 700, 700, 700, {700, 0, 1000}, 700, 300, 300}, // 工作区3 -> 载物台3
    },
};
// REVERSE_ACTION_GROUP_DELAYS_END

// 开机时三个载物台均为空载。
CargoPlatform cargoPlatforms[MATERIAL_STATION_COUNT] = {
    {MATERIAL_NONE}, {MATERIAL_NONE}, {MATERIAL_NONE}
};

namespace {
bool submit(const MotionRequest &request){return WaitMotion(SubmitMotion(request));}
MotionRequest transfer(MotionKind kind,uint8_t cargo,uint8_t work,uint8_t color=0) {
    MotionRequest request;request.kind=kind;request.cargo=cargo;request.work=work;request.color=color;return request;
}
MotionRequest round(MotionKind kind,const int *colors,const int *positions,uint8_t layer=1) {
    MotionRequest request;request.kind=kind;request.layer=layer;
    for(uint8_t i=0;i<3;++i){if(colors)request.colors[i]=colors[i];if(positions)request.positions[i]=positions[i];}
    return request;
}
}
bool MoveDiscToCargo(uint8_t color,uint8_t cargo) {
    auto r=transfer(MotionKind::DISC_GRAB,cargo,0,color);r.position=4;return submit(r);
}
bool MoveCargoToWorkArea(uint8_t cargo,uint8_t work){return submit(transfer(MotionKind::TRANSFER,cargo,work));}
bool MoveWorkAreaToCargo(uint8_t color,uint8_t work,uint8_t cargo){return submit(transfer(MotionKind::RETRIEVE,cargo,work,color));}
bool StackCargoToWorkArea(uint8_t cargo,uint8_t work){auto r=transfer(MotionKind::TRANSFER,cargo,work);r.layer=2;return submit(r);}
bool PlaceTaskCargoToWorkArea(const int positions[3],uint8_t layer){return submit(round(MotionKind::PLACE_ROUND,nullptr,positions,layer));}
bool RetrieveRoundToCargo(const int colors[3],const int positions[3]){return submit(round(MotionKind::RETRIEVE_ROUND,colors,positions));}
bool LoadRoundFromDisc(const int colors[3]){return submit(round(MotionKind::LOAD_ROUND,colors,nullptr));}
bool PlaceRoundToWorkArea(const int positions[3]){return PlaceTaskCargoToWorkArea(positions,1);}
bool StackRoundToWorkArea(const int positions[3]){return PlaceTaskCargoToWorkArea(positions,2);}
bool GrabDiscMaterial(uint8_t color,uint8_t cargo,bool force){auto r=transfer(MotionKind::DISC_GRAB,cargo,0,color);r.force=force;return submit(r);}
bool ValidateDiscPositionTransfer(uint8_t,uint8_t cargo){return cargoPlatforms[cargo-1].material==MATERIAL_NONE;}
bool PlaceHeldDiscMaterialToCargo(uint8_t color,uint8_t cargo){return submit(transfer(MotionKind::HELD_DISC,cargo,0,color));}
bool PrepareLidarScanPose(uint8_t zone){MotionRequest r;r.kind=MotionKind::LIDAR_POSE;r.zone=zone;return submit(r);}
bool DemoCargoToRoughArea(){MotionRequest r;r.kind=MotionKind::DEMO_PLACE;return submit(r);}
bool DemoWorkAreaToCargo(){MotionRequest r;r.kind=MotionKind::DEMO_RETRIEVE;return submit(r);}
bool DemoStackCargoToWorkArea(){MotionRequest r;r.kind=MotionKind::DEMO_PLACE;r.layer=2;return submit(r);}
bool DemoStackCargoToWorkArea3(){MotionRequest r;r.kind=MotionKind::DEMO_PLACE;r.layer=3;return submit(r);}
