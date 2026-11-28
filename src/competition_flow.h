// 比赛阶段编排。
// Phase 保存内部细分阶段，RobotState 对外报告进度。
// 流程通过接口提交运动并轮询结果，物理时序由运动执行层管理。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include "robot_state.h"  // 引入 robot_state.h 中的声明和定义。
#include "motion_job.h"  // 引入 motion_job.h 中的声明和定义。
#include "vision_alignment.h"  // 引入 vision_alignment.h 中的声明和定义。
#include "arm_motion.h"  // 引入 arm_motion.h 中的声明和定义。
#include "disc_material_decision.h"  // 引入 disc_material_decision.h 中的声明和定义。
#include "node_route_heading.h"  // 引入 node_route_heading.h 中的声明和定义。
#include <string.h>  // 引入 string.h 中的声明和定义。

struct CompetitionMap {  // 定义 CompetitionMap 的类型成员或作用域。
    uint8_t scan=2,disc=14,coarse=10,temp=22,home1=4,home2=0;  // 将 2,disc=14,coarse=10,temp=22,home1=4,home2=0 写入或初始化 scan（扫码）。
    float homeHeading=HOME_ZONE_HEADING;  // 将 HOME_ZONE_HEADING 写入或初始化 homeHeading（home航向）。
};  // 结束此类型定义。
struct CompetitionIO {  // 定义 CompetitionIO（Competition设备接口） 的类型成员或作用域。
    virtual ~CompetitionIO()=default;  // 声明接口 CompetitionIO（Competition设备接口），参数由此处的签名确定。
    /**
     * @brief 读取主流程使用的设备运行毫秒数，供阶段间隔计时。
     */
    virtual uint32_t now()=0;  // 声明接口 now，参数由此处的签名确定。
    /**
     * @brief 读取或更新对外 RobotState；读接口返回当前状态，写接口供阶段切换使用。
     */
    virtual RobotState state()=0;  // 声明接口 state（状态），参数由此处的签名确定。
    /**
     * @brief 读取或更新对外 RobotState；读接口返回当前状态，写接口供阶段切换使用。
     */
    virtual void state(RobotState value)=0;  // 声明接口 state（状态），参数由此处的签名确定。
    /**
     * @brief 查询整场任务是否已取消，取消后主流程不再提交动作。
     */
    virtual bool aborted()=0;  // 声明接口 aborted，参数由此处的签名确定。
    /**
     * @brief 读取当前选择的起始区号；主流程在启动准备时锁定返家区号。
     */
    virtual uint8_t zone()=0;  // 声明接口 zone，参数由此处的签名确定。
    /** @brief 开始机械臂初始化后的静置500ms及采集3秒航向标定，不阻塞主流程。 */
    virtual void beginHeadingCalibration()=0;  // 声明接口 beginHeadingCalibration（begin航向Calibration），参数由此处的签名确定。
    /** @brief 查询标定窗口是否结束；成功时由运行层闪灯两次。 */
    virtual bool headingCalibrationDone()=0;  // 声明接口 headingCalibrationDone（航向Calibration完成），参数由此处的签名确定。
    /**
     * @brief 查询正式比赛启动许可，许可到达后开始计时并进入扫码。
     */
    virtual bool permission()=0;  // 声明接口 permission，参数由此处的签名确定。
    /**
     * @brief 读取当前轮已经完成抓取并登记到载物台的物料件数。
     */
    virtual int progress()=0;  // 声明接口 progress，参数由此处的签名确定。
    /**
     * @brief 将当前轮圆盘抓取进度清零，供进入下一轮或阶段时使用。
     */
    virtual void clearProgress()=0;  // 声明接口 clearProgress，参数由此处的签名确定。
    /**
     * @brief 将指定轮次的三件颜色和工位编号填入运动请求。
     * @param request 接收本轮任务数据的动作请求。
     * @param secondRound true 第二轮，false 第一轮。
     */
    virtual void task(MotionRequest &request,bool secondRound)=0;  // 声明接口 task（任务），参数由此处的签名确定。
    /**
     * @brief 提交一个非阻塞运动请求；返回作业编号，后续用 status 查询完成结果。
     */
    virtual uint32_t submit(const MotionRequest &request)=0;  // 声明接口 submit，参数由此处的签名确定。
    virtual uint32_t submit(const MotionRequest &request,MotionSubmitError &error) {  // 定义函数或成员 的入口 submit，参数由此处的签名确定。
        const auto id=submit(request);  // 将 submit(request) 写入或初始化 id（编号）。
        error=id?MotionSubmitError::NONE:MotionSubmitError::UNAVAILABLE;  // 将 id?MotionSubmitError::NONE:MotionSubmitError::UNAVAILA… 写入或初始化 error（错误）。
        return id;  // 结束当前函数并返回 id。
    }  // 结束此函数或控制块。
    virtual bool motionBusy(){return false;}  // 定义 motionBusy，调用时返回 false。
    /**
     * @brief 提交起始区扫码准备姿态作业；返回作业编号，不在主流程等待物理动作。
     */
    virtual uint32_t lidar(uint8_t startZone)=0;  // 声明接口 lidar，参数由此处的签名确定。
    /**
     * @brief 读取指定作业的执行快照；只有终态 DONE 才表示该作业按当前完成策略结束。
     */
    virtual MotionStatus status(uint32_t id)=0;  // 声明接口 status（状态），参数由此处的签名确定。
    /**
     * @brief 请求开启指定视觉功能；工位对齐与单件物料识别使用不同模式。
     */
    virtual void visionStart(VisionStartMode mode,bool invertCornerXY=false)=0;  // 声明接口 visionStart（视觉起点），参数由此处的签名确定。
    /**
     * @brief 读取当前自动视觉对齐的 WAITING、DONE 或 FAILED 状态。
     */
    virtual AutoAlignmentState visionStatus()=0;  // 声明接口 visionStatus（视觉状态），参数由此处的签名确定。
    /**
     * @brief 请求停止指定视觉功能，结束该阶段的视觉控制。
     */
    virtual void visionStop(VisionStartMode mode)=0;  // 声明接口 visionStop（视觉停止），参数由此处的签名确定。
    /**
     * @brief 开启圆盘单件物料识别；识别结果触发独立的抓取作业。
     */
    virtual void materialStart()=0;  // 声明接口 materialStart（物料起点），参数由此处的签名确定。
    /**
     * @brief 停止圆盘单件物料识别并清除对应就绪标志。
     */
    virtual void materialStop()=0;  // 声明接口 materialStop（物料停止），参数由此处的签名确定。
    /**
     * @brief 发布已解析的任务码显示，并设置扫码结束后的位姿准备标志。
     */
    virtual void taskReady()=0;  // 声明接口 taskReady（任务就绪），参数由此处的签名确定。
    /**
     * @brief 记录正式开始时间、输出 RUN_START，并启动比赛总计时器。
     */
    virtual void runStart()=0;  // 声明接口 runStart（run起点），参数由此处的签名确定。
    /**
     * @brief 输出正式完成结果、停止比赛计时，并提交电机失能作业。
     */
    virtual void runDone()=0;  // 声明接口 runDone（run完成），参数由此处的签名确定。
    /**
     * @brief 转入指定业务失败状态，取消运动并更新显示；沿用现有失败处理。
     */
    virtual void fail(RobotState reason)=0;  // 声明接口 fail，参数由此处的签名确定。
    /**
     * @brief 向上位机发布当前业务阶段的显示文本。
     */
    virtual void display(const char *text)=0;  // 声明接口 display，参数由此处的签名确定。
};  // 结束此类型定义。
// 业务流程只负责提交动作、等待结果和切换阶段；电机执行仍由 MotionExecutor 负责。
// 对外 RobotState 保持原编号和日志名称；下面的 Phase 把每个大状态展开为可直接阅读的小阶段。
class CompetitionFlow {  // 定义 CompetitionFlow 的类型成员或作用域。
    enum class Phase {  // 定义 Phase（阶段） 的类型成员或作用域。
        START_PREPARE_ARM, START_CALIBRATE_HEADING, START_WAIT_ZONE, START_PREPARE_LIDAR, START_WAIT_PERMISSION,  // 声明 START_WAIT_PERMISSION（起点WAITPERMISSION），类型和可用范围由所在声明决定。
        SCAN_TASK,  // 为当前列表或跨行表达式提供这一项 SCAN_TASK。
        DISC_NAVIGATE, DISC_RECORD_POSITION, DISC_ALIGN, DISC_PREPARE_GRAB, DISC_GRAB_MATERIALS,  // 声明 DISC_GRAB_MATERIALS（圆盘GRABMATERIALS），类型和可用范围由所在声明决定。
        COARSE_NAVIGATE, COARSE_ALIGN, COARSE_PLACE,  // 声明 COARSE_PLACE，类型和可用范围由所在声明决定。
        COARSE_WAIT_RETRIEVE, COARSE_RETRIEVE,  // 声明 COARSE_RETRIEVE，类型和可用范围由所在声明决定。
        TEMP_NAVIGATE, TEMP_ALIGN, TEMP_PLACE,  // 声明 TEMP_PLACE，类型和可用范围由所在声明决定。
        HOME_PREPARE_ARM, HOME_NAVIGATE, HOME_PREPARE_VISION, HOME_ALIGN, HOME_RETURN,  // 声明 HOME_RETURN，类型和可用范围由所在声明决定。
        INACTIVE  // 续写当前声明或运算，提供 INACTIVE。
    };  // 结束此类型定义。
    CompetitionIO &io;  // 声明 io（设备接口），类型和可用范围由所在声明决定。
    CompetitionMap map;  // 声明 map，类型和可用范围由所在声明决定。
    motion::Ticket ticket;  // 声明 ticket，类型和可用范围由所在声明决定。
    RobotState observed=STATE_WAIT_START;  // 将 STATE_WAIT_START 写入或初始化 observed。
    Phase phase=Phase::START_PREPARE_ARM;  // 将 Phase::START_PREPARE_ARM 写入或初始化 phase（阶段）。
    uint8_t homeZone=0;  // 将 0 写入或初始化 homeZone。
    int preparedProgress=0;  // 将 0 写入或初始化 preparedProgress。
    uint32_t due=0,materialRequestedAt=0;  // 将 0,materialRequestedAt=0 写入或初始化 due。
    bool waitingForSubmit=false,alignmentRestarted=false,observationRetried=false,discSnapshotRetried=false;  // 将 false,alignmentRestarted=false,observationRetried=fals… 写入或初始化 waitingForSubmit。
    /**
     * @brief 把对外主状态映射到内部第一个阶段；失败或完成状态映射到 INACTIVE。
     */
    static Phase firstPhase(RobotState state) {  // 定义函数或成员 的入口 firstPhase（首个阶段），参数由此处的签名确定。
        switch(state) {  // 按 state 的值选择对应分支。
        case STATE_WAIT_START: return Phase::START_PREPARE_ARM;  // 当前值匹配 STATE_WAIT_START 时从这里执行。
        case STATE_READ_TASK: return Phase::SCAN_TASK;  // 当前值匹配 STATE_READ_TASK 时从这里执行。
        case STATE_GRAB_ROUND1:  // 当前值匹配 STATE_GRAB_ROUND1 时从这里执行。
        case STATE_GRAB_ROUND2: return Phase::DISC_NAVIGATE;  // 当前值匹配 STATE_GRAB_ROUND2 时从这里执行。
        case STATE_PLACE_COARSE1:  // 当前值匹配 STATE_PLACE_COARSE1 时从这里执行。
        case STATE_PLACE_COARSE2: return Phase::COARSE_NAVIGATE;  // 当前值匹配 STATE_PLACE_COARSE2 时从这里执行。
        // 原暂存区大状态包含“在粗加工区取回”，这里显式从取回阶段开始。
        case STATE_PLACE_TEMP1:  // 当前值匹配 STATE_PLACE_TEMP1 时从这里执行。
        case STATE_STACK_TEMP2: return Phase::COARSE_WAIT_RETRIEVE;  // 当前值匹配 STATE_STACK_TEMP2 时从这里执行。
        case STATE_RETURN_HOME: return Phase::HOME_PREPARE_ARM;  // 当前值匹配 STATE_RETURN_HOME 时从这里执行。
        default: return Phase::INACTIVE;  // 没有匹配的 case 时走此默认分支。
        }  // 结束此函数或控制块。
    }  // 结束此函数或控制块。
    /**
     * @brief 切换对外主状态，重置内部阶段和已消费的作业编号；不立即执行下一阶段。
     */
    void next(RobotState state) {  // 定义函数或成员 的入口 next（下一），参数由此处的签名确定。
        if(io.aborted()||io.state()!=observed)return;  // 当 io.aborted()||io.state()!=observed 成立时进入此分支，否则继续后续流程。
        io.state(state); // 更新对外主状态，供日志和其他任务读取。
        observed=state;  // 将 state 写入或初始化 observed。
        phase=firstPhase(state); // 从新主状态对应的第一个阶段开始。
        ticket.clear();  // 调用 ticket.clear。
        waitingForSubmit=false;  // 将 false 写入或初始化 waitingForSubmit。
    }  // 结束此函数或控制块。
    // 同一个动作反复检查只提交一次：0=执行中，1=完成，-1=失败或已取消。
    /**
     * @brief 首次调用提交动作，后续调用轮询同一编号；不阻塞主状态机。
     * @param r 待执行的请求；kind 决定运动任务中的具体执行入口。
     * @param failure 动作失败时使用的主状态，导航中的机械臂准备失败另按搬运失败处理。
     * @return 0 执行中，1 完成，-1 提交失败、执行失败或已取消。
     */
    int motion(const MotionRequest &r,RobotState failure=STATE_TRANSFER_FAILED) {  // 定义函数或成员 的入口 motion，参数由此处的签名确定。
        if(!ticket.id) {  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
            if(waitingForSubmit&&io.motionBusy())return 0;  // 若 waitingForSubmit&&io.motionBusy()，立即返回 0。
            MotionSubmitError error;  // 声明 error（错误），类型和可用范围由所在声明决定。
            ticket.id=io.submit(r,error);  // 将 io.submit(r,error) 写入或初始化 ticket.id。
            if(!ticket.id) {  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
                if(error==MotionSubmitError::BUSY||(error==MotionSubmitError::QUEUE&&!waitingForSubmit)) {  // 当 error==MotionSubmitError::BUSY||(error==MotionSubmitEr… 成立时进入此分支，否则继续后续流程。
                    waitingForSubmit=true;  // 将 true 写入或初始化 waitingForSubmit。
                    return 0;  // 结束当前函数并返回 0。
                }  // 结束此条件分支。
                waitingForSubmit=false;  // 将 false 写入或初始化 waitingForSubmit。
                io.fail(failure);  // 调用 io.fail，传入 failure。
                return -1;  // 结束当前函数并返回 -1。
            }  // 结束此条件分支。
            waitingForSubmit=false;  // 将 false 写入或初始化 waitingForSubmit。
        }  // 结束此条件分支。
        const auto s=io.status(ticket.id); // 查询同一作业的执行结果，不重复提交动作。
        if(io.aborted()||io.state()!=observed)return -1;  // 若 io.aborted()||io.state()!=observed，立即返回 -1。
        if(!s.complete())return 0;  // 若 !s.complete()，立即返回 0。
        ticket.clear();  // 调用 ticket.clear。
        if(s.state!=MotionState::DONE){  // 当 s.state!=MotionState::DONE 成立时进入此分支，否则继续后续流程。
            if(s.state==MotionState::FAILED&&r.kind==MotionKind::DISC_RECORD_POSITION&&!discSnapshotRetried) {  // 当 s.state==MotionState::FAILED&&r.kind==MotionKind::DISC… 成立时进入此分支，否则继续后续流程。
                discSnapshotRetried=true;  // 将 true 写入或初始化 discSnapshotRetried（圆盘SnapshotRetried）。
                return 0;  // 结束当前函数并返回 0。
            }  // 结束此条件分支。
            if(s.state==MotionState::FAILED&&r.kind==MotionKind::ARM_POSE&&r.observation==ArmObservation::LOOK3&&!observationRetried) {  // 当 s.state==MotionState::FAILED&&r.kind==MotionKind::ARM_… 成立时进入此分支，否则继续后续流程。
                observationRetried=true;  // 将 true 写入或初始化 observationRetried。
                return 0;  // 结束当前函数并返回 0。
            }  // 结束此条件分支。
            if(r.kind==MotionKind::NAVIGATE&&(!strcmp(s.phase,"ARM_PREPARE")||!strcmp(s.phase,"ARM_FAILED")))failure=STATE_TRANSFER_FAILED;  // 当 r.kind==MotionKind::NAVIGATE&&(!strcmp(s.phase,"ARM_PR… 成立时进入此分支，否则继续后续流程。
            io.fail(failure);return -1;  // 调用 io.fail，传入 failure。然后结束当前函数并返回 -1。
        }  // 结束此条件分支。
        observationRetried=discSnapshotRetried=false;  // 将 discSnapshotRetried=false 写入或初始化 observationRetried。
        return 1;  // 结束当前函数并返回 1。
    }  // 结束此初始化列表或局部作用域。
    /**
     * @brief 构造机械臂观察姿态请求；仅填写请求，不下发硬件命令。
     * @param value START、LOOK、LOOK2、LOOK3 或 LOOK4。
     */
    MotionRequest pose(ArmObservation value) {  // 定义函数或成员 的入口 pose，参数由此处的签名确定。
        MotionRequest r;r.kind=MotionKind::ARM_POSE;r.observation=value;r.zone=homeZone;return r;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::ARM_POSE 写入或初始化 r.kind。然后将 value 写入或初始化 r.observation。然后将 homeZone 写入或初始化 r.zone。然后结束当前函数并返回 r。
    }  // 结束此函数或控制块。
    /**
     * @brief 构造节点导航请求，并附带可提前执行的观察姿态；实际执行入口为 navigationTick。
     * @param from 起始节点编号。
     * @param to 终点节点编号。
     * @param observation 导航期间准备的观察姿态，NONE 表示不附带机械臂动作。
     * @param arrivalCargo 底盘到站时预选的载物台，0 表示不预选。
     */
    MotionRequest path(uint8_t from,uint8_t to,ArmObservation observation=ArmObservation::NONE,uint8_t arrivalCargo=0) {  // 定义函数或成员 的入口 path（路径），参数由此处的签名确定。
        MotionRequest r;r.kind=MotionKind::NAVIGATE;r.startNode=from;r.endNode=to;r.observation=observation;r.zone=homeZone;r.theta=NAN;r.cargo=arrivalCargo;return r;  // 声明 r，类型和可用范围由所在声明决定。然后将 MotionKind::NAVIGATE 写入或初始化 r.kind。然后将 from 写入或初始化 r.startNode。然后将 to 写入或初始化 r.endNode。然后将 observation 写入或初始化 r.observation。然后将 homeZone 写入或初始化 r.zone。然后将 NAN 写入或初始化 r.theta。然后将 arrivalCargo 写入或初始化 r.cargo。然后结束当前函数并返回 r。
    }  // 结束此初始化列表或局部作用域。
    /**
     * @brief 检查一次视觉对齐结果；结束时关闭视觉，失败时转入 ALIGN_FAILED。
     * @return true 对齐 DONE；false 仍在等待或已经失败。
     */
    void beginAlignment(VisionStartMode mode) {  // 定义函数或成员 的入口 beginAlignment，参数由此处的签名确定。
        alignmentRestarted=false;  // 将 false 写入或初始化 alignmentRestarted。
        io.visionStart(mode,mode==VisionStartMode::CORNER&&homeZone==2);  // 调用 io.visionStart，传入 mode,mode==VisionStartMode::CORNER&&homeZone==2。
    }  // 结束此函数或控制块。
    static constexpr bool HOME_RETURN_ENABLED=false; // 第二层叠放完成后暂时直接结束。
    static constexpr bool HOME_CORNER_ALIGNMENT_ENABLED=false; // 临时跳过返家角点视觉。
    bool aligned(VisionStartMode mode) {  // 定义函数或成员 的入口 aligned，参数由此处的签名确定。
        const auto result=io.visionStatus();if(result==AutoAlignmentState::WAITING)return false;  // 将 io.visionStatus() 写入或初始化 result（结果）。然后若 result==AutoAlignmentState::WAITING，立即返回 false。
        io.visionStop(mode); // 对齐已结束，关闭该模式的视觉控制。
        if(result==AutoAlignmentState::IDLE) {  // 当 result==AutoAlignmentState::IDLE 成立时进入此分支，否则继续后续流程。
            if(!alignmentRestarted) {  // 当 !alignmentRestarted 成立时进入此分支，否则继续后续流程。
                alignmentRestarted=true;  // 将 true 写入或初始化 alignmentRestarted。
                io.visionStart(mode,mode==VisionStartMode::CORNER&&homeZone==2);  // 调用 io.visionStart，传入 mode,mode==VisionStartMode::CORNER&&homeZone==2。
                return false;  // 结束当前函数并返回 false。
            }  // 结束此条件分支。
            io.display("ALIGN UNAVAILABLE - CONTINUE");  // 调用 io.display，传入 "ALIGN UNAVAILABLE - CONTINUE"。
            return true;  // 结束当前函数并返回 true。
        }  // 结束此条件分支。
        if(result!=AutoAlignmentState::DONE){io.fail(STATE_ALIGN_FAILED);return false;}  // 当 result!=AutoAlignmentState::DONE 成立时进入此分支，否则继续后续流程。
        return true;  // 结束当前函数并返回 true。
    }  // 结束此函数或控制块。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    CompetitionFlow(CompetitionIO &runtime,const CompetitionMap &geometry):io(runtime),map(geometry){}  // 调用 CompetitionFlow，传入 CompetitionIO &runtime,const CompetitionMap &geometry)…。

    // ================= 主状态机：从上到下阅读整场比赛 =================
    // 启动 → 扫码 → 圆盘取料 → 粗加工放料 → 粗加工取回 → 暂存放料
    //             ↑_________________________________________| 第一轮结束后重复
    // 第二轮暂存叠放完成 → 完成。启用返家时先返航归位。
    // 每次 tick 只处理当前阶段；动作完成后下一次 tick 才执行后续阶段。
    /**
     * @brief 推进一次比赛主状态机：启动、扫码、两轮取放料、返家；每次只处理当前阶段。运动通过 motion 提交，不在此等待机械行程。
     */
    void tick() {  // 定义函数或成员 的入口 tick（轮询），参数由此处的签名确定。
        const auto current=io.state();  // 将 io.state() 写入或初始化 current（当前）。
        if(current!=observed) {  // 当 current!=observed 成立时进入此分支，否则继续后续流程。
            observed=current;  // 将 current 写入或初始化 observed。
            phase=firstPhase(current);  // 将 firstPhase(current) 写入或初始化 phase（阶段）。
            ticket.clear();  // 调用 ticket.clear。
            waitingForSubmit=observationRetried=discSnapshotRetried=false;  // 将 observationRetried=discSnapshotRetried=false 写入或初始化 waitingForSubmit。
        }  // 结束此条件分支。
        if(io.aborted())return;  // 当 io.aborted() 成立时进入此分支，否则继续后续流程。
        const bool secondRound=current==STATE_GRAB_ROUND2 ||  // 将 current==STATE_GRAB_ROUND2 || 写入或初始化 secondRound（second轮次）。
            current==STATE_PLACE_COARSE2 || current==STATE_STACK_TEMP2;  // 继续上一行的组合条件，本行提供 current==STATE_PLACE_COARSE2 || current==STATE_STACK_TEMP2;。
        const auto tempVision=secondRound?VisionStartMode::WORK_AREA_LOADED:VisionStartMode::WORK_AREA;  // 将 secondRound?VisionStartMode::WORK_AREA_LOADED:VisionSt… 写入或初始化 tempVision（temp视觉）。

        switch(phase) {  // 按 phase 的值选择对应分支。
        // ---------- 启动准备：收臂、锁定起始区、标定航向、等待 Nano 扫描 ----------
        case Phase::START_PREPARE_ARM:  // 当前值匹配 Phase::START_PREPARE_ARM 时从这里执行。
            if(motion(pose(ArmObservation::START))==1) {  // 当 motion(pose(ArmObservation::START))==1 成立时进入此分支，否则继续后续流程。
                io.beginHeadingCalibration();  // 调用 io.beginHeadingCalibration。
                phase=Phase::START_CALIBRATE_HEADING;  // 将 Phase::START_CALIBRATE_HEADING 写入或初始化 phase（阶段）。
                io.display("IMU CALIBRATING");  // 调用 io.display，传入 "IMU CALIBRATING"。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        case Phase::START_CALIBRATE_HEADING:  // 当前值匹配 Phase::START_CALIBRATE_HEADING 时从这里执行。
            if(io.headingCalibrationDone()) {  // 当 io.headingCalibrationDone() 成立时进入此分支，否则继续后续流程。
                phase=Phase::START_WAIT_ZONE;  // 将 Phase::START_WAIT_ZONE 写入或初始化 phase（阶段）。
                io.display("WAIT start_zone");  // 调用 io.display，传入 "WAIT start_zone"。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        case Phase::START_WAIT_ZONE:  // 当前值匹配 Phase::START_WAIT_ZONE 时从这里执行。
            if(const uint8_t zone=io.zone()) {  // 当 const uint8_t zone=io.zone() 成立时进入此分支，否则继续后续流程。
                homeZone=zone; // 锁定开局区号，返家时使用同一区号。
                phase=Phase::START_PREPARE_LIDAR;  // 将 Phase::START_PREPARE_LIDAR 写入或初始化 phase（阶段）。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        case Phase::START_PREPARE_LIDAR: {  // 当前值匹配 Phase::START_PREPARE_LIDAR 时从这里执行。
            if(!ticket.id)ticket.id=io.lidar(homeZone);  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
            if(!ticket.id)break;  // 若 !ticket.id，从当前循环或分支退出。
            const auto preparation=io.status(ticket.id);  // 将 io.status(ticket.id) 写入或初始化 preparation。
            if(!preparation.complete())break;  // 若 !preparation.complete()，从当前循环或分支退出。
            if(preparation.state!=MotionState::DONE){io.fail(STATE_TRANSFER_FAILED);break;}  // 当 preparation.state!=MotionState::DONE 成立时进入此分支，否则继续后续流程。
            ticket.clear();  // 调用 ticket.clear。
            phase=Phase::START_WAIT_PERMISSION;  // 将 Phase::START_WAIT_PERMISSION 写入或初始化 phase（阶段）。
            io.display("WAIT START");  // 调用 io.display，传入 "WAIT START"。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。
        case Phase::START_WAIT_PERMISSION:  // 当前值匹配 Phase::START_WAIT_PERMISSION 时从这里执行。
            if(io.permission()) {  // 当 io.permission() 成立时进入此分支，否则继续后续流程。
                io.runStart(); // 启动正式计时并记录 RUN_START。
                next(STATE_READ_TASK); // 启动准备结束，进入扫码主状态。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。

        // ---------- 扫码：得到两轮颜色与位置任务 ----------
        case Phase::SCAN_TASK: {  // 当前值匹配 Phase::SCAN_TASK 时从这里执行。
            MotionRequest request;  // 声明 request（请求），类型和可用范围由所在声明决定。
            request.kind=MotionKind::SCAN_TASK;  // 将 MotionKind::SCAN_TASK 写入或初始化 request.kind。
            request.startNode=homeZone==1?map.home1:map.home2;  // 将 homeZone==1?map.home1:map.home2 写入或初始化 request.startNode。
            request.endNode=map.scan;  // 将 map.scan 写入或初始化 request.endNode。
            request.automatic=true;  // 将 true 写入或初始化 request.automatic。
            if(!ticket.id)io.display("READ TASK");  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
            if(motion(request,STATE_SCAN_FAILED)==1) { // 扫码作业完成后再发布任务；失败按扫码错误处理。
                io.taskReady(); // 发布任务码显示。
                next(STATE_GRAB_ROUND1); // 开始第一轮圆盘取料。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。

        // ---------- 圆盘：导航并提前 LOOK2 → 记录位置 → 对齐 → 取三件 ----------
        case Phase::DISC_NAVIGATE:  // 当前值匹配 Phase::DISC_NAVIGATE 时从这里执行。
            io.display(secondRound?"GRAB R2":"GRAB R1");  // 调用 io.display，传入 secondRound?"GRAB R2":"GRAB R1"。
            if(!ticket.id) {  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
                io.materialStop();  // 调用 io.materialStop。
                io.clearProgress();  // 调用 io.clearProgress。
            }  // 结束此条件分支。
            if(motion(path(secondRound?map.temp:map.scan,map.disc,ArmObservation::LOOK2,CargoForGrabIndex(0)),STATE_ROUTE_FAILED)==1)  // 当 motion(path(secondRound?map.temp:map.scan,map.disc,Arm… 成立时进入此分支，否则继续后续流程。
                phase=Phase::DISC_RECORD_POSITION;  // 将 Phase::DISC_RECORD_POSITION 写入或初始化 phase（阶段）。
            break;  // 退出当前循环或 switch 分支。
        // 进入对齐前记录底盘参考位置，后续离开圆盘区会用它计算偏移。
        // 排查回退量时应将这次采样与实际返程使用的基准对照。
        case Phase::DISC_RECORD_POSITION: {  // 当前值匹配 Phase::DISC_RECORD_POSITION 时从这里执行。
            MotionRequest request;  // 声明 request（请求），类型和可用范围由所在声明决定。
            request.kind=MotionKind::DISC_RECORD_POSITION;  // 将 MotionKind::DISC_RECORD_POSITION 写入或初始化 request.kind。
            if(motion(request,STATE_ROUTE_FAILED)==1) {  // 当 motion(request,STATE_ROUTE_FAILED)==1 成立时进入此分支，否则继续后续流程。
                beginAlignment(VisionStartMode::DISC);  // 调用 beginAlignment，传入 VisionStartMode::DISC。
                phase=Phase::DISC_ALIGN;  // 将 Phase::DISC_ALIGN 写入或初始化 phase（阶段）。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。
        case Phase::DISC_ALIGN:  // 当前值匹配 Phase::DISC_ALIGN 时从这里执行。
            if(aligned(VisionStartMode::DISC)) { // 检查圆盘对齐结果；完成时关闭本次视觉对齐。
                due=io.now()+10; // 保留原对齐后间隔。
                phase=Phase::DISC_PREPARE_GRAB;  // 将 Phase::DISC_PREPARE_GRAB 写入或初始化 phase（阶段）。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        case Phase::DISC_PREPARE_GRAB:  // 当前值匹配 Phase::DISC_PREPARE_GRAB 时从这里执行。
            if(static_cast<int32_t>(io.now()-due)<0)return;  // 当 static_cast<int32_t>(io.now()-due)<0 成立时进入此分支，否则继续后续流程。
            if(motion(pose(ArmObservation::LOOK3))==1) { // 对齐后切到单件物料观察姿态。
                preparedProgress=0;  // 将 0 写入或初始化 preparedProgress。
                io.materialStart(); // 开启单件物料识别，后续抓取由运动任务执行。
                materialRequestedAt=io.now();  // 将 io.now() 写入或初始化 materialRequestedAt（物料RequestedAt）。
                phase=Phase::DISC_GRAB_MATERIALS;  // 将 Phase::DISC_GRAB_MATERIALS 写入或初始化 phase（阶段）。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        case Phase::DISC_GRAB_MATERIALS: {  // 当前值匹配 Phase::DISC_GRAB_MATERIALS 时从这里执行。
            const int progress=io.progress(); // 读取已抓取并登记到载物台的件数。
            if(progress>=3) {  // 当 progress>=3 成立时进入此分支，否则继续后续流程。
                io.materialStop();  // 调用 io.materialStop。
                io.clearProgress();  // 调用 io.clearProgress。
                next(secondRound?STATE_PLACE_COARSE2:STATE_PLACE_COARSE1);  // 调用 next（下一），传入 secondRound?STATE_PLACE_COARSE2:STATE_PLACE_COARSE1。
                return;  // 结束当前函数，将执行权交回调用者。
            }  // 结束此条件分支。
            // 前两件抓完后重新进入 LOOK3，再开启下一件识别。
            if(progress>preparedProgress&&motion(pose(ArmObservation::LOOK3))==1) { // 新抓完一件后恢复 LOOK3，准备识别下一件。
                preparedProgress=progress;  // 将 progress 写入或初始化 preparedProgress。
                io.materialStart(); // 开启单件物料识别，后续抓取由运动任务执行。
                materialRequestedAt=io.now();  // 将 io.now() 写入或初始化 materialRequestedAt（物料RequestedAt）。
            }  // 结束此条件分支。
            if(progress==preparedProgress&&!io.motionBusy()&&io.now()-materialRequestedAt>=15000) {  // 当 progress==preparedProgress&&!io.motionBusy()&&io.now()… 成立时进入此分支，否则继续后续流程。
                io.materialStop();  // 调用 io.materialStop。
                io.materialStart();  // 调用 io.materialStart。
                materialRequestedAt=io.now();  // 将 io.now() 写入或初始化 materialRequestedAt（物料RequestedAt）。
                io.display("DISC MATERIAL RESCAN");  // 调用 io.display，传入 "DISC MATERIAL RESCAN"。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。
        // ---------- 粗加工区：导航并提前 LOOK → 对齐 → 放下三件 ----------
        case Phase::COARSE_NAVIGATE: {  // 当前值匹配 Phase::COARSE_NAVIGATE 时从这里执行。
            if(!ticket.id) {  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
                io.materialStop();  // 调用 io.materialStop。
                io.display(secondRound?"GO COARSE2":"GO COARSE1");  // 调用 io.display，传入 secondRound?"GO COARSE2":"GO COARSE1"。
            }  // 结束此条件分支。
            auto request=path(map.disc,map.coarse,ArmObservation::LOOK,CargoForGrabIndex(0));  // 将 path(map.disc,map.coarse,ArmObservation::LOOK,CargoFor… 写入或初始化 request（请求）。
            request.discDeparture=true;  // 将 true 写入或初始化 request.discDeparture。
            if(motion(request,STATE_ROUTE_FAILED)==1) {  // 当 motion(request,STATE_ROUTE_FAILED)==1 成立时进入此分支，否则继续后续流程。
                beginAlignment(VisionStartMode::WORK_AREA);  // 调用 beginAlignment，传入 VisionStartMode::WORK_AREA。
                phase=Phase::COARSE_ALIGN;  // 将 Phase::COARSE_ALIGN 写入或初始化 phase（阶段）。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。
        case Phase::COARSE_ALIGN:  // 当前值匹配 Phase::COARSE_ALIGN 时从这里执行。
            if(aligned(VisionStartMode::WORK_AREA))phase=Phase::COARSE_PLACE; // 对齐完成后进入放料阶段。
            break;  // 退出当前循环或 switch 分支。
        case Phase::COARSE_PLACE: {  // 当前值匹配 Phase::COARSE_PLACE 时从这里执行。
            MotionRequest request;  // 声明 request（请求），类型和可用范围由所在声明决定。
            request.kind=MotionKind::PLACE_ROUND;request.automatic=true;  // 将 MotionKind::PLACE_ROUND 写入或初始化 request.kind。然后将 true 写入或初始化 request.automatic。
            io.task(request,secondRound); // 将本轮三件物料的颜色和位置填入动作请求。
            request.layer=1;  // 将 1 写入或初始化 request.layer。
            request.ringCorrection=true;  // 将 true 写入或初始化 request.ringCorrection。
            if(!ticket.id)io.display(secondRound?"PLACE C2":"PLACE C1");  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
            if(motion(request)==1) { // 提交并等待三件物料放入粗加工区。
                io.clearProgress();  // 调用 io.clearProgress。
                next(secondRound?STATE_STACK_TEMP2:STATE_PLACE_TEMP1); // 转入取回及暂存流程，当前车体仍在粗加工区。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。

        // ---------- 粗加工区取回：仍在粗加工区，取回后才出发去暂存区 ----------
        // 对外仍归属 PLACE_TEMP1 / STACK_TEMP2，保持原状态编号与日志边界。
        case Phase::COARSE_WAIT_RETRIEVE:  // 当前值匹配 Phase::COARSE_WAIT_RETRIEVE 时从这里执行。
            due=io.now()+10; // 保留原放料与取回之间的间隔。
            phase=Phase::COARSE_RETRIEVE;  // 将 Phase::COARSE_RETRIEVE 写入或初始化 phase（阶段）。
            io.display(secondRound?"WAIT RETRIEVE C2":"WAIT RETRIEVE C1");  // 调用 io.display，传入 secondRound?"WAIT RETRIEVE C2":"WAIT RETRIEVE C1"。
            return;  // 结束当前函数，将执行权交回调用者。
        case Phase::COARSE_RETRIEVE: {  // 当前值匹配 Phase::COARSE_RETRIEVE 时从这里执行。
            if(static_cast<int32_t>(io.now()-due)<0)return;  // 当 static_cast<int32_t>(io.now()-due)<0 成立时进入此分支，否则继续后续流程。
            MotionRequest request;  // 声明 request（请求），类型和可用范围由所在声明决定。
            request.kind=MotionKind::RETRIEVE_ROUND;request.automatic=true;  // 将 MotionKind::RETRIEVE_ROUND 写入或初始化 request.kind。然后将 true 写入或初始化 request.automatic。
            io.task(request,secondRound); // 将本轮三件物料的颜色和位置填入动作请求。
            if(!ticket.id)io.display(secondRound?"RETRIEVE C2":"RETRIEVE C1");  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
            if(motion(request)==1) { // 提交并等待三件物料从粗加工区取回载物台。
                io.clearProgress();  // 调用 io.clearProgress。
                phase=Phase::TEMP_NAVIGATE;  // 将 Phase::TEMP_NAVIGATE 写入或初始化 phase（阶段）。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。

        // ---------- 暂存区：导航并提前 LOOK → 对齐 → 第一轮放料 / 第二轮叠放 ----------
        case Phase::TEMP_NAVIGATE:  // 当前值匹配 Phase::TEMP_NAVIGATE 时从这里执行。
            io.display(secondRound?"GO TEMP2":"GO TEMP1");  // 调用 io.display，传入 secondRound?"GO TEMP2":"GO TEMP1"。
            if(motion(path(map.coarse,map.temp,ArmObservation::LOOK,CargoForGrabIndex(0)),STATE_ROUTE_FAILED)==1) { // 三件取回后去暂存区，途中提前展开 LOOK。
                beginAlignment(tempVision);  // 调用 beginAlignment，传入 tempVision。
                phase=Phase::TEMP_ALIGN;  // 将 Phase::TEMP_ALIGN 写入或初始化 phase（阶段）。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        case Phase::TEMP_ALIGN:  // 当前值匹配 Phase::TEMP_ALIGN 时从这里执行。
            if(aligned(tempVision))phase=Phase::TEMP_PLACE; // 暂存区对齐完成后放料或叠放。
            break;  // 退出当前循环或 switch 分支。
        // 暂存区是两轮共用流程中的分叉。
        // 这里根据轮次选择放料或叠放，位置输入应结合第一轮物料分布及第二轮颜色顺序阅读。
        case Phase::TEMP_PLACE: {  // 当前值匹配 Phase::TEMP_PLACE 时从这里执行。
            MotionRequest request;  // 声明 request（请求），类型和可用范围由所在声明决定。
            request.kind=MotionKind::PLACE_ROUND;request.automatic=true;  // 将 MotionKind::PLACE_ROUND 写入或初始化 request.kind。然后将 true 写入或初始化 request.automatic。
            io.task(request,secondRound); // 将本轮三件物料的颜色和位置填入动作请求。
            if(secondRound) {  // 当 secondRound 成立时进入此分支，否则继续后续流程。
                MotionRequest firstRound;  // 声明 firstRound（首个轮次），类型和可用范围由所在声明决定。
                io.task(firstRound,false);  // 调用 io.task，传入 firstRound,false。
                // 第四段只规定粗加工位置；暂存保持第二批顺序，叠到第一批同色物料上。
                for(int i=0;i<3;++i)  // 按 int i=0;i<3;++i 组织循环的初值、继续条件和步进。
                    for(int j=0;j<3;++j)  // 按 int j=0;j<3;++j 组织循环的初值、继续条件和步进。
                        if(request.colors[i]==firstRound.colors[j])request.positions[i]=firstRound.positions[j];  // 当 request.colors[i]==firstRound.colors[j] 成立时进入此分支，否则继续后续流程。
            }  // 结束此条件分支。
            request.layer=secondRound?2:1;  // 将 secondRound?2:1 写入或初始化 request.layer。
            request.ringCorrection=!secondRound;  // 将 !secondRound 写入或初始化 request.ringCorrection。
            if(!ticket.id)io.display(secondRound?"STACK T2":"PLACE T1");  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
            if(motion(request)==1) { // 提交并等待本轮暂存放料或第二层叠放。
                io.clearProgress();  // 调用 io.clearProgress。
                if(!secondRound)next(STATE_GRAB_ROUND2);  // 当 !secondRound 成立时进入此分支，否则继续后续流程。
                else if(HOME_RETURN_ENABLED)next(STATE_RETURN_HOME);  // 仅在先前条件均不成立时，当 HOME_RETURN_ENABLED 成立时进入此分支，否则继续后续流程。
                else {  // 前一个判断不成立时执行此分支。
                    next(STATE_DONE);  // 调用 next（下一），传入 STATE_DONE。
                    if(io.state()==STATE_DONE)io.runDone();  // 当 io.state()==STATE_DONE 成立时进入此分支，否则继续后续流程。
                }  // 结束此条件分支。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。

        // ---------- 返家：路径前收臂 → 导航并提前 LOOK4 → 角点对齐 → 归位 ----------
        case Phase::HOME_PREPARE_ARM:  // 当前值匹配 Phase::HOME_PREPARE_ARM 时从这里执行。
            if(!ticket.id)io.materialStop();  // 当 !ticket.id 成立时进入此分支，否则继续后续流程。
            if(motion(pose(ArmObservation::START))==1)phase=Phase::HOME_NAVIGATE; // 先收臂，再开始返家路径。
            break;  // 退出当前循环或 switch 分支。
        case Phase::HOME_NAVIGATE: {  // 当前值匹配 Phase::HOME_NAVIGATE 时从这里执行。
            auto request=path(map.temp,homeZone==1?map.home1:map.home2,ArmObservation::LOOK4); // 按开局区号选择终点，途中提前展开 LOOK4。
            request.theta=map.homeHeading;  // 将 map.homeHeading 写入或初始化 request.theta。
            if(motion(request,STATE_ROUTE_FAILED)==1)phase=Phase::HOME_PREPARE_VISION; // 返家导航完成后执行视觉前准备。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。
        case Phase::HOME_PREPARE_VISION: {  // 当前值匹配 Phase::HOME_PREPARE_VISION 时从这里执行。
            MotionRequest request;  // 声明 request（请求），类型和可用范围由所在声明决定。
            request.kind=MotionKind::HOME_OBSERVE;  // 将 MotionKind::HOME_OBSERVE 写入或初始化 request.kind。
            request.zone=homeZone;  // 将 homeZone 写入或初始化 request.zone。
            request.armPrepared=true; // LOOK4 已随导航完成。
            if(motion(request)==1) { // 等待返家校角及区 2 右移完成。
                if(HOME_CORNER_ALIGNMENT_ENABLED) {  // 当 HOME_CORNER_ALIGNMENT_ENABLED 成立时进入此分支，否则继续后续流程。
                    beginAlignment(VisionStartMode::CORNER);  // 调用 beginAlignment，传入 VisionStartMode::CORNER。
                    phase=Phase::HOME_ALIGN;  // 将 Phase::HOME_ALIGN 写入或初始化 phase（阶段）。
                } else phase=Phase::HOME_RETURN;  // 前一个判断不成立时执行此分支。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。
        case Phase::HOME_ALIGN:  // 当前值匹配 Phase::HOME_ALIGN 时从这里执行。
            if(aligned(VisionStartMode::CORNER))phase=Phase::HOME_RETURN; // 角点对齐完成后收臂并执行最终归位。
            break;  // 退出当前循环或 switch 分支。
        case Phase::HOME_RETURN: {  // 当前值匹配 Phase::HOME_RETURN 时从这里执行。
            MotionRequest request;  // 声明 request（请求），类型和可用范围由所在声明决定。
            request.kind=MotionKind::HOME_RETURN;  // 将 MotionKind::HOME_RETURN 写入或初始化 request.kind。
            request.zone=homeZone;  // 将 homeZone 写入或初始化 request.zone。
            if(motion(request)==1) { // 提交并等待最终收臂及归位动作。
                io.clearProgress();  // 调用 io.clearProgress。
                next(STATE_DONE); // 全部动作结束，切换到完成状态。
                if(io.state()==STATE_DONE)io.runDone(); // 结束正式计时、记录结果并提交电机失能。
            }  // 结束此条件分支。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此条件分支。
        case Phase::INACTIVE: // 完成、失败等对外状态沿用原处理，不再发起动作。
            break;  // 退出当前循环或 switch 分支。
        }  // 结束此函数或控制块。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
