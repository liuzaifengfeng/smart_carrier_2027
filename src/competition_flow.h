#pragma once
#include "robot_state.h"
#include "motion_job.h"
#include "vision_alignment.h"
#include "arm_motion.h"
#include <string.h>

struct CompetitionMap {
    uint8_t scan=2,disc=14,coarse=10,temp=22,home1=4,home2=0;
    float homeHeading=180;
};
struct CompetitionIO {
    virtual ~CompetitionIO()=default;
    /**
     * @brief 读取主流程使用的设备运行毫秒数，供阶段间隔计时。
     */
    virtual uint32_t now()=0;
    /**
     * @brief 读取或更新对外 RobotState；读接口返回当前状态，写接口供阶段切换使用。
     */
    virtual RobotState state()=0;
    /**
     * @brief 读取或更新对外 RobotState；读接口返回当前状态，写接口供阶段切换使用。
     */
    virtual void state(RobotState value)=0;
    /**
     * @brief 查询整场任务是否已取消，取消后主流程不再提交动作。
     */
    virtual bool aborted()=0;
    /**
     * @brief 读取当前选择的起始区号；主流程在启动准备时锁定返家区号。
     */
    virtual uint8_t zone()=0;
    /**
     * @brief 查询正式比赛启动许可，许可到达后开始计时并进入扫码。
     */
    virtual bool permission()=0;
    /**
     * @brief 读取当前轮已经完成抓取并登记到载物台的物料件数。
     */
    virtual int progress()=0;
    /**
     * @brief 将当前轮圆盘抓取进度清零，供进入下一轮或阶段时使用。
     */
    virtual void clearProgress()=0;
    /**
     * @brief 将指定轮次的三件颜色和工位编号填入运动请求。
     * @param request 接收本轮任务数据的动作请求。
     * @param secondRound true 第二轮，false 第一轮。
     */
    virtual void task(MotionRequest &request,bool secondRound)=0;
    /**
     * @brief 提交一个非阻塞运动请求；返回作业编号，后续用 status 查询完成结果。
     */
    virtual uint32_t submit(const MotionRequest &request)=0;
    /**
     * @brief 提交起始区扫码准备姿态作业；返回作业编号，不在主流程等待物理动作。
     */
    virtual uint32_t lidar()=0;
    /**
     * @brief 读取指定作业的执行快照；只有终态 DONE 才表示该作业按当前完成策略结束。
     */
    virtual MotionStatus status(uint32_t id)=0;
    /**
     * @brief 请求开启指定视觉功能；工位对齐与单件物料识别使用不同模式。
     */
    virtual void visionStart(VisionStartMode mode)=0;
    /**
     * @brief 读取当前自动视觉对齐的 WAITING、DONE 或 FAILED 状态。
     */
    virtual AutoAlignmentState visionStatus()=0;
    /**
     * @brief 请求停止指定视觉功能，结束该阶段的视觉控制。
     */
    virtual void visionStop(VisionStartMode mode)=0;
    /**
     * @brief 开启圆盘单件物料识别；识别结果触发独立的抓取作业。
     */
    virtual void materialStart()=0;
    /**
     * @brief 停止圆盘单件物料识别并清除对应就绪标志。
     */
    virtual void materialStop()=0;
    /**
     * @brief 发布已解析的任务码显示，并设置扫码结束后的位姿准备标志。
     */
    virtual void taskReady()=0;
    /**
     * @brief 记录正式开始时间、输出 RUN_START，并启动比赛总计时器。
     */
    virtual void runStart()=0;
    /**
     * @brief 输出正式完成结果、停止比赛计时，并提交电机失能作业。
     */
    virtual void runDone()=0;
    /**
     * @brief 转入指定业务失败状态，取消运动并更新显示；沿用现有失败处理。
     */
    virtual void fail(RobotState reason)=0;
    /**
     * @brief 向上位机发布当前业务阶段的显示文本。
     */
    virtual void display(const char *text)=0;
};
// 业务流程只负责提交动作、等待结果和切换阶段；电机执行仍由 MotionExecutor 负责。
// 对外 RobotState 保持原编号和日志名称；下面的 Phase 把每个大状态展开为可直接阅读的小阶段。
class CompetitionFlow {
    enum class Phase {
        START_PREPARE_ARM, START_WAIT_ZONE, START_PREPARE_LIDAR, START_WAIT_PERMISSION,
        SCAN_TASK,
        DISC_NAVIGATE, DISC_ALIGN, DISC_PREPARE_GRAB, DISC_GRAB_MATERIALS,
        COARSE_NAVIGATE, COARSE_ALIGN, COARSE_PLACE,
        COARSE_WAIT_RETRIEVE, COARSE_RETRIEVE,
        TEMP_NAVIGATE, TEMP_ALIGN, TEMP_PLACE,
        HOME_PREPARE_ARM, HOME_NAVIGATE, HOME_PREPARE_VISION, HOME_ALIGN, HOME_RETURN,
        INACTIVE
    };
    CompetitionIO &io;
    CompetitionMap map;
    motion::Ticket ticket;
    RobotState observed=STATE_WAIT_START;
    Phase phase=Phase::START_PREPARE_ARM;
    uint8_t homeZone=0;
    int preparedProgress=0;
    uint32_t due=0;
    /**
     * @brief 把对外主状态映射到内部第一个阶段；失败或完成状态映射到 INACTIVE。
     */
    static Phase firstPhase(RobotState state) {
        switch(state) {
        case STATE_WAIT_START: return Phase::START_PREPARE_ARM;
        case STATE_READ_TASK: return Phase::SCAN_TASK;
        case STATE_GRAB_ROUND1:
        case STATE_GRAB_ROUND2: return Phase::DISC_NAVIGATE;
        case STATE_PLACE_COARSE1:
        case STATE_PLACE_COARSE2: return Phase::COARSE_NAVIGATE;
        // 原暂存区大状态包含“在粗加工区取回”，这里显式从取回阶段开始。
        case STATE_PLACE_TEMP1:
        case STATE_STACK_TEMP2: return Phase::COARSE_WAIT_RETRIEVE;
        case STATE_RETURN_HOME: return Phase::HOME_PREPARE_ARM;
        default: return Phase::INACTIVE;
        }
    }
    /**
     * @brief 切换对外主状态，重置内部阶段和已消费的作业编号；不立即执行下一阶段。
     */
    void next(RobotState state) {
        if(io.aborted()||io.state()!=observed)return;
        io.state(state); // 更新对外主状态，供日志和其他任务读取。
        observed=state;
        phase=firstPhase(state); // 从新主状态对应的第一个阶段开始。
        ticket.clear();
    }
    // 同一个动作反复检查只提交一次：0=执行中，1=完成，-1=失败或已取消。
    /**
     * @brief 首次调用提交动作，后续调用轮询同一编号；不阻塞主状态机。
     * @param r 待执行的请求；kind 决定运动任务中的具体执行入口。
     * @param failure 动作失败时使用的主状态，导航中的机械臂准备失败另按搬运失败处理。
     * @return 0 执行中，1 完成，-1 提交失败、执行失败或已取消。
     */
    int motion(const MotionRequest &r,RobotState failure=STATE_TRANSFER_FAILED) {
        if(!ticket.submitOnce(r,[&](const MotionRequest &value){return io.submit(value);})) {io.fail(failure);return -1;}
        const auto s=io.status(ticket.id); // 查询同一作业的执行结果，不重复提交动作。
        if(io.aborted()||io.state()!=observed)return -1;
        if(!s.complete())return 0;
        ticket.clear();
        if(s.state!=MotionState::DONE){
            if(r.kind==MotionKind::NAVIGATE&&(!strcmp(s.phase,"ARM_PREPARE")||!strcmp(s.phase,"ARM_FAILED")))failure=STATE_TRANSFER_FAILED;
            io.fail(failure);return -1;
        }
        return 1;
    }
    /**
     * @brief 构造机械臂观察姿态请求；仅填写请求，不下发硬件命令。
     * @param value START、LOOK、LOOK2、LOOK3 或 LOOK4。
     */
    MotionRequest pose(ArmObservation value) {
        MotionRequest r;r.kind=MotionKind::ARM_POSE;r.observation=value;r.zone=homeZone;return r;
    }
    /**
     * @brief 构造节点导航请求，并附带可提前执行的观察姿态；实际执行入口为 navigationTick。
     * @param from 起始节点编号。
     * @param to 终点节点编号。
     * @param observation 导航期间准备的观察姿态，NONE 表示不附带机械臂动作。
     */
    MotionRequest path(uint8_t from,uint8_t to,ArmObservation observation=ArmObservation::NONE) {
        MotionRequest r;r.kind=MotionKind::NAVIGATE;r.startNode=from;r.endNode=to;r.observation=observation;r.zone=homeZone;r.theta=NAN;return r;
    }
    /**
     * @brief 检查一次视觉对齐结果；结束时关闭视觉，失败时转入 ALIGN_FAILED。
     * @return true 对齐 DONE；false 仍在等待或已经失败。
     */
    bool aligned(VisionStartMode mode) {
        const auto result=io.visionStatus();if(result==AutoAlignmentState::WAITING)return false;
        io.visionStop(mode); // 对齐已结束，关闭该模式的视觉控制。
        if(result!=AutoAlignmentState::DONE){io.fail(STATE_ALIGN_FAILED);return false;}
        return true;
    }
public:
    CompetitionFlow(CompetitionIO &runtime,const CompetitionMap &geometry):io(runtime),map(geometry){}

    // ================= 主状态机：从上到下阅读整场比赛 =================
    // 启动 → 扫码 → 圆盘取料 → 粗加工放料 → 粗加工取回 → 暂存放料
    //             ↑_________________________________________| 第一轮结束后重复
    // 第二轮暂存叠放完成 → 返家 → 角点对齐 → 收臂归位 → 完成。
    // 每次 tick 只处理当前阶段；动作完成后下一次 tick 才执行后续阶段。
    /**
     * @brief 推进一次比赛主状态机：启动、扫码、两轮取放料、返家；每次只处理当前阶段。运动通过 motion 提交，不在此等待机械行程。
     */
    void tick() {
        const auto current=io.state();
        if(current!=observed) {
            observed=current;
            phase=firstPhase(current);
            ticket.clear();
        }
        if(io.aborted())return;
        const bool secondRound=current==STATE_GRAB_ROUND2 ||
            current==STATE_PLACE_COARSE2 || current==STATE_STACK_TEMP2;
        const auto tempVision=secondRound?VisionStartMode::WORK_AREA_LOADED:VisionStartMode::WORK_AREA;

        switch(phase) {
        // ---------- 启动准备：收臂、锁定起始区、准备扫码姿态、等待启动 ----------
        case Phase::START_PREPARE_ARM:
            if(motion(pose(ArmObservation::START))==1) { // 提交收臂动作，完成后才等待起始区。
                phase=Phase::START_WAIT_ZONE;
                io.display("WAIT start_zone");
            }
            break;
        case Phase::START_WAIT_ZONE:
            if(io.zone()) {
                homeZone=io.zone(); // 锁定开局区号，返家时使用同一区号。
                phase=Phase::START_PREPARE_LIDAR;
            }
            break;
        case Phase::START_PREPARE_LIDAR:
            if(!ticket.id)ticket.id=io.lidar(); // 提交一次扫码准备姿态动作，保存作业编号。
            if(!STARTUP_LIDAR_SCAN_ENABLED) {
                if(!ticket.id)break; // 尚未提交斜移，继续等待提交成功。
                const auto preparation=io.status(ticket.id);
                if(!preparation.complete())break;
                if(preparation.state!=MotionState::DONE){io.fail(STATE_TRANSFER_FAILED);break;}
                ticket.clear();
                io.display("AUTO START");
                io.runStart(); // 跳过雷达时，斜移成功完成即开始正式计时。
                next(STATE_READ_TASK); // 直接扫码，不再等待外部START。
                break;
            }
            if(!ticket.id||io.status(ticket.id).complete()) {
                ticket.clear();
                phase=Phase::START_WAIT_PERMISSION;
                io.display("WAIT START");
            }
            break;
        case Phase::START_WAIT_PERMISSION:
            if(io.permission()) {
                io.runStart(); // 启动正式计时并记录 RUN_START。
                next(STATE_READ_TASK); // 启动准备结束，进入扫码主状态。
            }
            break;

        // ---------- 扫码：得到两轮颜色与位置任务 ----------
        case Phase::SCAN_TASK: {
            MotionRequest request;
            request.kind=MotionKind::SCAN_TASK;
            request.zone=homeZone;
            io.display("READ TASK");
            if(motion(request,STATE_SCAN_FAILED)==1) { // 扫码作业完成后再发布任务；失败按扫码错误处理。
                io.taskReady(); // 发布任务码显示，并设置扫码结束后的准备标志。
                next(STATE_GRAB_ROUND1); // 开始第一轮圆盘取料。
            }
            break;
        }

        // ---------- 圆盘：导航并提前 LOOK2 → 对齐 → LOOK3 → 依次取三件 ----------
        case Phase::DISC_NAVIGATE:
            io.display(secondRound?"GRAB R2":"GRAB R1");
            if(!ticket.id) {
                io.materialStop();
                io.clearProgress();
            }
            if(motion(path(secondRound?map.temp:map.scan,map.disc,ArmObservation::LOOK2),STATE_ROUTE_FAILED)==1) { // 去圆盘并提前 LOOK2，等待底盘和观察姿态都完成。
                io.visionStart(VisionStartMode::DISC); // 到位后开启圆盘整体对齐。
                phase=Phase::DISC_ALIGN;
            }
            break;
        case Phase::DISC_ALIGN:
            if(aligned(VisionStartMode::DISC)) { // 检查圆盘对齐结果；完成时关闭本次视觉对齐。
                due=io.now()+10; // 保留原对齐后间隔。
                phase=Phase::DISC_PREPARE_GRAB;
            }
            break;
        case Phase::DISC_PREPARE_GRAB:
            if(static_cast<int32_t>(io.now()-due)<0)return;
            if(motion(pose(ArmObservation::LOOK3))==1) { // 对齐后切到单件物料观察姿态。
                preparedProgress=0;
                io.materialStart(); // 开启单件物料识别，后续抓取由运动任务执行。
                phase=Phase::DISC_GRAB_MATERIALS;
            }
            break;
        case Phase::DISC_GRAB_MATERIALS: {
            const int progress=io.progress(); // 读取已抓取并登记到载物台的件数。
            if(progress>=3) {
                io.materialStop();
                io.clearProgress();
                next(secondRound?STATE_PLACE_COARSE2:STATE_PLACE_COARSE1); // 三件取齐，进入本轮粗加工区放料。
                return;
            }
            // 前两件抓完后重新进入 LOOK3，再开启下一件识别；第三件完成直接离开。
            if(progress>preparedProgress&&motion(pose(ArmObservation::LOOK3))==1) { // 新抓完一件后恢复 LOOK3，准备识别下一件。
                preparedProgress=progress;
                io.materialStart(); // 开启单件物料识别，后续抓取由运动任务执行。
            }
            break;
        }

        // ---------- 粗加工区：导航并提前 LOOK → 对齐 → 放下三件 ----------
        case Phase::COARSE_NAVIGATE:
            if(!ticket.id) {
                io.materialStop();
                io.display(secondRound?"GO COARSE2":"GO COARSE1");
            }
            if(motion(path(map.disc,map.coarse,ArmObservation::LOOK),STATE_ROUTE_FAILED)==1) { // 去粗加工区，途中提前展开 LOOK。
                io.visionStart(VisionStartMode::WORK_AREA); // 开启粗加工区工位对齐。
                phase=Phase::COARSE_ALIGN;
            }
            break;
        case Phase::COARSE_ALIGN:
            if(aligned(VisionStartMode::WORK_AREA))phase=Phase::COARSE_PLACE; // 对齐完成后进入放料阶段。
            break;
        case Phase::COARSE_PLACE: {
            MotionRequest request;
            request.kind=MotionKind::PLACE_ROUND;
            io.task(request,secondRound); // 将本轮三件物料的颜色和位置填入动作请求。
            request.layer=1;
            io.display(secondRound?"PLACE C2":"PLACE C1");
            if(motion(request)==1) { // 提交并等待三件物料放入粗加工区。
                io.clearProgress();
                next(secondRound?STATE_STACK_TEMP2:STATE_PLACE_TEMP1); // 转入取回及暂存流程，当前车体仍在粗加工区。
            }
            break;
        }

        // ---------- 粗加工区取回：仍在粗加工区，取回后才出发去暂存区 ----------
        // 对外仍归属 PLACE_TEMP1 / STACK_TEMP2，保持原状态编号与日志边界。
        case Phase::COARSE_WAIT_RETRIEVE:
            due=io.now()+10; // 保留原放料与取回之间的间隔。
            phase=Phase::COARSE_RETRIEVE;
            io.display(secondRound?"WAIT RETRIEVE C2":"WAIT RETRIEVE C1");
            return;
        case Phase::COARSE_RETRIEVE: {
            if(static_cast<int32_t>(io.now()-due)<0)return;
            MotionRequest request;
            request.kind=MotionKind::RETRIEVE_ROUND;
            io.task(request,secondRound); // 将本轮三件物料的颜色和位置填入动作请求。
            io.display(secondRound?"RETRIEVE C2":"RETRIEVE C1");
            if(motion(request)==1) { // 提交并等待三件物料从粗加工区取回载物台。
                io.clearProgress();
                phase=Phase::TEMP_NAVIGATE;
            }
            break;
        }

        // ---------- 暂存区：导航并提前 LOOK → 对齐 → 第一轮放料 / 第二轮叠放 ----------
        case Phase::TEMP_NAVIGATE:
            io.display(secondRound?"GO TEMP2":"GO TEMP1");
            if(motion(path(map.coarse,map.temp,ArmObservation::LOOK),STATE_ROUTE_FAILED)==1) { // 三件取回后去暂存区，途中提前展开 LOOK。
                io.visionStart(tempVision); // 第一轮使用空工位对齐，第二轮使用已有物料的工位对齐。
                phase=Phase::TEMP_ALIGN;
            }
            break;
        case Phase::TEMP_ALIGN:
            if(aligned(tempVision))phase=Phase::TEMP_PLACE; // 暂存区对齐完成后放料或叠放。
            break;
        case Phase::TEMP_PLACE: {
            MotionRequest request;
            request.kind=MotionKind::PLACE_ROUND;
            io.task(request,secondRound); // 将本轮三件物料的颜色和位置填入动作请求。
            request.layer=secondRound?2:1;
            io.display(secondRound?"STACK T2":"PLACE T1");
            if(motion(request)==1) { // 提交并等待本轮暂存放料或第二层叠放。
                io.clearProgress();
                next(secondRound?STATE_RETURN_HOME:STATE_GRAB_ROUND2); // 第一轮继续取料，第二轮完成后返家。
            }
            break;
        }

        // ---------- 返家：路径前收臂 → 导航并提前 LOOK4 → 角点对齐 → 归位 ----------
        case Phase::HOME_PREPARE_ARM:
            if(!ticket.id)io.materialStop();
            if(motion(pose(ArmObservation::START))==1)phase=Phase::HOME_NAVIGATE; // 先收臂，再开始返家路径。
            break;
        case Phase::HOME_NAVIGATE: {
            auto request=path(map.temp,homeZone==1?map.home1:map.home2,ArmObservation::LOOK4); // 按开局区号选择终点，途中提前展开 LOOK4。
            request.theta=map.homeHeading;
            if(motion(request,STATE_ROUTE_FAILED)==1)phase=Phase::HOME_PREPARE_VISION; // 返家导航完成后执行视觉前准备。
            break;
        }
        case Phase::HOME_PREPARE_VISION: {
            MotionRequest request;
            request.kind=MotionKind::HOME_OBSERVE;
            request.zone=homeZone;
            request.armPrepared=true; // LOOK4 已随导航完成；保留原朝向修正。
            if(motion(request)==1) { // 执行原返家朝向修正，复用已完成的 LOOK4。
                io.visionStart(VisionStartMode::CORNER); // 朝向准备完成，开启角点对齐。
                phase=Phase::HOME_ALIGN;
            }
            break;
        }
        case Phase::HOME_ALIGN:
            if(aligned(VisionStartMode::CORNER))phase=Phase::HOME_RETURN; // 角点对齐完成后收臂并执行最终归位。
            break;
        case Phase::HOME_RETURN: {
            MotionRequest request;
            request.kind=MotionKind::HOME_RETURN;
            request.zone=homeZone;
            if(motion(request)==1) { // 提交并等待最终收臂及归位动作。
                io.clearProgress();
                next(STATE_DONE); // 全部动作结束，切换到完成状态。
                if(io.state()==STATE_DONE)io.runDone(); // 结束正式计时、记录结果并提交电机失能。
            }
            break;
        }
        case Phase::INACTIVE: // 完成、失败等对外状态沿用原处理，不再发起动作。
            break;
        }
    }
};
