"""编译期执行固件实际第二轮/返家分支及串口抓取完成逻辑。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class RemainingFlowTests(unittest.TestCase):
    def test_round_two_and_return_home(self):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file())
        main = (root / "src/main.cpp").read_text(encoding="utf-8")
        serial = (root / "src/serial_commands.inc").read_text(encoding="utf-8")
        branches = main[main.index("        case STATE_GRAB_ROUND2:"):
                        main.index("        case STATE_DONE:")]
        declarations = main[main.index("    bool disc2RouteCompleted"):
                            main.index("\n\n    while (1)")]
        grab = serial[serial.index("static void serialExecuteTaskGrab("):
                      serial.index("static void serialDiscColorDecision(")]
        runtime = (root / "src/robot_runtime.cpp").read_text(encoding="utf-8")
        timer = runtime[runtime.index("void vHomeTimerCallback("):
                        runtime.index("// @brief 等待扫码")]
        header = (root / "src/robot_runtime.h").read_text(encoding="utf-8")
        start_heading = next(line for line in header.splitlines()
                             if line.startswith("constexpr float START_ZONE_HEADING ="))
        harness = start_heading + r'''
#include <stddef.h>
#include "disc_material_decision.h"
enum RobotState { STATE_GRAB_ROUND1, STATE_GRAB_ROUND2, STATE_PLACE_COARSE2,
    STATE_STACK_TEMP2, STATE_RETURN_HOME, STATE_DONE, STATE_ALIGN_FAILED,
    STATE_TRANSFER_FAILED, STATE_ROUTE_FAILED, STATE_TIMEOUT_FAILED };
enum StartZone { START_ZONE_UNKNOWN, START_ZONE_1, START_ZONE_2 };
enum class AutoAlignmentState { WAITING, DONE, FAILED, IDLE };
enum class VisionStartMode { NONE, DISC, DISC_MATERIAL, WORK_AREA, WORK_AREA_LOADED, CORNER };
constexpr int TEMP_AREA_NODE=22, DISC_AREA_NODE=14, COARSE_AREA_NODE=10;
constexpr int HOME_ZONE1_NODE=4, HOME_ZONE2_NODE=0, portTICK_PERIOD_MS=1;
constexpr int pdMS_TO_TICKS(int ms) { return ms; }
struct Log {
    template<class... A> constexpr void printf(const char*, A...) {}
    constexpr void println(const char*) {}
};
struct Task { int round2_colors[3]={6,5,4}; int round2_pos[3]={3,1,2}; };
struct Run {
    RobotState currentState=STATE_GRAB_ROUND2;
    Task currentTask;
    Log Serial;
    bool serialDebugMode=false, firstDiscGrabReady=false, discMessageResumeRequested=false;
    int roundProgress=0, discMaterialColor=0, xHomeTimer=1, xVisualTaskQueue=1;
    int routes=0, looks=0, armStarts=0, grabs=0, retrieves=0, coarsePlaces=0, stacks=0, timerStops=0;
    bool valid=true, pid=false, material=false;
    bool routeOK=true, lookOK=true, grabOK=true, transferOK=true;
    int armFailAt=0;
    VisionStartMode camera=VisionStartMode::NONE;
    AutoAlignmentState alignment=AutoAlignmentState::WAITING;
    DECLARATIONS
    constexpr void updateDisplay(const char*, const char*, const char*) {}
    constexpr void xQueueReset(int) {}
    int retrievalWaits=0;
    bool interruptWait=false;
    constexpr void vTaskDelay(int ms) {
        if (currentState==STATE_STACK_TEMP2 && !temp2CargoRetrieved) {
            valid=valid && !pid && camera==VisionStartMode::NONE && ms==100;
            ++retrievalWaits;
            if (interruptWait) { currentState=STATE_RETURN_HOME; taskMotionAborted=true; }
        }
    }
    constexpr void xTimerStop(int, int) { ++timerStops; }
    constexpr AutoAlignmentState getAutoAlignmentState() { return alignment; }
    constexpr void setAlignmentEnabled(bool enabled) { pid=enabled; }
    constexpr bool InitArm_look2() { ++looks; return lookOK; }
    constexpr bool InitArm_start() { ++armStarts; return armStarts != armFailAt; }
    bool taskMotionAborted = false;
    int workLooks = 0;
    bool workLookOK = true;
    constexpr bool InitArm_look() { valid = valid && !pid; ++workLooks; return workLookOK; }
    constexpr void requestVisionStart(VisionStartMode mode) {
        valid = valid && camera == VisionStartMode::NONE;
        if (currentState==STATE_STACK_TEMP2) valid=valid && mode==VisionStartMode::WORK_AREA_LOADED;
        if (currentState==STATE_PLACE_COARSE2) valid=valid && mode==VisionStartMode::WORK_AREA;
        if (mode==VisionStartMode::DISC_MATERIAL) { material=true; valid=valid && !pid; }
        else { camera=mode; pid=true; alignment=AutoAlignmentState::WAITING; }
    }
    constexpr void requestVisionStop(VisionStartMode mode) {
        if (mode==VisionStartMode::DISC_MATERIAL) material=false;
        else { valid=valid && camera==mode; camera=VisionStartMode::NONE; pid=false; }
    }
    constexpr bool requestAndMoveNodePath(int from, int to, float heading=-999) {
        valid=valid && !pid && !material && camera==VisionStartMode::NONE;
        if (currentState==STATE_GRAB_ROUND2) valid=valid && from==22 && to==14;
        if (currentState==STATE_PLACE_COARSE2) valid=valid && from==14 && to==10;
        if (currentState==STATE_STACK_TEMP2) valid=valid && from==10 && to==22;
        if (currentState==STATE_RETURN_HOME) valid=valid && from==22
            && to==(homeStartZone==START_ZONE_1 ? 4 : 0)
            && heading==180;
        ++routes; return routeOK;
    }
    constexpr bool GrabDiscMaterial(uint8_t color, uint8_t cargo) {
        valid=valid && material && !pid && !firstDiscGrabReady
            && color==currentTask.round2_colors[roundProgress] && cargo==roundProgress+1;
        ++grabs; return grabOK;
    }
    constexpr bool RetrieveRoundToCargo(const int* colors, const int* positions) {
        valid=valid && !pid && camera==VisionStartMode::NONE && retrievalWaits==1
            && colors==currentTask.round2_colors && positions==currentTask.round2_pos;
        ++retrieves; return transferOK;
    }
    constexpr bool PlaceTaskCargoToWorkArea(const int* positions, int layer) {
        valid=valid && !pid && camera==VisionStartMode::NONE && positions==currentTask.round2_pos;
        if (currentState==STATE_PLACE_COARSE2) { valid=valid && layer==1; ++coarsePlaces; }
        else { valid=valid && layer==2; ++stacks; }
        return transferOK;
    }
    GRAB_LOGIC
    constexpr void tick() { switch(currentState) { BRANCHES default: break; } }
};
constexpr bool fullRound(StartZone zone) {
    Run r; r.homeStartZone=zone;
    r.tick(); r.tick();
    if (r.routes!=1 || r.grabs || r.firstDiscGrabReady) return false;
    r.alignment=AutoAlignmentState::DONE; r.tick();
    for (int i=0; i<3; ++i) {
        if (!r.firstDiscGrabReady || !r.material) return false;
        r.serialExecuteTaskGrab(r.currentTask.round2_colors[i], i+1);
        if (r.firstDiscGrabReady) return false;
        r.tick();
        if (i<2) {
            if (!r.discMessageResumeRequested || r.disc2PreparedProgress!=i+1) return false;
            r.discMessageResumeRequested=false; r.firstDiscGrabReady=true;
        }
    }
    if (r.currentState!=STATE_PLACE_COARSE2 || r.material || r.grabs!=3) return false;
    r.tick(); r.tick();
    if (r.coarsePlaces) return false;
    r.alignment=AutoAlignmentState::DONE; r.tick();
    if (r.currentState!=STATE_STACK_TEMP2 || r.retrieves) return false;
    r.alignment=AutoAlignmentState::IDLE; r.tick(); r.tick(); r.tick();
    if (r.stacks || r.camera!=VisionStartMode::WORK_AREA_LOADED || !r.pid) return false;
    r.tick();
    if (r.stacks || r.camera!=VisionStartMode::WORK_AREA_LOADED || !r.pid) return false;
    r.alignment=AutoAlignmentState::DONE; r.tick();
    if (r.currentState!=STATE_RETURN_HOME || r.stacks!=1) return false;
    r.tick(); r.tick();
    if (r.currentState==STATE_DONE || r.armStarts!=1) return false;
    r.alignment=AutoAlignmentState::DONE; r.tick(); r.tick();
    return r.valid && r.currentState==STATE_DONE && !r.pid && !r.material
        && r.camera==VisionStartMode::NONE && r.routes==4 && r.coarsePlaces==1
        && r.retrieves==1 && r.stacks==1 && r.armStarts==2 && r.timerStops==1 && r.workLooks==2 && r.retrievalWaits==1;
}
constexpr bool grabFailure() {
    Run r; r.material=true; r.firstDiscGrabReady=true; r.grabOK=false;
    r.serialExecuteTaskGrab(6,1); r.tick();
    return r.valid && r.currentState==STATE_TRANSFER_FAILED && !r.material
        && !r.firstDiscGrabReady && r.roundProgress==0 && r.timerStops==1 && r.grabs==1;
}
constexpr bool failures(int kind) {
    Run r; r.homeStartZone=START_ZONE_1;
    if (kind<=3) {
        r.currentState=STATE_PLACE_COARSE2;
        if (kind==0) r.routeOK=false;
        r.tick();
        if (kind==1 || kind==2) r.alignment=kind==1 ? AutoAlignmentState::FAILED : AutoAlignmentState::IDLE;
        if (kind==3) { r.alignment=AutoAlignmentState::DONE; r.transferOK=false; }
        if (kind!=0) r.tick();
    } else if (kind<=7) {
        r.currentState=STATE_STACK_TEMP2; r.camera=VisionStartMode::NONE; r.pid=false;
        r.alignment=kind==4 ? AutoAlignmentState::FAILED : AutoAlignmentState::DONE;
        if (kind==4 || kind==5) r.transferOK=false;
        r.tick();
        if (kind>=6) { r.routeOK=kind!=6; r.tick(); }
        if (kind==7) { r.alignment=AutoAlignmentState::DONE; r.transferOK=false; r.tick(); }
    } else {
        r.currentState=STATE_RETURN_HOME;
        if (kind==8) r.homeStartZone=START_ZONE_UNKNOWN;
        if (kind==9) r.armFailAt=1;
        if (kind==10) r.routeOK=false;
        r.tick();
        if (kind==11) { r.alignment=AutoAlignmentState::FAILED; r.tick(); }
        if (kind==12) { r.alignment=AutoAlignmentState::DONE; r.armFailAt=2; r.tick(); }
    }
    auto expected=(kind==0 || kind==6 || kind==8 || kind==10) ? STATE_ROUTE_FAILED
        : (kind==1 || kind==2 || kind==11) ? STATE_ALIGN_FAILED : STATE_TRANSFER_FAILED;
    r.tick();
    return r.valid && r.currentState==expected && r.timerStops==1 && !r.pid
        && !r.material && r.camera==VisionStartMode::NONE;
}
constexpr bool allFailures() { for(int i=0;i<=12;++i) if(!failures(i)) return false; return true; }
constexpr bool workLookFailures() {
    Run empty;
    empty.currentState = STATE_PLACE_COARSE2;
    empty.workLookOK = false;
    empty.tick();
    return empty.valid && empty.currentState == STATE_TRANSFER_FAILED
        && empty.camera == VisionStartMode::NONE && empty.workLooks == 1
        && empty.timerStops == 1 && empty.coarsePlaces == 0;
}
constexpr bool retrievalWaitInterrupted() {
    Run r; r.currentState=STATE_STACK_TEMP2; r.interruptWait=true; r.tick();
    return r.valid && r.retrievalWaits==1 && r.retrieves==0 && r.routes==0
        && r.currentState==STATE_RETURN_HOME && r.taskMotionAborted;
}
constexpr bool loadedAlignmentFailure() {
    Run r; r.currentState=STATE_STACK_TEMP2;
    r.tick(); r.tick();
    if (r.camera!=VisionStartMode::WORK_AREA_LOADED || !r.pid || r.stacks) return false;
    r.alignment=AutoAlignmentState::FAILED;
    r.tick(); r.tick();
    return r.valid && r.currentState==STATE_ALIGN_FAILED && r.stacks==0
        && r.camera==VisionStartMode::NONE && !r.pid && r.timerStops==1;
}
static_assert(loadedAlignmentFailure(), "loaded alignment failure stops matching mode and prevents stacking");
static_assert(workLookFailures(), "look failure prevents empty work-area vision");
static_assert(retrievalWaitInterrupted(), "interruption during settling wait prevents retrieval");
static_assert(fullRound(START_ZONE_1), "right home: complete ordered round without repeated actions");
static_assert(fullRound(START_ZONE_2), "left home: preserve zone and restore initial heading");
static_assert(grabFailure(), "round two grab failure stops recognition and progress");
static_assert(allFailures(), "route/alignment/transfer/home reset failures cannot advance");
using TimerHandle_t = int;
enum class NodePathState { WAITING, FAILED };
struct TimeoutRun {
    RobotState currentState=STATE_GRAB_ROUND2;
    bool taskMotionAborted=false, firstDiscGrabReady=true, pid=true;
    NodePathState nodePathState=NodePathState::WAITING;
    int nodePathMux=1, xTask_MainStateMachine_Handle=1;
    int modesStopped=0, suspended=0, stoppedMotors=0;
    Log Serial;
    constexpr void requestVisionStop(VisionStartMode) { ++modesStopped; pid=false; }
    constexpr void setAlignmentEnabled(bool enabled) { pid=enabled; }
    constexpr void portENTER_CRITICAL(int*) {}
    constexpr void portEXIT_CRITICAL(int*) {}
    constexpr void vTaskSuspend(int) {
        ++suspended;
        // 模拟主任务在暂停前因 MoveArm 中止而覆盖故障状态。
        currentState=STATE_TRANSFER_FAILED;
    }
    constexpr void Emm_V5_Stop_Now(uint8_t motor, bool sync) {
        stoppedMotors |= 1 << (motor-1);
    }
    constexpr void updateDisplay(const char*, const char*, const char*) {}
    TIMER_LOGIC
};
constexpr bool timeoutStops() {
    TimeoutRun r; r.vHomeTimerCallback(1);
    if (!r.taskMotionAborted || r.firstDiscGrabReady || r.pid || r.modesStopped!=5
        || r.suspended!=1 || r.stoppedMotors!=63 || r.nodePathState!=NodePathState::FAILED
        || r.currentState!=STATE_TIMEOUT_FAILED) return false;
    TimeoutRun done; done.currentState=STATE_DONE; done.vHomeTimerCallback(1);
    return !done.taskMotionAborted && done.suspended==0 && done.modesStopped==0;
}
static_assert(timeoutStops(), "timeout latches abort, stops six motors and five modes, preserves DONE");
'''
        harness = harness.replace("DECLARATIONS", declarations).replace("BRANCHES", branches).replace(
            "GRAB_LOGIC", grab.replace("static void", "constexpr void", 1)).replace(
            "TIMER_LOGIC", timer.replace("void vHomeTimerCallback", "constexpr void vHomeTimerCallback", 1))
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "remaining_flow.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only",
                                     "-I", str(root / "src"), str(fixture)], capture_output=True,
                                    text=True, encoding="utf-8", errors="replace")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
