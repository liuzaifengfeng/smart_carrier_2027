"""编译期执行实际 TEMP1 状态分支，验证步骤顺序、失败隔离及单次动作。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class PlaceTemp1Tests(unittest.TestCase):
    def test_actual_state_flow(self):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        source = (root / "src/main.cpp").read_text(encoding="utf-8")
        start = source.index("        case STATE_PLACE_TEMP1:")
        end = source.index("        case STATE_GRAB_ROUND2:", start)
        declarations = source[source.index("    bool temp1CargoRetrieved"):
                              source.index("    bool disc2RouteCompleted")]
        harness = r'''
#include <stddef.h>
enum State { STATE_PLACE_TEMP1, STATE_GRAB_ROUND2, STATE_ALIGN_FAILED,
             STATE_ROUTE_FAILED, STATE_TRANSFER_FAILED, STATE_RETURN_HOME };
enum class AutoAlignmentState { WAITING, DONE, FAILED, IDLE };
enum class VisionStartMode { WORK_AREA_LOADED, WORK_AREA };
constexpr int COARSE_AREA_NODE = 10, TEMP_AREA_NODE = 22;
constexpr int pdMS_TO_TICKS(int ms) { return ms; }
struct Task { int round1_colors[3] = {6, 2, 4}; int round1_pos[3] = {3, 1, 2}; };
struct Run {
    State currentState = STATE_PLACE_TEMP1;
    AutoAlignmentState alignment = AutoAlignmentState::WAITING;
    Task currentTask;
    int roundProgress = 3;
    int xHomeTimer = 1;
    bool pid = false, loaded = false, empty = false;
    bool retrieveOK = true, routeOK = true, placeOK = true;
    int interruptAt = 0;
    int retrieved = 0, routed = 0, placed = 0, timerStops = 0;
    bool valid = true;
    DECLARATIONS
    constexpr void updateDisplay(const char*, const char*, const char*) {}
    constexpr AutoAlignmentState getAutoAlignmentState() { return alignment; }
    constexpr void setAlignmentEnabled(bool enabled) { pid = enabled; }
    constexpr void xTimerStop(int, int) { ++timerStops; }
    int retrievalWaits = 0;
    constexpr void vTaskDelay(int ms) {
        valid = valid && !pid && !loaded && !retrieved && ms == 1000;
        ++retrievalWaits;
        if (interruptAt == 4) currentState = STATE_RETURN_HOME;
        if (interruptAt == 5) taskMotionAborted = true;
    }
    constexpr void requestVisionStop(VisionStartMode mode) {
        pid = false;
        if (mode == VisionStartMode::WORK_AREA_LOADED) loaded = false;
        else empty = false;
    }
    bool taskMotionAborted = false;
    int workLooks = 0;
    bool workLookOK = true;
    constexpr bool InitArm_look() { valid = valid && !pid; ++workLooks; return workLookOK; }
    constexpr void requestVisionStart(VisionStartMode mode) {
        valid = valid && mode == VisionStartMode::WORK_AREA && routed == 1 && !loaded;
        valid = valid && workLooks == 1;
        empty = true; pid = true; alignment = AutoAlignmentState::WAITING;
    }
    constexpr bool RetrieveRoundToCargo(const int* colors, const int* positions) {
        valid = valid && !pid && !loaded && retrievalWaits == 1 && !routed && !placed
                && colors == currentTask.round1_colors && positions == currentTask.round1_pos;
        ++retrieved;
        if (interruptAt == 1) currentState = STATE_RETURN_HOME;
        return retrieveOK;
    }
    constexpr bool requestAndMoveNodePath(int from, int to) {
        valid = valid && !pid && !loaded && retrieved == 1 && !placed
                && from == 10 && to == 22;
        ++routed;
        if (interruptAt == 2) currentState = STATE_RETURN_HOME;
        return routeOK;
    }
    constexpr bool PlaceTaskCargoToWorkArea(const int* positions, int layer) {
        valid = valid && !pid && !empty && !loaded && retrieved == 1 && routed == 1
                && positions == currentTask.round1_pos && layer == 1;
        ++placed;
        if (interruptAt == 3) currentState = STATE_RETURN_HOME;
        return placeOK;
    }
    constexpr void tick() {
        switch (currentState) {
            ACTUAL_BRANCH
            default: break;
        }
    }
};
constexpr bool success() {
    Run r;
    r.tick(); r.tick();
    for (int i = 0; i < 4; ++i) r.tick();
    if (r.retrieved != 1 || r.routed != 1 || r.placed || !r.empty || !r.pid) return false;
    r.alignment = AutoAlignmentState::DONE;
    r.tick(); r.tick();
    return r.valid && r.retrieved == 1 && r.routed == 1 && r.placed == 1
            && !r.pid && !r.empty && !r.loaded && !r.timerStops
            && !r.roundProgress && r.retrievalWaits == 1 && r.currentState == STATE_GRAB_ROUND2;
}
constexpr bool failure(int phase, AutoAlignmentState state) {
    Run r;
    r.alignment = AutoAlignmentState::DONE;
    if (phase == 0) r.alignment = state;
    if (phase == 1) r.retrieveOK = false;
    if (phase == 2) r.routeOK = false;
    if (phase == 4) r.placeOK = false;
    r.tick(); r.tick();
    if (phase >= 3) { r.alignment = phase == 3 ? state : AutoAlignmentState::DONE; r.tick(); }
    r.tick();
    State expected = phase == 0 || phase == 3 ? STATE_ALIGN_FAILED
                     : phase == 2 ? STATE_ROUTE_FAILED : STATE_TRANSFER_FAILED;
    return r.valid && r.currentState == expected && r.timerStops == 1 && !r.pid
            && !r.loaded && !r.empty && r.retrieved == (phase == 0 ? 0 : 1)
            && r.routed == (phase < 2 ? 0 : 1) && r.placed == (phase == 4 ? 1 : 0);
}
constexpr bool interrupted(int action) {
    Run r;
    r.interruptAt = action; r.alignment = AutoAlignmentState::DONE;
    r.tick(); r.tick();
    r.alignment = AutoAlignmentState::DONE; r.tick(); r.tick();
    return r.valid && r.currentState == STATE_RETURN_HOME && !r.timerStops
            && r.retrieved == 1 && r.routed == (action >= 2 ? 1 : 0)
            && r.placed == (action == 3 ? 1 : 0) && !r.empty && !r.loaded;
}
constexpr bool lookFailure() {
    Run r;
    r.alignment = AutoAlignmentState::DONE;
    r.tick();
    r.workLookOK = false;
    r.tick();
    return r.valid && r.currentState == STATE_TRANSFER_FAILED && r.workLooks == 1
        && !r.empty && !r.pid && !r.placed && r.timerStops == 1;
}
static_assert(lookFailure(), "failed look blocks temporary-area vision and placement");
static_assert(success(), "ordered flow, waits, task mapping, no duplicate actions");
constexpr bool skipsLoadedAlignment(AutoAlignmentState state) {
    Run r; r.alignment = state; r.tick();
    return r.valid && r.retrieved == 1 && r.retrievalWaits == 1 && !r.pid && !r.loaded;
}
constexpr bool interruptedDuringWait(int action) {
    Run r; r.interruptAt = action; r.tick();
    return r.valid && r.retrieved == 0 && r.routed == 0 && r.retrievalWaits == 1
        && (action == 4 ? r.currentState == STATE_RETURN_HOME : r.taskMotionAborted);
}
static_assert(skipsLoadedAlignment(AutoAlignmentState::WAITING)
    && skipsLoadedAlignment(AutoAlignmentState::FAILED)
    && skipsLoadedAlignment(AutoAlignmentState::IDLE), "retrieval does not wait for loaded alignment");
static_assert(interruptedDuringWait(4) && interruptedDuringWait(5), "abort during wait prevents retrieval");
static_assert(failure(1, AutoAlignmentState::DONE), "retrieval failure blocks route");
static_assert(failure(2, AutoAlignmentState::DONE), "route failure blocks vision/place");
static_assert(failure(3, AutoAlignmentState::FAILED), "temp alignment failure");
static_assert(failure(3, AutoAlignmentState::IDLE), "temp alignment cancelled");
static_assert(failure(4, AutoAlignmentState::DONE), "placement failure blocks next round");
static_assert(interrupted(1) && interrupted(2) && interrupted(3), "preserve external state changes");
'''
        harness = harness.replace("DECLARATIONS", declarations).replace(
            "ACTUAL_BRANCH", source[start:end])
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "place_temp1.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only", str(fixture)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
