"""编译期验证BOOT0消抖、空闲门控及真实Release入口。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class BootReleaseTests(unittest.TestCase):
    def test_button_and_release_entry(self):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file())
        source = (root / "src/serial_commands.inc").read_text(encoding="utf-8")
        polling = source[source.index("void PollDebugBootButton() {"):
                         source.index("static void serialReply(")]
        release = source[source.index("static void serialRelease("):
                         source.index("// 普通 COLOR/GRAB 共用完成路径")]
        actual = (polling + release).replace("static void", "constexpr void").replace(
            "void PollDebugBootButton", "constexpr void PollDebugBootButton")
        harness = r'''
#include <stddef.h>
#include "debug_release_button.h"
constexpr int BOOT0_PIN=0, LOW=0, STATE_WAIT_START=1, pdFALSE=0, pdPASS=1, portMAX_DELAY=1;
constexpr int pdMS_TO_TICKS(int ms) { return ms; }
enum class VisionStartMode { DISC_MATERIAL };
struct SerialFrame {};
constexpr bool parseSerialFrame(char *command, SerialFrame&) {
    const char expected[]="{CMD,SYS,RELEASE}";
    for (unsigned i=0; i<sizeof(expected); ++i) if (command[i]!=expected[i]) return false;
    return true;
}
struct CRGB { enum { Green=1 }; };
void Task_MainStateMachine(void*);
void vHomeTimerCallback(int);
struct Log { constexpr void println(const char*) {} };
struct Led { int updates=0; constexpr void show() { ++updates; } };
struct Run {
    DebugReleaseButton bootReleaseButton;
    int bootReleaseMux=0;
    bool serialDebugMode=true, discMaterialActive=true, taskMotionAborted=true;
    bool firstDiscGrabReady=true, enableRun=true, alignmentEnabled=true;
    int currentState=0, xHomeTimer=0, xTask_MainStateMachine_Handle=0;
    int timerCreates=0, taskCreates=0, timerDeletes=0, replies=0, errors=0;
    bool timerOK=true, taskOK=true;
    int leds[1]={0}, rawLevel=1;
    uint32_t now=0;
    Log Serial;
    Led FastLED;
    constexpr int digitalRead(int pin) { return pin==0 ? rawLevel : -1; }
    constexpr uint32_t millis() { return now; }
    constexpr void portENTER_CRITICAL(int*) {}
    constexpr void portEXIT_CRITICAL(int*) {}
    constexpr void requestVisionStop(VisionStartMode) { discMaterialActive=false; }
    constexpr void setAlignmentEnabled(bool enabled) { alignmentEnabled=enabled; }
    template<class... Args> constexpr int xTimerCreate(Args...) {
        ++timerCreates; return timerOK ? 1 : 0;
    }
    template<class... Args> constexpr int xTaskCreate(Args...) {
        ++taskCreates; return taskOK ? pdPASS : 0;
    }
    constexpr void xTimerDelete(int, int) { ++timerDeletes; }
    constexpr void serialError(const SerialFrame&, const char*) { ++errors; }
    constexpr void serialReply(const SerialFrame&, const char*) { ++replies; }
ACTUAL
    constexpr void sample(bool pressed, uint32_t time) {
        rawLevel=pressed ? 0 : 1; now=time; PollDebugBootButton();
    }
};
constexpr bool policies() {
    DebugReleaseButton b; b.setIdle(true);
    b.update(true, 0); b.update(true, 1000);
    if (b.takeRequest()) return false; // 上电按住不触发。
    b.update(false, 1001); b.update(false, 1041);
    b.update(true, 1050); b.update(false, 1060); b.update(true, 1070);
    b.update(true, 1109); if (b.takeRequest()) return false;
    b.update(true, 1110); if (!b.takeRequest() || b.takeRequest()) return false;
    b.setIdle(true); b.update(true, 9000);
    if (b.takeRequest()) return false; // 长按不重入。
    b.update(false, 9001); b.update(false, 9041);
    b.setIdle(false); b.update(true, 9050);
    b.setIdle(true); b.update(true, 9090);
    if (b.takeRequest()) return false; // 忙碌时开始按下，空闲后仍不触发。
    b.update(false, 9100); b.update(false, 9140);
    b.update(true, 9150); b.update(true, 9190);
    b.setIdle(false); b.setIdle(true);
    if (b.takeRequest()) return false; // 串口新命令取消尚未处理的请求。
    b.update(false, 9200); b.update(false, 9240);
    b.update(true, 0xfffffff0u); b.update(true, 0x17u);
    if (b.takeRequest()) return false;
    b.update(true, 0x18u);
    return b.takeRequest(); // millis回绕后仍能正确消抖。
}
constexpr bool releaseViaButton(int failure) {
    Run r; r.timerOK=failure!=1; r.taskOK=failure!=2;
    r.serialBootReleaseAtIdle(true);
    r.sample(false, 0); r.sample(false, 40);
    r.sample(true, 50); r.sample(true, 90);
    r.serialBootReleaseAtIdle(true);
    if (r.timerCreates!=1 || r.taskCreates!=(failure==1 ? 0 : 1)) return false;
    if (failure) {
        if (!r.serialDebugMode || r.errors!=1 || r.replies || r.xHomeTimer!=0) return false;
        if (r.timerDeletes!=(failure==2 ? 1 : 0)) return false;
    } else {
        if (r.serialDebugMode || r.replies!=1 || r.errors || r.alignmentEnabled
                || r.discMaterialActive || r.enableRun || r.taskMotionAborted
                || r.firstDiscGrabReady || r.currentState!=STATE_WAIT_START
                || r.leds[0]!=CRGB::Green || r.FastLED.updates!=1) return false;
    }
    r.serialBootReleaseAtIdle(true); r.sample(true, 1000); r.serialBootReleaseAtIdle(true);
    if (r.timerCreates!=1) return false;
    r.sample(false, 1010); r.sample(false, 1050);
    r.sample(true, 1060); r.sample(true, 1100); r.serialBootReleaseAtIdle(true);
    return r.timerCreates==(failure ? 2 : 1); // 失败可松开重试，Release中按键无效。
}
constexpr bool busyAndPartialFrame() {
    Run r; r.serialBootSetIdle(true);
    r.sample(false, 0); r.sample(false, 40);
    r.serialBootSetIdle(false); r.sample(true, 50); r.sample(true, 90);
    r.serialBootReleaseAtIdle(true);
    if (r.timerCreates) return false;
    r.sample(false, 100); r.sample(false, 140);
    r.sample(true, 150); r.sample(true, 190);
    r.serialBootReleaseAtIdle(false); // 半帧输入存在，丢弃请求。
    r.serialBootReleaseAtIdle(true);
    return r.timerCreates==0;
}
static_assert(policies(), "debounce, startup hold, busy press, cancellation, rollover");
static_assert(releaseViaButton(0), "button uses actual release flow exactly once without SYS START");
static_assert(releaseViaButton(1), "timer allocation failure remains Debug and allows re-press");
static_assert(releaseViaButton(2), "task allocation failure deletes timer and allows re-press");
static_assert(busyAndPartialFrame(), "busy/partial-frame press must not start a deferred task");
'''.replace("ACTUAL", actual)
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "boot_release.cpp"
            fixture.write_text(harness, encoding="utf-8")
            result = subprocess.run([str(compiler), "-std=c++14", "-fsyntax-only",
                                     "-I", str(root / "src"), str(fixture)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
