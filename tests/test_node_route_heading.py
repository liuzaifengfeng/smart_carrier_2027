"""以编译期常量求值验证实际 C++ 路段朝向规划，无需实车。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class NodeRouteHeadingTests(unittest.TestCase):
    def test_final_heading_forward_backward_and_perpendicular(self):
        root = Path(__file__).resolve().parents[1]
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        cases = r'''
#include "node_route_heading.h"
constexpr auto forward = PlanNodeSegmentHeading(0, 0, true, 0);
static_assert(!forward.reverse && forward.turn == 0 && forward.finishTurn == 0, "forward arrival");
constexpr auto backward = PlanNodeSegmentHeading(0, 180, true, 0);
static_assert(backward.reverse && backward.heading == 0 && backward.turn == 0
              && backward.finishTurn == 0, "backward arrival keeps car heading");
constexpr auto disc = PlanNodeSegmentHeading(90, 270, true, 90);
static_assert(disc.reverse && disc.heading == 90 && disc.finishTurn == 0, "disc arrival");
constexpr auto coarse = PlanNodeSegmentHeading(270, 90, true, 270);
static_assert(coarse.reverse && coarse.heading == 270 && coarse.finishTurn == 0, "coarse arrival");
constexpr auto temporary = PlanNodeSegmentHeading(180, 0, true, 180);
static_assert(temporary.reverse && temporary.heading == 180 && temporary.finishTurn == 0, "temporary arrival");
constexpr auto perpendicular = PlanNodeSegmentHeading(180, 90, true, 0);
static_assert(!perpendicular.reverse && perpendicular.turn == -90
              && perpendicular.finishTurn == -90, "perpendicular arrival");
constexpr auto preTurn = PlanNodeSegmentHeading(180, 0, true, 0);
static_assert(preTurn.turn == -180 && preTurn.finishTurn == 0, "required preturn must not be skipped");
constexpr auto tie = PlanNodeSegmentHeading(90, 0, false, 0);
static_assert(!tie.reverse && tie.turn == -90, "equal turns prefer forward");
constexpr auto freeReverse = PlanNodeSegmentHeading(0, 180, false, 0);
static_assert(freeReverse.reverse && freeReverse.turn == 0, "unconstrained behavior retained");
#if __cplusplus >= 201402L
constexpr bool sweep() {
    const float finals[] = {90, 270, 180};
    for (int current = 0; current < 360; ++current) {
        for (int travel = 0; travel < 360; travel += 90) {
            auto free = PlanNodeSegmentHeading(current, travel, false, 0);
            if (NodeRouteAbs(free.turn) > 90 || free.finishTurn != 0) return false;
            for (float final : finals) {
                auto plan = PlanNodeSegmentHeading(current, travel, true, final);
                float finish = NodeRouteAbs(plan.finishTurn);
                if (finish != 0 && finish != 90) return false;
                if (plan.heading != (plan.reverse ? NodeRouteBackward(travel) : travel)) return false;
                if (plan.finishTurn != NodeRouteTurn(plan.heading, final)) return false;
                if (NodeRouteTurn(plan.heading, travel) == 0 && plan.reverse) return false;
            }
        }
    }
    return true;
}
static_assert(sweep(), "all initial integer headings and cardinal route/final combinations");
#endif
'''
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "heading.cpp"
            fixture.write_text(cases, encoding="utf-8")
            for standard in ("c++11", "c++14"):
                with self.subTest(standard=standard):
                    result = subprocess.run([str(compiler), f"-std={standard}", "-fsyntax-only",
                                             "-I", str(root / "src"), str(fixture)],
                                            capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
