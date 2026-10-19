"""实际规划代码的编译期验证：两段选角、限位与三件整组的穷举对照。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TurretPathTests(unittest.TestCase):
    def compile_cases(self, cases, standard="c++14", evaluate=True):
        compiler = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / (
            "packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++.exe")
        self.assertTrue(compiler.is_file(), "需要已安装的 ESP32-S3 编译器")
        if evaluate:
            header = (ROOT / "src/turret_path.h").read_text(encoding="utf-8")
            self.assertEqual(header.count("inline bool PlanTurretPath("), 1)
            # 仅把实际函数改为constexpr，让编译器执行验证；生产代码兼容C++11。
            header = header.replace("#pragma once\n", "").replace(
                "inline bool PlanTurretPath(", "constexpr bool PlanTurretPath(", 1)
        else:
            header = '#include "turret_path.h"\n'
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "turret.cpp"
            fixture.write_text(header + "\n" + cases, encoding="utf-8")
            result = subprocess.run([str(compiler), f"-std={standard}", "-fsyntax-only",
                                     "-I", str(ROOT / "src"), str(fixture)],
                                    capture_output=True, text=True, encoding="utf-8", errors="replace")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_cpp11_public_api(self):
        self.compile_cases('''
bool api() {
    const float targets[] = {300, 48.7f};
    TurretPathPlan plan;
    return PlanTurretPath(123.3f, targets, 2, -150, 360, plan);
}
''', standard="c++11", evaluate=False)

    def test_examples_and_rejected_input_preserves_output(self):
        self.compile_cases(r'''
constexpr bool near(float a, float b) { return a-b < 0.001f && b-a < 0.001f; }
constexpr bool one(float start, float logical, float target, float cost) {
    const float goals[] = {logical}; TurretPathPlan plan;
    return PlanTurretPath(start, goals, 1, -150, 360, plan) && plan.count==1
        && near(plan.anglesDeg[0],target) && near(plan.totalDegrees,cost);
}
constexpr bool examples() {
    const float goals[] = {300,48.7f}; TurretPathPlan plan;
    if (!PlanTurretPath(123.3f,goals,2,-150,360,plan) || plan.count!=2
            || !near(plan.anglesDeg[0],-60) || !near(plan.anglesDeg[1],48.7f)
            || !near(plan.totalDegrees,292)) return false;
    const float repeated[] = {90,90,90};
    return PlanTurretPath(90,repeated,3,-150,360,plan) && plan.totalDegrees==0
        && plan.anglesDeg[0]==90 && plan.anglesDeg[1]==90 && plan.anglesDeg[2]==90;
}
constexpr bool guards() {
    const float good[] = {90}, nan[] = {__builtin_nanf("")}, inf[] = {__builtin_inff()};
    TurretPathPlan plan; plan.count=2; plan.totalDegrees=888; plan.anglesDeg[0]=17;
    if (PlanTurretPath(0,nullptr,1,-150,360,plan) || PlanTurretPath(0,good,0,-150,360,plan)
            || PlanTurretPath(0,good,7,-150,360,plan) || PlanTurretPath(-151,good,1,-150,360,plan)
            || PlanTurretPath(361,good,1,-150,360,plan) || PlanTurretPath(nan[0],good,1,-150,360,plan)
            || PlanTurretPath(inf[0],good,1,-150,360,plan) || PlanTurretPath(0,nan,1,-150,360,plan)
            || PlanTurretPath(0,inf,1,-150,360,plan) || PlanTurretPath(0,good,1,nan[0],360,plan)
            || PlanTurretPath(0,good,1,-150,inf[0],plan) || PlanTurretPath(0,good,1,0,0,plan)
            || PlanTurretPath(0,good,1,360,-150,plan) || PlanTurretPath(0,good,1,-360,360,plan)
            || PlanTurretPath(0,good,1,-20,20,plan)) return false;
    for (size_t i=1;i<TURRET_PATH_MAX_TARGETS;++i) if (plan.anglesDeg[i]!=0) return false;
    return plan.count==2 && plan.totalDegrees==888 && plan.anglesDeg[0]==17;
}
static_assert(examples(), "two-leg planning chooses 292 degrees instead of greedy 428; repeated goals stay still");
static_assert(one(10,350,-10,20), "use the nearest valid equivalent");
static_assert(one(350,10,10,340), "a short arc outside cable limits is forbidden");
static_assert(one(-150,180,180,330), "minimum cable endpoint is included");
static_assert(one(360,0,360,0) && one(0,360,0,0), "zero and 360 retain the correct winding");
static_assert(one(0,359,-1,1), "planned minus-one is a real angle, not a skip command");
static_assert(one(60,240,-120,180), "equal-cost choices deterministically prefer the lower branch");
static_assert(guards(), "invalid input leaves every output field unchanged");
''')

    def test_all_batch_permutations_match_exhaustive_search(self):
        self.compile_cases(r'''
constexpr float distance(float a,float b) { const float d=a-b; return d<0 ? -d : d; }
constexpr bool near(float a,float b) { return distance(a,b)<0.001f; }
constexpr float exhaustive(float start,const float *goals,size_t count) {
    size_t possibilities=1;
    for (size_t i=0;i<count;++i) possibilities*=3;
    float best=FLT_MAX;
    for (size_t code=0;code<possibilities;++code) {
        size_t digits=code; float position=start,total=0; bool valid=true;
        for (size_t i=0;i<count;++i) {
            const float next=goals[i]+360.0f*(int(digits%3)-1); digits/=3;
            if (next < -150 || next > 360) { valid=false; break; }
            total+=distance(position,next); position=next;
        }
        if (valid && total<best) best=total;
    }
    return best;
}
constexpr bool batches() {
    constexpr int orders[][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    constexpr float cargo[]={239.6f,271.2f,300},work[]={48.7f,86.2f,123.3f};
    constexpr float starts[]={-150,90,123.3f,360};
    for (int order=0;order<6;++order) for (int reverse=0;reverse<2;++reverse) {
        float goals[6]={};
        for (int i=0;i<3;++i) {
            goals[2*i]=reverse ? work[orders[order][i]-1] : cargo[i];
            goals[2*i+1]=reverse ? cargo[i] : work[orders[order][i]-1];
        }
        for (float start:starts) {
            TurretPathPlan plan;
            if (!PlanTurretPath(start,goals,6,-150,360,plan) || plan.count!=6
                    || !near(plan.totalDegrees,exhaustive(start,goals,6))) return false;
            float position=start,total=0;
            for (size_t i=0;i<6;++i) {
                const float angle=plan.anglesDeg[i];
                if (angle < -150 || angle > 360 || !(near(angle,goals[i])
                        || near(angle,goals[i]-360) || near(angle,goals[i]+360))) return false;
                total+=distance(position,angle); position=angle;
            }
            if (!near(total,plan.totalDegrees)) return false;
        }
    }
    return true;
}
static_assert(batches(), "48 forward/reverse batches preserve order and match exhaustive global optimum");
''')


if __name__ == "__main__":
    unittest.main()
