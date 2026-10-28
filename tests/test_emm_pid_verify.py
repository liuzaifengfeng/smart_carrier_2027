"""写入无 ACK 时仍须以实际回读一致确认；不重发写入。"""
import unittest
from native_support import ROOT, build_native, run_native
from test_servo_multiturn_interval import function_source


class EmmPidVerifyTests(unittest.TestCase):
    def test_transaction(self):
        function = function_source((ROOT / 'src/node_position_hardware.cpp').read_text(encoding='utf-8'),
                                   'const char *SetMotorPositionPidVerified(')
        source = r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
struct MotorBusGuard {};
const char *writeError=nullptr,*readError=nullptr;
int writes=0,reads=0;bool saved=false;uint32_t actual[3]={10,20,30};
const char *WriteMotorPositionPid(uint8_t id,const uint32_t (&values)[3],bool save){
    assert(id==7);++writes;saved=save;return writeError;
}
const char *ReadMotorPositionPid(uint8_t id,uint32_t (&values)[3]){
    assert(id==7);++reads;for(int i=0;i<3;++i)values[i]=actual[i];return readError;
}
FUNCTION
int main(){
    uint32_t target[3]={10,20,30};
    assert(!SetMotorPositionPidVerified(7,target,false));assert(writes==1&&reads==1);
    writeError="MOTOR_FEEDBACK_TIMEOUT";
    assert(!SetMotorPositionPidVerified(7,target,true));assert(writes==2&&reads==2&&saved);
    actual[1]=21;
    assert(!strcmp(SetMotorPositionPidVerified(7,target,false),"PID_VERIFY_MISMATCH"));
    readError="MOTOR_FEEDBACK_TIMEOUT";
    assert(!strcmp(SetMotorPositionPidVerified(7,target,false),readError));
    writeError="MOTOR_COMMAND_REJECTED";int before=reads;
    assert(!strcmp(SetMotorPositionPidVerified(7,target,false),writeError));assert(reads==before);
    assert(writes==5); // 每次操作只写一次
}
'''.replace('FUNCTION', function)
        path = ROOT / '.pio/motion-native/emm_pid_verify.cpp'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source, encoding='utf-8')
        run_native(build_native('emm-pid-verify', [str(path)]))
