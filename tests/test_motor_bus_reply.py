"""实际总线应答处理：地址匹配、错误原帧日志与请求功能码。"""
import unittest
from native_support import ROOT, build_native, run_native
from test_servo_multiturn_interval import function_source


class MotorBusReplyTests(unittest.TestCase):
    def test_actual_reply(self):
        source = (ROOT / 'src/node_position_hardware.cpp').read_text(encoding='utf-8')
        reply = function_source(source, 'const char *reply(') + '\n' + function_source(source, 'const char *readEncoderSnapshot(') + '\n' + function_source(source, 'const char *readPositionPid(') + '\n' + function_source(source, 'const char *writePositionPid(')
        harness = r'''
#include "Arduino.h"
#include "emm_feedback.h"
#include <assert.h>
#include <vector>
NativeSerial Serial;
uint32_t timeMs=0;
uint32_t millis(){return timeMs;}
void vTaskDelay(uint32_t ms){timeMs+=ms;}
struct Port {
    std::vector<uint8_t> input,pending,sent;size_t index=0;
    int available(){return index<input.size();}
    int read(){return input[index++];}
    void write(const uint8_t *data,size_t n){sent.assign(data,data+n);input=pending;index=0;}
    void flush(){}
} Serial1;
struct MotorBusGuard {};
struct Hardware {
    void drain(){while(Serial1.available())Serial1.read();}
    bool stopped=false;
    bool cancelled(){return stopped;}
    ACTUAL
};
int main(){
    Hardware h;EmmReply out;
    Serial1.input={6,0,0xEE,0x6B,7,0x36,0,0,1,0,0,0x6B};
    assert(!h.reply(7,0x36,out)&&EmmFeedbackParser::position(out)==65536);
    assert(Serial.text.empty());
    Serial1={};Serial1.input={7,0,0xEE,0x6B};
    assert(!strcmp(h.reply(7,0x36,out),"MOTOR_COMMAND_ERROR"));
    assert(Serial.text=="[MotorBus] id=7 request=0x36 RX=07 00 EE 6B\n");
    assert(timeMs==25); // 仅使用既有响应窗口，错误不触发重试或延长超时。
    Serial1={};Serial.text.clear();
    Serial1.input={7,0,0xEE,0x6B,7,0x36,0,0,1,0,0,0x6B};
    assert(!h.reply(7,0x36,out)&&EmmFeedbackParser::position(out)==65536);
    assert(Serial.text.find("matched after unassigned error")!=std::string::npos);
    Serial1={};Serial1.input={7,0,0xEE,0x6B,7,0xFD,2,0x6B};
    assert(!strcmp(h.reply(7,0x36,out),"MOTOR_COMMAND_ERROR")); // 非位置应答不能冒充查询成功。
    Serial1={};Serial1.input={7,0xFD,2,0x6B,7,0x36,1,0,0,0x80,0,0x6B};
    assert(!h.reply(7,0x36,out)&&EmmFeedbackParser::position(out)==-32768);
    Serial1={};
    assert(!strcmp(h.reply(7,0x36,out),"MOTOR_FEEDBACK_TIMEOUT"));
    Serial1={};Serial1.pending={7,0x43,31,9,0x5C,0x67,0,3,0x43,0xEB,1,0,1,0,0,0,0,0,1,0,1,0,0,1,0,0,0,8,3,3,0x6B};
    Serial1.pending[15]=1;Serial1.pending[17]=42;Serial1.pending[28]=1;Serial1.pending[29]=0x83;
    uint16_t encoder=0;int64_t position=0;int16_t rpm=0;uint8_t flags=0;
    assert(!h.readEncoderSnapshot(7,encoder,position,rpm,flags)&&encoder==17387&&position==-65536&&rpm==-42&&flags==0x83);
    assert(Serial1.sent==std::vector<uint8_t>({7,0x43,0x7A,0x6B}));
    Serial1.pending={};encoder=123;position=456;
    assert(h.readEncoderSnapshot(7,encoder,position,rpm,flags)&&encoder==123&&position==456);
    Serial1={};Serial1.pending={7,0x21,0,0,0xF2,0x30,0,0,0,0x64,0,0,0xF2,0x30,0x6B};
    uint32_t gains[3]={};assert(!h.readPositionPid(7,gains)&&gains[0]==62000&&gains[1]==100&&gains[2]==62000);
    assert(Serial1.sent==std::vector<uint8_t>({7,0x21,0x6B}));
    gains[2]=93000;Serial1.pending={7,0x4A,2,0x6B};
    assert(!h.writePositionPid(7,gains));
    assert(Serial1.sent==std::vector<uint8_t>({7,0x4A,0xC3,0,0,0,0xF2,0x30,0,0,0,0x64,0,1,0x6B,0x48,0x6B}));
    Serial1.pending={7,0x4A,0xE2,0x6B};assert(!strcmp(h.writePositionPid(7,gains),"MOTOR_COMMAND_REJECTED"));
    Serial.text.clear();Serial1.pending={};
    assert(!strcmp(h.writePositionPid(7,gains),"MOTOR_FEEDBACK_TIMEOUT"));
    assert(Serial.text.find("request=0x4A timeout=25ms RX=<none>")!=std::string::npos);
    assert(Serial.text.find("PID write error=MOTOR_FEEDBACK_TIMEOUT TX=07 4A C3 00")!=std::string::npos);
    Serial.text.clear();Serial1.pending={7,0x4A,2};
    assert(!strcmp(h.writePositionPid(7,gains),"MOTOR_FEEDBACK_TIMEOUT"));
    assert(Serial.text.find("RX=07 4A 02")!=std::string::npos);
    Serial1.pending={7,0x4A,2,0x6B};
    assert(!h.writePositionPid(7,gains,true));assert(Serial1.sent[3]==1);
    h.stopped=true;
    assert(!strcmp(h.reply(7,0x36,out),"STOPPED"));
}
'''.replace('ACTUAL', reply).replace('#include <vector>', '#include <vector>\n#include <string.h>')
        cache = ROOT / '.pio/motion-native'
        cache.mkdir(parents=True, exist_ok=True)
        path = cache / 'motor_bus_reply_actual.cpp'
        path.write_text(harness, encoding='utf-8')
        run_native(build_native('motor-bus-reply', [str(path)]))
