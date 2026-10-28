"""实际0x42/0x48链路：33字节解析、7字段替换和配置保留。"""
import unittest
from native_support import ROOT, build_native, run_native
from test_servo_multiturn_interval import function_source


class DriveConfigTests(unittest.TestCase):
    def test_actual_transaction(self):
        source = (ROOT / 'src/node_position_hardware.cpp').read_text(encoding='utf-8')
        methods = '\n'.join(function_source(source, name) for name in (
            'const char *reply(', 'const char *readDriveConfig(', 'const char *writeDriveConfig('))
        wrappers = '\n'.join(function_source(source, name) for name in (
            'const char *ReadMotorDriveParams(', 'const char *SetMotorDriveParamsVerified('))
        harness = r'''
#include "Arduino.h"
#include "emm_feedback.h"
#include "emm_drive_config.h"
#include <assert.h>
#include <string.h>
#include <vector>
#include <deque>
NativeSerial Serial;
uint32_t clockMs=0;int depth=0,gaps=0,flashGaps=0;
uint32_t millis(){return clockMs;}
void vTaskDelay(uint32_t ms){assert(depth>0);assert(ms==1||ms==5||ms==100);if(ms==5||ms==100)++gaps;if(ms==100)++flashGaps;clockMs+=ms;}
struct MotorBusGuard{MotorBusGuard(){++depth;}~MotorBusGuard(){--depth;}};
struct Port {
    std::deque<std::vector<uint8_t>> replies;
    std::vector<std::vector<uint8_t>> sent;
    std::vector<uint8_t> input;size_t pos=0;
    int available(){return pos<input.size();}
    uint8_t read(){return input[pos++];}
    void write(const uint8_t *p,size_t n){
        sent.emplace_back(p,p+n);input.clear();pos=0;
        if(!replies.empty()){input=replies.front();replies.pop_front();}
    }
    void flush(){}
} Serial1;
struct Hardware {
    Hardware(uint32_t,void*){}
    bool cancelled(){return false;}
    void drain(){while(Serial1.available())Serial1.read();}
    METHODS
};
WRAPPERS
int main(){
    // 原厂33字节示例，地址改为7；内部含0x6B也不能提前截帧。
    std::vector<uint8_t> original={7,0x42,0x21,0x15,0x19,2,2,2,0,16,1,0,3,0xE8,0x0B,0xB8,0x0F,0xA0,5,7,7,0,1,1,0,40,9,0x60,0x0F,0xA0,0,1,0x6B};
    EmmFeedbackParser parser;EmmReply parsed;
    auto embedded=original;embedded[13]=0x6B;
    for(size_t i=0;i<embedded.size()-1;++i)assert(!parser.feed(embedded[i],parsed));
    assert(parser.feed(embedded.back(),parsed)&&parsed.size==33);
    uint16_t values[7]={};Serial1.replies={original};
    assert(!ReadMotorDriveParams(7,values));
    uint16_t expected[7]={4000,1000,3000,40,2400,4000,1};
    assert(!memcmp(values,expected,sizeof(values)));
    assert(Serial1.sent[0]==std::vector<uint8_t>({7,0x42,0x6C,0x6B}));
    uint16_t desired[7]={5000,1100,2400,8,2200,2000,8};
    uint16_t requested[7];memcpy(requested,desired,sizeof(desired));
    EmmDriveConfig patched;for(int i=0;i<28;++i)patched.data[i]=original[i+4];
    for(int i=0;i<7;++i)patched.set(i,desired[i]);
    auto actual=original;for(int i=0;i<28;++i)actual[i+4]=patched.data[i];
    Serial1={};Serial1.replies={original,{7,0x48,2,0x6B},actual};gaps=0;
    assert(!SetMotorDriveParamsVerified(7,desired,true));assert(gaps==2&&flashGaps==1&&depth==0);
    assert(Serial1.sent.size()==3);
    const auto &sent=Serial1.sent[1];assert(sent.size()==33&&sent[0]==7&&sent[1]==0x48&&sent[2]==0xD1&&sent[3]==1&&sent[32]==0x6B);
    bool changed[28]={};for(int i=0;i<7;++i){changed[EmmDriveConfig::offset(i)]=true;changed[EmmDriveConfig::offset(i)+1]=true;}
    for(int i=0;i<28;++i)if(!changed[i])assert(sent[i+4]==original[i+4]);
    assert(sent[16]==0x13&&sent[17]==0x88); // 最大电压5000mV
    assert(sent[24]==0&&sent[25]==8&&sent[30]==0&&sent[31]==8);
    // 回读不一致不能宣称成功；没有重发。
    Serial1={};Serial1.replies={original,{7,0x48,2,0x6B},original};
    Serial.text.clear();gaps=0;flashGaps=0;
    assert(!strcmp(SetMotorDriveParamsVerified(7,desired,false),"DRIVE_VERIFY_MISMATCH"));assert(Serial1.sent.size()==3);
    assert(gaps==2&&flashGaps==0);
    assert(!memcmp(desired,expected,sizeof(desired))); // 返回真实值，不保留未生效请求。
    memcpy(desired,requested,sizeof(desired));
    assert(Serial.text.find("stage=VERIFY field=0 expected=5000 actual=4000")!=std::string::npos);
    Serial1={};Serial1.replies={original,{7,0x48,0xE2,0x6B}};
    Serial.text.clear();
    assert(!strcmp(SetMotorDriveParamsVerified(7,desired,false),"MOTOR_COMMAND_REJECTED"));assert(Serial1.sent.size()==2);
    assert(Serial.text.find("stage=WRITE_ACK error=MOTOR_COMMAND_REJECTED")!=std::string::npos);
    Serial1={};Serial1.replies={{}};
    Serial.text.clear();
    assert(!strcmp(SetMotorDriveParamsVerified(7,desired,false),"MOTOR_FEEDBACK_TIMEOUT"));assert(Serial1.sent.size()==1);
    assert(Serial.text.find("stage=BASE_READ error=MOTOR_FEEDBACK_TIMEOUT (write not sent)")!=std::string::npos);
    Serial1={};Serial1.replies={original,{7,0x48,2,0x6B},{}};Serial.text.clear();
    assert(!strcmp(SetMotorDriveParamsVerified(7,desired,false),"MOTOR_FEEDBACK_TIMEOUT"));assert(Serial1.sent.size()==3);
    assert(Serial.text.find("stage=READBACK error=MOTOR_FEEDBACK_TIMEOUT")!=std::string::npos);
    assert(Serial.text.find("config ACK RX=07 48 02 6B")!=std::string::npos);
    // ACK缺失仍可按实际回读确认，已超时的路径不追加5ms。
    Serial1={};Serial1.replies={original,{},actual};gaps=0;
    assert(!SetMotorDriveParamsVerified(7,desired,false));assert(gaps==1&&Serial1.sent.size()==3);
}
'''.replace('METHODS', methods).replace('WRAPPERS', wrappers)
        path = ROOT / '.pio/motion-native/emm_drive_config.cpp'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(harness, encoding='utf-8')
        run_native(build_native('emm-drive-config', [str(path)]))
