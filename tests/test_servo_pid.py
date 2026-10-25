"""执行实际PID读写函数，核对厂家专用帧头、字节顺序和保留参数。"""
import unittest
from native_support import ROOT, build_native, run_native
from test_servo_multiturn_interval import function_source


class ServoPidTests(unittest.TestCase):
    def test_actual_pid_packets_and_reserved_bytes(self):
        source = (ROOT / 'src/servo.cpp').read_text(encoding='utf-8')
        actual = '\n'.join(function_source(source, name) for name in (
            'static void sendPacket(', 'static bool readPidBlock(',
            'bool Servo_ReadPid(', 'bool Servo_WritePid('))
        harness = r'''
#include <stdint.h>
#include <assert.h>
#include <vector>
#include <string.h>
constexpr uint16_t FSUS_PACK_REQ_HEADER=0x4C12;
constexpr uint8_t FSUS_CMD_STOP=0x18,FSUS_STATUS_SUCCESS=0;
struct ServoBusGuard{bool locked=true;};
uint32_t waited=0;
uint32_t pdMS_TO_TICKS(uint32_t x){return x;}
void vTaskDelay(uint32_t x){waited+=x;}
struct Port {
    std::vector<uint8_t> bytes;
    int available(){return 0;} int read(){return -1;}
    void flush(){}
    void write(uint8_t x){bytes.push_back(x);}
    void write(const uint8_t *p,uint8_t n){bytes.insert(bytes.end(),p,p+n);}
} port;
Port *s_servoSerial=&port;
uint8_t original[26];bool available=true;
uint8_t receivePacket(uint8_t cmd,uint8_t *out,uint8_t *length,uint32_t){
    assert(cmd==0xC5);
    if(!available)return 1;
    memcpy(out,original,26);*length=26;return 0;
}
ACTUAL
int main(){
    original[0]=2;for(int i=1;i<26;++i)original[i]=i;
    uint16_t values[7];assert(Servo_ReadPid(2,values));
    for(int i=0;i<7;++i)assert(values[i]==(1+2*i)+((2+2*i)<<8));
    const uint8_t query[]={0x13,0x4D,0xC5,1,2,0x28};
    assert(port.bytes.size()==6&&!memcmp(port.bytes.data(),query,6));
    port.bytes.clear();
    for(int i=0;i<7;++i)values[i]=0x1234+i;
    assert(Servo_WritePid(2,values)&&waited==360);
    const auto &b=port.bytes;
    assert(b.size()==55); // C5查询6 + 解锁9 + C4写入31 + 锁力9。
    assert(b[8]==0x18&&b[11]==0x10);
    assert(b[15]==0x13&&b[16]==0x4D&&b[17]==0xC4&&b[18]==26&&b[19]==2);
    for(int i=0;i<7;++i){assert(b[20+2*i]==0x34+i);assert(b[21+2*i]==0x12);}
    for(int i=15;i<26;++i)assert(b[19+i]==original[i]);
    assert(b[48]==0x18&&b[51]==0x11);
    for(size_t offset=0;offset<b.size();){
        size_t size=b[offset+3]+5;uint8_t sum=0;
        for(size_t i=0;i<size-1;++i)sum+=b[offset+i];
        assert(sum==b[offset+size-1]);offset+=size;
    }
    port.bytes.clear();available=false;
    assert(!Servo_WritePid(2,values)&&port.bytes.size()==6); // 未读取原块时不覆盖保留字节。
}
'''.replace('ACTUAL', actual)
        cache = ROOT / '.pio/motion-native'
        cache.mkdir(parents=True, exist_ok=True)
        path = cache / 'servo_pid_actual.cpp'
        path.write_text(harness, encoding='utf-8')
        run_native(build_native('servo-pid', [str(path)]))
