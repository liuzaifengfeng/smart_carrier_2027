// H30 串行字节流的帧解析。
// USB 层输入收到的字节，解析结果供上层构造姿态样本。

#pragma once  // 要求编译器在同一编译单元中只展开本头文件一次。
#include <stdint.h>  // 引入 stdint.h 中的声明和定义。
#include <stddef.h>  // 引入 stddef.h 中的声明和定义。

// Arduino 默认 C++11；测试用 C++14 在编译期执行同一份解析逻辑。
#if __cplusplus >= 201402L  // 在编译阶段判断是否纳入此分支。
#define H30_CONSTEXPR constexpr  // 为后续文本展开定义宏 H30_CONSTEXPR constexpr。
#else  // 前面的编译条件均未选中时使用此分支。
#define H30_CONSTEXPR  // 为后续文本展开定义宏 H30_CONSTEXPR。
#endif  // 结束本组条件编译。

struct H30Attitude {  // 定义 H30Attitude 的类型成员或作用域。
    float pitch = 0, roll = 0, yaw = 0; // 设备原始坐标，单位 deg
    float accel[3]={},gyro[3]={}; // 原始坐标，m/s²（含重力）、deg/s
    uint8_t fields=0; // bit0加速度、bit1角速度、bit2姿态；缺项不能当成零
    uint16_t sequence = 0;  // 将 0 写入或初始化 sequence。
    uint32_t receivedAtMs = 0;  // 将 0 写入或初始化 receivedAtMs（已接收At毫秒）。
};  // 结束此类型定义。

// 单个 USB 客户端任务持有；不假设 USB 包边界等于 YIS 帧边界。
class H30Parser {  // 定义 H30Parser 的类型成员或作用域。
public:  // 此后的成员采用 public 访问权限，直到下一个权限标签。
    /**
     * @brief 清空H30字节解析缓存，下一帧从帧头重新接收。
     */
    H30_CONSTEXPR void reset() { size_ = 0; }  // 将 0; } 写入或初始化 size_（大小）。
    /**
     * @brief 逐字节解析H30 YIS数据帧，校验通过且含姿态字段时输出sample。
     * @return true 得到完整姿态帧，false 尚未形成有效姿态帧。
     */
    H30_CONSTEXPR bool feed(uint8_t byte, H30Attitude &sample) {  // 定义函数或成员 的入口 feed，参数由此处的签名确定。
        buffer_[size_++] = byte;  // 将 byte 写入或初始化 buffer_[size_++]。
        for (;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
            if (size_ && buffer_[0] != 0x59) { discard(1); continue; }  // 当 size_ && buffer_[0] != 0x59 成立时进入此分支，否则继续后续流程。
            if (size_ < 2) return false;  // 若 size_ < 2，立即返回 false。
            if (buffer_[1] != 0x53) { discard(1); continue; }  // 当 buffer_[1] != 0x53 成立时进入此分支，否则继续后续流程。
            if (size_ < 5) return false;  // 若 size_ < 5，立即返回 false。
            const size_t total = buffer_[4] + 7;  // 将 buffer_[4] + 7 写入或初始化 total（总计）。
            if (size_ < total) return false;  // 若 size_ < total，立即返回 false。
            uint8_t a = 0, b = 0;  // 将 0, b = 0 写入或初始化 a。
            for (size_t i = 2; i < total - 2; ++i) { a += buffer_[i]; b += a; }  // 按 size_t i = 2; i < total - 2; ++i 组织循环的初值、继续条件和步进。
            if (a != buffer_[total - 2] || b != buffer_[total - 1]) {  // 当 a != buffer_[total - 2] || b != buffer_[total - 1] 成立时进入此分支，否则继续后续流程。
                discard(1); continue;  // 调用 discard，传入 1。然后跳过本次剩余处理，开始下一轮循环。
            }  // 结束此条件分支。
            bool valid = true, attitude = false;  // 将 true, attitude = false 写入或初始化 valid（有效）。
            H30Attitude result;  // 声明 result（结果），类型和可用范围由所在声明决定。
            result.sequence = buffer_[2] | (uint16_t(buffer_[3]) << 8);  // 将 buffer_[2] | (uint16_t(buffer_[3]) << 8) 写入或初始化 result.sequence。
            size_t pos = 5;  // 将 5 写入或初始化 pos（位置）。
            while (pos < total - 2) {  // 当 pos < total - 2 成立时继续循环。
                if (pos + 2 > total - 2) { valid = false; break; }  // 当 pos + 2 > total - 2 成立时进入此分支，否则继续后续流程。
                const uint8_t id = buffer_[pos++], len = buffer_[pos++];  // 将 buffer_[pos++], len = buffer_[pos++] 写入或初始化 id（编号）。
                if (pos + len > total - 2) { valid = false; break; }  // 当 pos + len > total - 2 成立时进入此分支，否则继续后续流程。
                if (id == 0x40) {  // 当 id == 0x40 成立时进入此分支，否则继续后续流程。
                    if (len != 12) { valid = false; break; }  // 当 len != 12 成立时进入此分支，否则继续后续流程。
                    result.pitch = component(buffer_ + pos);  // 将 component(buffer_ + pos) 写入或初始化 result.pitch。
                    result.roll = component(buffer_ + pos + 4);  // 将 component(buffer_ + pos + 4) 写入或初始化 result.roll。
                    result.yaw = component(buffer_ + pos + 8);  // 将 component(buffer_ + pos + 8) 写入或初始化 result.yaw。
                    attitude = true;  // 将 true 写入或初始化 attitude。
                    result.fields|=4;  // 用 4 对 result.fields 应用 |= 更新。
                } else if(id==0x10||id==0x20) {  // 仅在先前条件均不成立时，当 id==0x10||id==0x20 成立时进入此分支，否则继续后续流程。
                    if(len!=12){valid=false;break;}  // 当 len!=12 成立时进入此分支，否则继续后续流程。
                    for(size_t axis=0;axis<3;++axis) {  // 按 size_t axis=0;axis<3;++axis 组织循环的初值、继续条件和步进。
                        if(id==0x10)result.accel[axis]=component(buffer_+pos+axis*4);  // 当 id==0x10 成立时进入此分支，否则继续后续流程。
                        else result.gyro[axis]=component(buffer_+pos+axis*4);  // 前一个判断不成立时执行此分支。
                    }  // 结束此循环体。
                    result.fields|=id==0x10?1:2;  // 用 id==0x10?1:2 对 result.fields 应用 |= 更新。
                }  // 结束此条件分支。
                pos += len;  // 把 len 累加到 pos（位置）。
            }  // 结束此循环体。
            discard(total);  // 调用 discard，传入 total。
            if (valid && attitude) { sample = result; return true; }  // 当 valid && attitude 成立时进入此分支，否则继续后续流程。
        }  // 结束此循环体。
    }  // 结束此函数或控制块。
private:  // 此后的成员采用 private 访问权限，直到下一个权限标签。
    uint8_t buffer_[262] = {};  // 将 {} 写入或初始化 buffer_[262]。
    size_t size_ = 0;  // 将 0 写入或初始化 size_（大小）。
    /**
     * @brief 丢弃H30缓存前缀字节，并保留剩余未消费数据。
     */
    H30_CONSTEXPR void discard(size_t count) {  // 定义函数或成员 的入口 discard，参数由此处的签名确定。
        size_ -= count;  // 从 size_（大小） 扣除 count。
        for (size_t i = 0; i < size_; ++i) buffer_[i] = buffer_[i + count];  // 按 size_t i = 0; i < size_; ++i) buffer_[i] = buffer_[i +… 组织循环的初值、继续条件和步进。
    }  // 结束此函数或控制块。
    /**
     * @brief 将小端有符号32位字段按1e-6比例解码为浮点物理量。
     */
    static H30_CONSTEXPR float component(const uint8_t *p) {  // 定义函数或成员 的入口 component，参数由此处的签名确定。
        const uint32_t bits = uint32_t(p[0]) | (uint32_t(p[1]) << 8)  // 将 uint32_t(p[0]) | (uint32_t(p[1]) << 8) 写入或初始化 bits。
            | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);  // 调用 uint32_t，传入 p[2]) << 16) | (uint32_t(p[3]) << 24。
        const int32_t value = bits & 0x80000000u ? int64_t(bits) - 0x100000000LL : bits;  // 将 bits & 0x80000000u ? int64_t(bits) - 0x100000000LL : b… 写入或初始化 value（测值）。
        return value * 0.000001f;  // 结束当前函数并返回 value * 0.000001f。
    }  // 结束此函数或控制块。
};  // 结束此类型定义。
#undef H30_CONSTEXPR  // 撤销宏 H30_CONSTEXPR，限定其作用范围。
