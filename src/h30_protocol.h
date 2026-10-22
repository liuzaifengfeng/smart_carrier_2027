#pragma once
#include <stdint.h>
#include <stddef.h>

// Arduino 默认 C++11；测试用 C++14 在编译期执行同一份解析逻辑。
#if __cplusplus >= 201402L
#define H30_CONSTEXPR constexpr
#else
#define H30_CONSTEXPR
#endif

struct H30Attitude {
    float pitch = 0, roll = 0, yaw = 0; // 设备原始坐标，单位 deg
    float accel[3]={},gyro[3]={}; // 原始坐标，m/s²（含重力）、deg/s
    uint8_t fields=0; // bit0加速度、bit1角速度、bit2姿态；缺项不能当成零
    uint16_t sequence = 0;
    uint32_t receivedAtMs = 0;
};

// 单个 USB 客户端任务持有；不假设 USB 包边界等于 YIS 帧边界。
class H30Parser {
public:
    /**
     * @brief 清空H30字节解析缓存，下一帧从帧头重新接收。
     */
    H30_CONSTEXPR void reset() { size_ = 0; }
    /**
     * @brief 逐字节解析H30 YIS数据帧，校验通过且含姿态字段时输出sample。
     * @return true 得到完整姿态帧，false 尚未形成有效姿态帧。
     */
    H30_CONSTEXPR bool feed(uint8_t byte, H30Attitude &sample) {
        buffer_[size_++] = byte;
        for (;;) {
            if (size_ && buffer_[0] != 0x59) { discard(1); continue; }
            if (size_ < 2) return false;
            if (buffer_[1] != 0x53) { discard(1); continue; }
            if (size_ < 5) return false;
            const size_t total = buffer_[4] + 7;
            if (size_ < total) return false;
            uint8_t a = 0, b = 0;
            for (size_t i = 2; i < total - 2; ++i) { a += buffer_[i]; b += a; }
            if (a != buffer_[total - 2] || b != buffer_[total - 1]) {
                discard(1); continue;
            }
            bool valid = true, attitude = false;
            H30Attitude result;
            result.sequence = buffer_[2] | (uint16_t(buffer_[3]) << 8);
            size_t pos = 5;
            while (pos < total - 2) {
                if (pos + 2 > total - 2) { valid = false; break; }
                const uint8_t id = buffer_[pos++], len = buffer_[pos++];
                if (pos + len > total - 2) { valid = false; break; }
                if (id == 0x40) {
                    if (len != 12) { valid = false; break; }
                    result.pitch = component(buffer_ + pos);
                    result.roll = component(buffer_ + pos + 4);
                    result.yaw = component(buffer_ + pos + 8);
                    attitude = true;
                    result.fields|=4;
                } else if(id==0x10||id==0x20) {
                    if(len!=12){valid=false;break;}
                    for(size_t axis=0;axis<3;++axis) {
                        if(id==0x10)result.accel[axis]=component(buffer_+pos+axis*4);
                        else result.gyro[axis]=component(buffer_+pos+axis*4);
                    }
                    result.fields|=id==0x10?1:2;
                }
                pos += len;
            }
            discard(total);
            if (valid && attitude) { sample = result; return true; }
        }
    }
private:
    uint8_t buffer_[262] = {};
    size_t size_ = 0;
    /**
     * @brief 丢弃H30缓存前缀字节，并保留剩余未消费数据。
     */
    H30_CONSTEXPR void discard(size_t count) {
        size_ -= count;
        for (size_t i = 0; i < size_; ++i) buffer_[i] = buffer_[i + count];
    }
    /**
     * @brief 将小端有符号32位字段按1e-6比例解码为浮点物理量。
     */
    static H30_CONSTEXPR float component(const uint8_t *p) {
        const uint32_t bits = uint32_t(p[0]) | (uint32_t(p[1]) << 8)
            | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
        const int32_t value = bits & 0x80000000u ? int64_t(bits) - 0x100000000LL : bits;
        return value * 0.000001f;
    }
};
#undef H30_CONSTEXPR
