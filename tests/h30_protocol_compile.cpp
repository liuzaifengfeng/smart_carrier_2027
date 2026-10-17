#include "../src/h30_protocol.h"

struct Frame {
    uint8_t bytes[262] = {0x59, 0x53, 0x34, 0x12, 14, 0x40, 12};
    size_t length = 21;
    constexpr Frame() {
        const int32_t values[] = {1000000, -2000000, 3000000};
        for (int axis = 0; axis < 3; ++axis)
            for (int i = 0; i < 4; ++i)
                bytes[7 + axis * 4 + i] = uint32_t(values[axis]) >> (8 * i);
        checksum();
    }
    constexpr void checksum() {
        uint8_t a = 0, b = 0;
        for (size_t i = 2; i < length - 2; ++i) { a += bytes[i]; b += a; }
        bytes[length - 2] = a; bytes[length - 1] = b;
    }
};
constexpr int feed(H30Parser &parser, H30Attitude &sample, const Frame &f) {
    int count = 0;
    for (size_t i = 0; i < f.length; ++i) if (parser.feed(f.bytes[i], sample)) ++count;
    return count;
}
constexpr bool verify() {
    H30Parser parser;
    H30Attitude sample;
    Frame good;
    parser.feed(0x00, sample); parser.feed(0x59, sample); parser.feed(0x59, sample);
    if (feed(parser, sample, good) != 1 || sample.sequence != 0x1234
            || sample.pitch != 1 || sample.roll != -2 || sample.yaw != 3) return false;
    Frame bad = good; bad.bytes[20] ^= 1;
    if (feed(parser, sample, bad) || feed(parser, sample, good) != 1) return false;
    Frame malformed = good;
    malformed.bytes[6] = 13; malformed.checksum(); // 项目越界
    if (feed(parser, sample, malformed)) return false;
    malformed.bytes[6] = 11; malformed.checksum(); // 错误姿态长度
    if (feed(parser, sample, malformed)) return false;
    malformed = good; malformed.bytes[5] = 0x20; malformed.checksum(); // 缺少姿态
    if (feed(parser, sample, malformed)) return false;
    if (feed(parser, sample, good) != 1 || feed(parser, sample, good) != 1) return false;
    for (size_t i = 0; i < 9; ++i) parser.feed(good.bytes[i], sample);
    parser.reset(); // 重连清除半帧
    if (feed(parser, sample, good) != 1) return false;
    Frame maximum;
    maximum.bytes[4] = 255; maximum.length = 262; maximum.checksum();
    feed(parser, sample, maximum);
    return feed(parser, sample, good) == 1;
}
static_assert(verify(), "YIS checksum, signed axes, boundaries, resync and reconnect");
