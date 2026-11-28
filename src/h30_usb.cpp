// H30 USB Host 适配层。
// 设备事件管理连接状态，接收回调更新姿态数据，上层通过快照读取角度及采样时间。
// 设备失效和航向校准各有独立入口。

#include "h30_usb.h"  // 引入 h30_usb.h 中的声明和定义。
#include "h30_heading.h"  // 引入 h30_heading.h 中的声明和定义。
#include <Arduino.h>  // 引入 Arduino.h 中的声明和定义。
#include "usb/usb_host.h"  // 引入 usb/usb_host.h 中的声明和定义。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // 将 portMUX_INITIALIZER_UNLOCKED 写入或初始化 mux（临界区锁）。
H30Attitude latest;  // 声明 latest，类型和可用范围由所在声明决定。
H30HeadingReference headingReference; // 与 latest 共用 mux，标定与采样原子一致
uint32_t headingReferenceVersion=0;  // 将 0 写入或初始化 headingReferenceVersion（航向Reference版本）。
H30HeadingAverage headingAverage;  // 声明 headingAverage（航向Average），类型和可用范围由所在声明决定。
constexpr uint32_t HEADING_SETTLE_MS=500;  // 将 500 写入或初始化 HEADING_SETTLE_MS（航向SETTLE毫秒）。
bool headingAverageActive=false,headingAverageSettling=false;  // 将 false,headingAverageSettling=false 写入或初始化 headingAverageActive（航向Average活跃）。
bool headingAverageReady=false;  // 将 false 写入或初始化 headingAverageReady（航向Average就绪）。
float headingAverageYaw=0;  // 将 0 写入或初始化 headingAverageYaw（航向Average偏航）。
bool initialized = false, online = false, valid = false;  // 将 false, online = false, valid = false 写入或初始化 initialized。
usb_host_client_handle_t client = nullptr;  // 将 nullptr 写入或初始化 client。
usb_device_handle_t device = nullptr;  // 将 nullptr 写入或初始化 device。
usb_transfer_t *rx = nullptr, *control = nullptr;  // 将 nullptr, *control = nullptr 写入或初始化 rx（接收）。
int commIf = -1, dataIf = -1;  // 将 -1, dataIf = -1 写入或初始化 commIf。
bool commClaimed = false, dataClaimed = false;  // 将 false, dataClaimed = false 写入或初始化 commClaimed。
uint8_t endpoint = 0;  // 将 0 写入或初始化 endpoint。
bool rxPending = false, controlPending = false, stopping = false;  // 将 false, controlPending = false, stopping = false 写入或初始化 rxPending（接收Pending）。
bool controlDone = false, controlOK = false;  // 将 false, controlOK = false 写入或初始化 controlDone（control完成）。
int stage = 0;  // 将 0 写入或初始化 stage（阶段）。
uint32_t controlStarted = 0, retryAt = 0;  // 将 0, retryAt = 0 写入或初始化 controlStarted。
H30Parser parser;  // 声明 parser，类型和可用范围由所在声明决定。

void invalidate() {  // 定义函数或成员 的入口 invalidate，参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    online = false;  // 将 false 写入或初始化 online。
    valid = false;  // 将 false 写入或初始化 valid（有效）。
    headingReference.reset();  // 调用 headingReference.reset。
    headingAverageReady=false;  // 将 false 写入或初始化 headingAverageReady（航向Average就绪）。
    ++headingReferenceVersion;  // 续写当前声明或运算，提供 ++headingReferenceVersion;。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    parser.reset();  // 调用 parser.reset。
}  // 结束此函数或控制块。

void events(const usb_host_client_event_msg_t *event, void *) {  // 定义函数或成员 的入口 events，参数由此处的签名确定。
    if (event->event == USB_HOST_CLIENT_EVENT_DEV_GONE  // 当 event->event == USB_HOST_CLIENT_EVENT_DEV_GONE 成立时进入此分支，否则继续后续流程。
            && event->dev_gone.dev_hdl == device) {  // 继续上一行的组合条件，本行提供 && event->dev_gone.dev_hdl == device) {。
        stopping = true;  // 将 true 写入或初始化 stopping。
        invalidate();  // 调用 invalidate，使用无参形式推进相应操作。
    }  // 结束此初始化列表或局部作用域。
}  // 结束此函数或控制块。

void received(usb_transfer_t *transfer) {  // 定义函数或成员 的入口 received（已接收），参数由此处的签名确定。
    rxPending = false;  // 将 false 写入或初始化 rxPending（接收Pending）。
    if (stopping) return;  // 当 stopping 成立时进入此分支，否则继续后续流程。
    if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {  // 当 transfer->status != USB_TRANSFER_STATUS_COMPLETED 成立时进入此分支，否则继续后续流程。
        stopping = true;  // 将 true 写入或初始化 stopping。
        invalidate();  // 调用 invalidate，使用无参形式推进相应操作。
        return;  // 结束当前函数，将执行权交回调用者。
    }  // 结束此条件分支。
    // 最多处理 512 字节，无日志、阻塞等待或业务计算，无需另设字节队列。
    for (int i = 0; i < transfer->actual_num_bytes; ++i) {  // 按 int i = 0; i < transfer->actual_num_bytes; ++i 组织循环的初值、继续条件和步进。
        H30Attitude sample;  // 声明 sample（采样），类型和可用范围由所在声明决定。
        if (!parser.feed(transfer->data_buffer[i], sample)) continue;  // 若 !parser.feed(transfer->data_buffer[i], sample)，直接进入下一轮循环。
        sample.receivedAtMs = millis();  // 将 millis() 写入或初始化 sample.receivedAtMs。
        portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
        // 输出中断可能伴随模块复位或航向跳变；恢复数据后仍需重新标定。
        if (valid && sample.receivedAtMs - latest.receivedAtMs > 200) {  // 当 valid && sample.receivedAtMs - latest.receivedAtMs > 2… 成立时进入此分支，否则继续后续流程。
            headingReference.reset();++headingReferenceVersion;  // 调用 headingReference.reset。然后续写当前声明或运算，提供 ++headingReferenceVersion;。
            headingAverageReady=false;  // 将 false 写入或初始化 headingAverageReady（航向Average就绪）。
        }  // 结束此条件分支。
        if(headingAverageActive)headingAverage.add(sample);  // 当 headingAverageActive 成立时进入此分支，否则继续后续流程。
        latest = sample;  // 将 sample 写入或初始化 latest。
        valid = true;  // 将 true 写入或初始化 valid（有效）。
        portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    }  // 结束此循环体。
    if (usb_host_transfer_submit(rx) == ESP_OK) rxPending = true;  // 当 usb_host_transfer_submit(rx) == ESP_OK 成立时进入此分支，否则继续后续流程。
    else { stopping = true; invalidate(); }  // 前一个判断不成立时执行此分支。
}  // 结束此函数或控制块。

void controlled(usb_transfer_t *transfer) {  // 定义函数或成员 的入口 controlled，参数由此处的签名确定。
    controlPending = false;  // 将 false 写入或初始化 controlPending。
    controlDone = true;  // 将 true 写入或初始化 controlDone（control完成）。
    controlOK = transfer->status == USB_TRANSFER_STATUS_COMPLETED;  // 将 transfer->status == USB_TRANSFER_STATUS_COMPLETED 写入或初始化 controlOK。
}  // 结束此函数或控制块。

bool submitControl(bool lineCoding) {  // 定义函数或成员 的入口 submitControl，参数由此处的签名确定。
    memset(control->data_buffer, 0, 15);  // 调用 memset，传入 control->data_buffer, 0, 15。
    uint8_t *p = control->data_buffer;  // 将 control->data_buffer 写入或初始化 p。
    p[0] = 0x21;  // 将 0x21 写入或初始化 p[0]。
    p[1] = lineCoding ? 0x20 : 0x22;  // 将 lineCoding ? 0x20 : 0x22 写入或初始化 p[1]。
    p[2] = lineCoding ? 0 : 3;  // 将 lineCoding ? 0 : 3 写入或初始化 p[2]。
    p[4] = commIf;  // 将 commIf 写入或初始化 p[4]。
    p[6] = lineCoding ? 7 : 0;  // 将 lineCoding ? 7 : 0 写入或初始化 p[6]。
    if (lineCoding) {  // 当 lineCoding 成立时进入此分支，否则继续后续流程。
        const uint8_t coding[] = {0x00, 0x08, 0x07, 0x00, 0x00, 0x00, 0x08};  // 声明 coding，类型和可用范围由所在声明决定。
        memcpy(p + 8, coding, sizeof(coding));  // 调用 memcpy，传入 p + 8, coding, sizeof(coding)。
    }  // 结束此条件分支。
    control->device_handle = device;  // 将 device 写入或初始化 control->device_handle。
    control->bEndpointAddress = 0;  // 将 0 写入或初始化 control->bEndpointAddress。
    control->callback = controlled;  // 将 controlled 写入或初始化 control->callback。
    control->num_bytes = lineCoding ? 15 : 8;  // 将 lineCoding ? 15 : 8 写入或初始化 control->num_bytes。
    controlDone = false;  // 将 false 写入或初始化 controlDone（control完成）。
    if (usb_host_transfer_submit_control(client, control) != ESP_OK) return false;  // 若 usb_host_transfer_submit_control(client, control) != E…，立即返回 false。
    controlPending = true;  // 将 true 写入或初始化 controlPending。
    controlStarted = millis();  // 将 millis() 写入或初始化 controlStarted。
    return true;  // 结束当前函数并返回 true。
}  // 结束此函数或控制块。

bool openDevice(uint8_t address) {  // 定义函数或成员 的入口 openDevice，参数由此处的签名确定。
    if (usb_host_device_open(client, address, &device) != ESP_OK) return false;  // 若 usb_host_device_open(client, address, &device) != ESP_…，立即返回 false。
    const usb_device_desc_t *desc = nullptr;  // 将 nullptr 写入或初始化 desc。
    if (usb_host_get_device_descriptor(device, &desc) != ESP_OK  // 当 usb_host_get_device_descriptor(device, &desc) != ESP_OK 成立时进入此分支，否则继续后续流程。
            || desc->idVendor != 0x1A86 || desc->idProduct != 0x55D4) return false;  // 继续上一行的组合条件，本行提供 || desc->idVendor != 0x1A86 || desc->idProduct != 0x55D4) return false;。
    const usb_config_desc_t *config = nullptr;  // 将 nullptr 写入或初始化 config（配置）。
    if (usb_host_get_active_config_descriptor(device, &config) != ESP_OK) return false;  // 若 usb_host_get_active_config_descriptor(device, &config)…，立即返回 false。
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(config);  // 将 reinterpret_cast<const uint8_t *>(config) 写入或初始化 bytes（字节）。
    int currentIf = -1, currentClass = -1;  // 将 -1, currentClass = -1 写入或初始化 currentIf（当前If）。
    uint16_t packetSize = 0;  // 将 0 写入或初始化 packetSize（packet大小）。
    for (size_t pos = 0; pos + 2 <= config->wTotalLength;) {  // 按 size_t pos = 0; pos + 2 <= config->wTotalLength; 组织循环的初值、继续条件和步进。
        const uint8_t len = bytes[pos], type = bytes[pos + 1];  // 将 bytes[pos], type = bytes[pos + 1] 写入或初始化 len。
        if (len < 2 || pos + len > config->wTotalLength) return false;  // 若 len < 2 || pos + len > config->wTotalLength，立即返回 false。
        const uint8_t *p = bytes + pos;  // 将 bytes + pos 写入或初始化 p。
        if (type == 4 && len >= 9) {  // 当 type == 4 && len >= 9 成立时进入此分支，否则继续后续流程。
            currentIf = p[3] == 0 ? p[2] : -1; // 只使用默认 alternate setting
            currentClass = p[5];  // 将 p[5] 写入或初始化 currentClass（当前Class）。
            if (currentIf >= 0 && currentClass == 2 && p[6] == 2) commIf = currentIf;  // 当 currentIf >= 0 && currentClass == 2 && p[6] == 2 成立时进入此分支，否则继续后续流程。
        } else if (type == 5 && len >= 7 && currentIf >= 0 && currentClass == 0x0A  // 仅在先前条件均不成立时，当 type == 5 && len >= 7 && currentIf >= 0 && currentClas… 成立时进入此分支，否则继续后续流程。
                && (p[2] & 0x80) && (p[3] & 3) == 2) {  // 继续上一行的组合条件，本行提供 && (p[2] & 0x80) && (p[3] & 3) == 2) {。
            dataIf = currentIf;  // 将 currentIf 写入或初始化 dataIf（数据If）。
            endpoint = p[2];  // 将 p[2] 写入或初始化 endpoint。
            packetSize = p[4] | (uint16_t(p[5]) << 8);  // 将 p[4] | (uint16_t(p[5]) << 8) 写入或初始化 packetSize（packet大小）。
        }  // 结束此函数或控制块。
        pos += len;  // 把 len 累加到 pos（位置）。
    }  // 结束此循环体。
    if (commIf < 0 || dataIf < 0 || !packetSize || packetSize > 512) return false;  // 若 commIf < 0 || dataIf < 0 || !packetSize || packetSize …，立即返回 false。
    if (usb_host_interface_claim(client, device, commIf, 0) != ESP_OK) return false;  // 若 usb_host_interface_claim(client, device, commIf, 0) !=…，立即返回 false。
    commClaimed = true;  // 将 true 写入或初始化 commClaimed。
    if (usb_host_interface_claim(client, device, dataIf, 0) != ESP_OK) return false;  // 若 usb_host_interface_claim(client, device, dataIf, 0) !=…，立即返回 false。
    dataClaimed = true;  // 将 true 写入或初始化 dataClaimed（数据Claimed）。
    if (usb_host_transfer_alloc(512, 0, &rx) != ESP_OK  // 当 usb_host_transfer_alloc(512, 0, &rx) != ESP_OK 成立时进入此分支，否则继续后续流程。
            || usb_host_transfer_alloc(15, 0, &control) != ESP_OK) return false;  // 继续上一行的组合条件，本行提供 || usb_host_transfer_alloc(15, 0, &control) != ESP_OK) return false;。
    rx->device_handle = device;  // 将 device 写入或初始化 rx->device_handle。
    rx->bEndpointAddress = endpoint;  // 将 endpoint 写入或初始化 rx->bEndpointAddress。
    rx->num_bytes = (512 / packetSize) * packetSize;  // 将 (512 / packetSize) * packetSize 写入或初始化 rx->num_bytes。
    rx->callback = received;  // 将 received 写入或初始化 rx->callback。
    stage = 1;  // 将 1 写入或初始化 stage（阶段）。
    return submitControl(true);  // 结束当前函数并返回 submitControl(true)。
}  // 结束此函数或控制块。

void closeDevice() {  // 定义函数或成员 的入口 closeDevice，参数由此处的签名确定。
    invalidate();  // 调用 invalidate，使用无参形式推进相应操作。
    // 回调仍可能引用传输对象，必须先取消并通过客户端事件循环收回。
    if (rxPending) {  // 当 rxPending 成立时进入此分支，否则继续后续流程。
        usb_host_endpoint_halt(device, endpoint);  // 调用 usb_host_endpoint_halt，传入 device, endpoint。
        usb_host_endpoint_flush(device, endpoint);  // 调用 usb_host_endpoint_flush，传入 device, endpoint。
    }  // 结束此条件分支。
    // EP0 由 Host 栈管理，控制超时后保留其对象直到完成/拔出回调。
    // 不释放在途对象，也不因外设异常重启整机。
    if (rxPending || controlPending) return;  // 当 rxPending || controlPending 成立时进入此分支，否则继续后续流程。
    usb_host_transfer_free(rx); rx = nullptr;  // 调用 usb_host_transfer_free，传入 rx。然后将 nullptr 写入或初始化 rx（接收）。
    usb_host_transfer_free(control); control = nullptr;  // 调用 usb_host_transfer_free，传入 control。然后将 nullptr 写入或初始化 control。
    if (dataClaimed) usb_host_interface_release(client, device, dataIf);  // 当 dataClaimed 成立时进入此分支，否则继续后续流程。
    if (commClaimed) usb_host_interface_release(client, device, commIf);  // 当 commClaimed 成立时进入此分支，否则继续后续流程。
    if (device) usb_host_device_close(client, device);  // 当 device 成立时进入此分支，否则继续后续流程。
    device = nullptr;  // 将 nullptr 写入或初始化 device。
    commIf = dataIf = -1;  // 将 dataIf = -1 写入或初始化 commIf。
    commClaimed = dataClaimed = false;  // 将 dataClaimed = false 写入或初始化 commClaimed。
    endpoint = 0;  // 将 0 写入或初始化 endpoint。
    stage = 0;  // 将 0 写入或初始化 stage（阶段）。
    controlDone = stopping = false;  // 将 stopping = false 写入或初始化 controlDone（control完成）。
    retryAt = millis() + 1000;  // 将 millis() + 1000 写入或初始化 retryAt（重试At）。
}  // 结束此函数或控制块。

void libraryTask(void *) {  // 定义函数或成员 的入口 libraryTask（library任务），参数由此处的签名确定。
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 调用 ulTaskNotifyTake（ul任务NotifyTake），传入 pdTRUE, portMAX_DELAY。
    for (;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
        uint32_t flags = 0;  // 将 0 写入或初始化 flags（标志位）。
        usb_host_lib_handle_events(portMAX_DELAY, &flags);  // 调用 usb_host_lib_handle_events，传入 portMAX_DELAY, &flags。
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();  // 当 flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS 成立时进入此分支，否则继续后续流程。
    }  // 结束此循环体。
}  // 结束此函数或控制块。

void clientTask(void *) {  // 定义函数或成员 的入口 clientTask（client任务），参数由此处的签名确定。
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 调用 ulTaskNotifyTake（ul任务NotifyTake），传入 pdTRUE, portMAX_DELAY。
    for (;;) {  // 按 ;; 组织循环的初值、继续条件和步进。
        usb_host_client_handle_events(client, pdMS_TO_TICKS(10));  // 调用 usb_host_client_handle_events，传入 client, pdMS_TO_TICKS(10)。
        if (device && controlPending && millis() - controlStarted > 3000) {  // 当 device && controlPending && millis() - controlStarted … 成立时进入此分支，否则继续后续流程。
            stopping = true;  // 将 true 写入或初始化 stopping。
            invalidate();  // 调用 invalidate，使用无参形式推进相应操作。
        }  // 结束此条件分支。
        if (stopping) { closeDevice(); continue; }  // 当 stopping 成立时进入此分支，否则继续后续流程。
        if (device && controlDone) {  // 当 device && controlDone 成立时进入此分支，否则继续后续流程。
            controlDone = false;  // 将 false 写入或初始化 controlDone（control完成）。
            if (!controlOK) stopping = true;  // 当 !controlOK 成立时进入此分支，否则继续后续流程。
            else if (stage == 1) {  // 仅在先前条件均不成立时，当 stage == 1 成立时进入此分支，否则继续后续流程。
                stage = 2;  // 将 2 写入或初始化 stage（阶段）。
                if (!submitControl(false)) stopping = true;  // 当 !submitControl(false) 成立时进入此分支，否则继续后续流程。
            } else if (stage == 2) {  // 仅在先前条件均不成立时，当 stage == 2 成立时进入此分支，否则继续后续流程。
                stage = 3;  // 将 3 写入或初始化 stage（阶段）。
                if (usb_host_transfer_submit(rx) == ESP_OK) {  // 当 usb_host_transfer_submit(rx) == ESP_OK 成立时进入此分支，否则继续后续流程。
                    rxPending = true;  // 将 true 写入或初始化 rxPending（接收Pending）。
                    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
                    online = true;  // 将 true 写入或初始化 online。
                    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
                } else stopping = true;  // 前一个判断不成立时执行此分支。
            }  // 结束此条件分支。
            if (stopping) invalidate();  // 当 stopping 成立时进入此分支，否则继续后续流程。
        }  // 结束此条件分支。
        // 枚举后轮询地址，也覆盖任务启动前已插入的设备以及出错后的重试。
        if (!device && int32_t(millis() - retryAt) >= 0) {  // 当 !device && int32_t(millis() - retryAt) >= 0 成立时进入此分支，否则继续后续流程。
            retryAt = millis() + 1000;  // 将 millis() + 1000 写入或初始化 retryAt（重试At）。
            uint8_t addresses[8];  // 声明 addresses，类型和可用范围由所在声明决定。
            int count = 0;  // 将 0 写入或初始化 count（计数）。
            if (usb_host_device_addr_list_fill(8, addresses, &count) != ESP_OK) continue;  // 若 usb_host_device_addr_list_fill(8, addresses, &count) !…，直接进入下一轮循环。
            for (int i = 0; i < count; ++i) {  // 按 int i = 0; i < count; ++i 组织循环的初值、继续条件和步进。
                if (openDevice(addresses[i])) break;  // 若 openDevice(addresses[i])，从当前循环或分支退出。
                if (device) { stopping = true; closeDevice(); }  // 当 device 成立时进入此分支，否则继续后续流程。
            }  // 结束此循环体。
        }  // 结束此条件分支。
    }  // 结束此循环体。
}  // 结束此函数或控制块。
} // namespace

/**
 * @brief 初始化H30 USB接收，不等待设备插入；失败不阻断其他业务。
 * @return true 初始化成功。
 */
bool H30UsbBegin() {  // 定义函数或成员 的入口 H30UsbBegin，参数由此处的签名确定。
    if (initialized) return true;  // 若 initialized，立即返回 true。
    usb_host_config_t config = {};  // 将 {} 写入或初始化 config（配置）。
    config.intr_flags = ESP_INTR_FLAG_LEVEL1;  // 将 ESP_INTR_FLAG_LEVEL1 写入或初始化 config.intr_flags。
    if (usb_host_install(&config) != ESP_OK) return false;  // 若 usb_host_install(&config) != ESP_OK，立即返回 false。
    usb_host_client_config_t clientConfig = {};  // 将 {} 写入或初始化 clientConfig（client配置）。
    clientConfig.max_num_event_msg = 8;  // 将 8 写入或初始化 clientConfig.max_num_event_msg。
    clientConfig.async.client_event_callback = events;  // 将 events 写入或初始化 clientConfig.async.client_event_callback。
    if (usb_host_client_register(&clientConfig, &client) != ESP_OK) {  // 当 usb_host_client_register(&clientConfig, &client) != ES… 成立时进入此分支，否则继续后续流程。
        usb_host_uninstall();  // 调用 usb_host_uninstall，使用无参形式推进相应操作。
        return false;  // 结束当前函数并返回 false。
    }  // 结束此条件分支。
    // 任务先等待通知，两个核心上都不会在初始化完成前处理 USB。
    TaskHandle_t libTask = nullptr, ioTask = nullptr;  // 将 nullptr, ioTask = nullptr 写入或初始化 libTask（lib任务）。
    const bool ok = xTaskCreate(libraryTask, "H30Host", 4096, nullptr, 4, &libTask) == pdPASS  // 将 xTaskCreate(libraryTask, "H30Host", 4096, nullptr, 4, … 写入或初始化 ok。
        && xTaskCreate(clientTask, "H30Client", 6144, nullptr, 4, &ioTask) == pdPASS;  // 继续上一行的组合条件，本行提供 && xTaskCreate(clientTask, "H30Client", 6144, nullptr, 4, &ioTask) == pdPASS;。
    if (!ok) {  // 当 !ok 成立时进入此分支，否则继续后续流程。
        if (libTask) vTaskDelete(libTask);  // 当 libTask 成立时进入此分支，否则继续后续流程。
        if (ioTask) vTaskDelete(ioTask);  // 当 ioTask 成立时进入此分支，否则继续后续流程。
    }  // 结束此条件分支。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    initialized = ok;  // 将 ok 写入或初始化 initialized。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    if (ok) {  // 当 ok 成立时进入此分支，否则继续后续流程。
        xTaskNotifyGive(libTask);  // 调用 xTaskNotifyGive（横向任务NotifyGive），传入 libTask。
        xTaskNotifyGive(ioTask);  // 调用 xTaskNotifyGive（横向任务NotifyGive），传入 ioTask。
    }  // 结束此条件分支。
    if (!ok) {  // 当 !ok 成立时进入此分支，否则继续后续流程。
        usb_host_client_deregister(client);  // 调用 usb_host_client_deregister，传入 client。
        client = nullptr;  // 将 nullptr 写入或初始化 client。
        usb_host_device_free_all();  // 调用 usb_host_device_free_all，使用无参形式推进相应操作。
        usb_host_uninstall();  // 调用 usb_host_uninstall，使用无参形式推进相应操作。
    }  // 结束此条件分支。
    return ok;  // 结束当前函数并返回 ok。
}  // 结束此函数或控制块。

/**
 * @brief 读取最新原始姿态和数据年龄，不进行地图航向标定。
 * @param sample 输出H30原始姿态。
 * @param ageMs 输出数据年龄(ms)。
 * @return nullptr 成功，否则为初始化、离线、缺数据或过期原因。
 */
const char *H30UsbGetAttitude(H30Attitude &sample, uint32_t &ageMs) {  // 定义函数或成员 的入口 H30UsbGetAttitude，参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    const bool ready = initialized, connected = online, hasSample = valid;  // 将 initialized, connected = online, hasSample = valid 写入或初始化 ready（就绪）。
    sample = latest;  // 将 latest 写入或初始化 sample（采样）。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    ageMs = millis() - sample.receivedAtMs;  // 将 millis() - sample.receivedAtMs 写入或初始化 ageMs（样本年龄毫秒）。
    if (!ready) return "INIT";  // 若 !ready，立即返回 "INIT"。
    if (!connected) return "OFFLINE";  // 若 !connected，立即返回 "OFFLINE"。
    if (!hasSample) return "NO_DATA";  // 若 !hasSample，立即返回 "NO_DATA"。
    return ageMs > 200 ? "STALE" : nullptr;  // 结束当前函数并返回 ageMs > 200 ? "STALE" : nullptr。
}  // 结束此函数或控制块。

namespace {  // 开始匿名命名空间，本文件内部共享其中的定义。
// 调用者必须持有 mux；标定与查询采用完全相同的新鲜度规则。
const char *headingSampleError(uint32_t &ageMs) {  // 定义函数或成员 的入口 headingSampleError（航向采样错误），参数由此处的签名确定。
    ageMs = millis() - latest.receivedAtMs;  // 将 millis() - latest.receivedAtMs 写入或初始化 ageMs（样本年龄毫秒）。
    if (!initialized) return "INIT";  // 若 !initialized，立即返回 "INIT"。
    if (!online) return "OFFLINE";  // 若 !online，立即返回 "OFFLINE"。
    if (!valid) return "NO_DATA";  // 若 !valid，立即返回 "NO_DATA"。
    return ageMs > 200 ? "STALE" : nullptr;  // 结束当前函数并返回 ageMs > 200 ? "STALE" : nullptr。
}  // 结束此函数或控制块。
}  // 结束命名空间。

/**
 * @brief 将当前有效原始Yaw绑定到已知车体地图航向。
 * @param mapHeading 已确认的地图航向(deg)，0~360。
 * @return nullptr 成功，否则为不可标定原因。
 */
const char *H30UsbCalibrateHeading(float mapHeading,bool keepReferenceOnFailure) {  // 定义函数或成员 的入口 H30UsbCalibrateHeading（H30UsbCalibrate航向），参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    uint32_t age = 0;  // 将 0 写入或初始化 age（样本年龄）。
    const char *error = headingSampleError(age);  // 将 headingSampleError(age) 写入或初始化 error（错误）。
    H30HeadingReference reference;  // 声明 reference，类型和可用范围由所在声明决定。
    if (!error && !reference.calibrate(latest.yaw, mapHeading)) error = "RANGE";  // 当 !error && !reference.calibrate(latest.yaw, mapHeading) 成立时进入此分支，否则继续后续流程。
    if (!error || !keepReferenceOnFailure) {  // 当 !error || !keepReferenceOnFailure 成立时进入此分支，否则继续后续流程。
        headingReference=reference;  // 将 reference 写入或初始化 headingReference（航向Reference）。
        ++headingReferenceVersion;  // 续写当前声明或运算，提供 ++headingReferenceVersion;。
    }  // 结束此条件分支。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    return error;  // 结束当前函数并返回 error。
}  // 结束此函数或控制块。

/**
 * @brief 读取标定后的地图航向及数据年龄，不驱动电机；成功返回 nullptr。
 */
const char *H30UsbGetHeading(float &heading, uint32_t &ageMs) {  // 定义函数或成员 的入口 H30UsbGetHeading（H30UsbGet航向），参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    const char *error = headingSampleError(ageMs);  // 将 headingSampleError(ageMs) 写入或初始化 error（错误）。
    if (!error && !headingReference.get(latest.yaw, heading)) error = "NOT_CALIBRATED";  // 当 !error && !headingReference.get(latest.yaw, heading) 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    return error;  // 结束当前函数并返回 error。
}  // 结束此函数或控制块。

/**
 * @brief 建立地图航向参考并输出REFERENCE事件；heading为已确认的车体航向(deg)。
 */
void CalibrateH30MapHeading(float heading) {  // 定义函数或成员 的入口 CalibrateH30MapHeading（CalibrateH30Map航向），参数由此处的签名确定。
    const char *error = H30UsbCalibrateHeading(heading);  // 将 H30UsbCalibrateHeading(heading) 写入或初始化 error（错误）。
    if (error) Serial.printf("{EVT,IMU,REFERENCE,ERR,%s}\n", error);  // 当 error 成立时进入此分支，否则继续后续流程。
    else Serial.printf("{EVT,IMU,REFERENCE,OK,%.3f}\n", heading);  // 前一个判断不成立时执行此分支。
}  // 结束此函数或控制块。

/**
 * @brief 读取同一快照中的地图航向、采样毫秒数和参考代次，供闭环判断新鲜度及重标定。
 */
const char *H30UsbGetHeadingSnapshot(float &heading,uint32_t &sampleMs,uint32_t &version) {  // 定义函数或成员 的入口 H30UsbGetHeadingSnapshot（H30UsbGet航向Snapshot），参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    uint32_t age=0;  // 将 0 写入或初始化 age（样本年龄）。
    const char *error=headingSampleError(age);  // 将 headingSampleError(age) 写入或初始化 error（错误）。
    sampleMs=latest.receivedAtMs;version=headingReferenceVersion;  // 将 latest.receivedAtMs 写入或初始化 sampleMs（采样毫秒）。然后将 headingReferenceVersion 写入或初始化 version（版本）。
    if(!error&&!headingReference.get(latest.yaw,heading))error="NOT_CALIBRATED";  // 当 !error&&!headingReference.get(latest.yaw,heading) 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    return error;  // 结束当前函数并返回 error。
}  // 结束此函数或控制块。

/**
 * @brief 读取同一帧完整惯性数据、地图航向及参考代次；停稳检查需要加速度和角速度字段。
 */
const char *H30UsbGetMotionSnapshot(H30Attitude &sample,float &heading,uint32_t &version) {  // 定义函数或成员 的入口 H30UsbGetMotionSnapshot，参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    uint32_t age=0;const char *error=headingSampleError(age);  // 将 0 写入或初始化 age（样本年龄）。然后将 headingSampleError(age) 写入或初始化 error（错误）。
    sample=latest;version=headingReferenceVersion;  // 将 latest 写入或初始化 sample（采样）。然后将 headingReferenceVersion 写入或初始化 version（版本）。
    if(!error&&(sample.fields&7)!=7)error="MOTION_FIELDS_REQUIRED";  // 当 !error&&(sample.fields&7)!=7 成立时进入此分支，否则继续后续流程。
    if(!error&&!headingReference.get(sample.yaw,heading))error="NOT_CALIBRATED";  // 当 !error&&!headingReference.get(sample.yaw,heading) 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&mux);return error;  // 离开临界区，保护共享状态的本段访问。然后结束当前函数并返回 error。
}  // 结束此函数或控制块。

void H30UsbBeginHeadingAverage() {  // 定义函数或成员 的入口 H30UsbBeginHeadingAverage（H30UsbBegin航向Average），参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    headingAverage.begin(millis(),HEADING_SETTLE_MS);  // 调用 headingAverage.begin，传入 millis(),HEADING_SETTLE_MS。
    headingAverageReady=false;  // 将 false 写入或初始化 headingAverageReady（航向Average就绪）。
    headingAverageActive=headingAverageSettling=true;  // 将 headingAverageSettling=true 写入或初始化 headingAverageActive（航向Average活跃）。
    headingReference.reset();++headingReferenceVersion;  // 调用 headingReference.reset。然后续写当前声明或运算，提供 ++headingReferenceVersion;。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    Serial.printf("{EVT,IMU,REFERENCE_SETTLE,BEGIN,%lu}\n",static_cast<unsigned long>(HEADING_SETTLE_MS));  // 调用 Serial.printf，传入 "{EVT,IMU,REFERENCE_SETTLE,BEGIN,%lu}\n",static_cast<u…。
}  // 结束此函数或控制块。
int H30UsbPollHeadingAverage() {  // 定义函数或成员 的入口 H30UsbPollHeadingAverage（H30UsbPoll航向Average），参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    const bool collecting=headingAverageSettling&&headingAverage.collecting(millis());  // 将 headingAverageSettling&&headingAverage.collecting(mill… 写入或初始化 collecting。
    if(collecting)headingAverageSettling=false;  // 当 collecting 成立时进入此分支，否则继续后续流程。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    if(collecting)Serial.println("{EVT,IMU,REFERENCE_COLLECT,BEGIN,3000}");  // 当 collecting 成立时进入此分支，否则继续后续流程。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    if(!headingAverage.elapsed(millis())){portEXIT_CRITICAL(&mux);return 0;}  // 当 !headingAverage.elapsed(millis()) 成立时进入此分支，否则继续后续流程。
    headingAverageActive=false;  // 将 false 写入或初始化 headingAverageActive（航向Average活跃）。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    // 开局任务独占采样结果，关闭USB写入后在临界区外排序和求平均。
    const auto result=headingAverage.result();  // 将 headingAverage.result() 写入或初始化 result（结果）。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    uint32_t age=0;  // 将 0 写入或初始化 age（样本年龄）。
    const char *sourceError=headingSampleError(age);  // 将 headingSampleError(age) 写入或初始化 sourceError（source错误）。
    const bool ok=!sourceError&&result.count;  // 将 !sourceError&&result.count 写入或初始化 ok。
    headingAverageReady=ok;  // 将 ok 写入或初始化 headingAverageReady（航向Average就绪）。
    headingAverageYaw=result.yaw;  // 将 result.yaw 写入或初始化 headingAverageYaw（航向Average偏航）。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    Serial.printf("{EVT,IMU,REFERENCE_COLLECT,END,%lu,%.3f}\n",static_cast<unsigned long>(result.count),result.yaw);  // 调用 Serial.printf，传入 "{EVT,IMU,REFERENCE_COLLECT,END,%lu,%.3f}\n",static_ca…。
    if(!ok)Serial.printf("{EVT,IMU,REFERENCE,ERR,%s}\n",sourceError?sourceError:"NO_DATA");  // 当 !ok 成立时进入此分支，否则继续后续流程。
    return ok?1:-1;  // 结束当前函数并返回 ok?1:-1。
}  // 结束此函数或控制块。
void H30UsbBindHeadingAverage(float mapHeading) {  // 定义函数或成员 的入口 H30UsbBindHeadingAverage（H30UsbBind航向Average），参数由此处的签名确定。
    portENTER_CRITICAL(&mux);  // 进入临界区，保护共享状态的本段访问。
    uint32_t age=0;  // 将 0 写入或初始化 age（样本年龄）。
    const char *error=headingSampleError(age);  // 将 headingSampleError(age) 写入或初始化 error（错误）。
    const bool ok=headingAverageReady&&!error&&headingReference.calibrate(headingAverageYaw,mapHeading);  // 将 headingAverageReady&&!error&&headingReference.calibrat… 写入或初始化 ok。
    if(ok)++headingReferenceVersion;  // 当 ok 成立时进入此分支，否则继续后续流程。
    headingAverageReady=false;  // 将 false 写入或初始化 headingAverageReady（航向Average就绪）。
    portEXIT_CRITICAL(&mux);  // 离开临界区，保护共享状态的本段访问。
    if(ok)Serial.printf("{EVT,IMU,REFERENCE,OK,%.3f}\n",mapHeading);  // 当 ok 成立时进入此分支，否则继续后续流程。
}  // 结束此函数或控制块。
bool H30UsbHeadingAverageActive() {  // 定义函数或成员 的入口 H30UsbHeadingAverageActive（H30Usb航向Average活跃），参数由此处的签名确定。
    portENTER_CRITICAL(&mux);const bool active=headingAverageActive;portEXIT_CRITICAL(&mux);return active;  // 进入临界区，保护共享状态的本段访问。然后将 headingAverageActive 写入或初始化 active（活跃）。然后离开临界区，保护共享状态的本段访问。然后结束当前函数并返回 active。
}  // 结束此函数或控制块。
