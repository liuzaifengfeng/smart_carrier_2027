#include "h30_usb.h"
#include "h30_heading.h"
#include <Arduino.h>
#include "usb/usb_host.h"

namespace {
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
H30Attitude latest;
H30HeadingReference headingReference; // 与 latest 共用 mux，标定与采样原子一致
uint32_t headingReferenceVersion=0;
bool initialized = false, online = false, valid = false;
usb_host_client_handle_t client = nullptr;
usb_device_handle_t device = nullptr;
usb_transfer_t *rx = nullptr, *control = nullptr;
int commIf = -1, dataIf = -1;
bool commClaimed = false, dataClaimed = false;
uint8_t endpoint = 0;
bool rxPending = false, controlPending = false, stopping = false;
bool controlDone = false, controlOK = false;
int stage = 0;
uint32_t controlStarted = 0, retryAt = 0;
H30Parser parser;

void invalidate() {
    portENTER_CRITICAL(&mux);
    online = false;
    valid = false;
    headingReference.reset();
    ++headingReferenceVersion;
    portEXIT_CRITICAL(&mux);
    parser.reset();
}

void events(const usb_host_client_event_msg_t *event, void *) {
    if (event->event == USB_HOST_CLIENT_EVENT_DEV_GONE
            && event->dev_gone.dev_hdl == device) {
        stopping = true;
        invalidate();
    }
}

void received(usb_transfer_t *transfer) {
    rxPending = false;
    if (stopping) return;
    if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        stopping = true;
        invalidate();
        return;
    }
    // 最多处理 512 字节，无日志、阻塞等待或业务计算，无需另设字节队列。
    for (int i = 0; i < transfer->actual_num_bytes; ++i) {
        H30Attitude sample;
        if (!parser.feed(transfer->data_buffer[i], sample)) continue;
        sample.receivedAtMs = millis();
        portENTER_CRITICAL(&mux);
        // 输出中断可能伴随模块复位或航向跳变；恢复数据后仍需重新标定。
        if (valid && sample.receivedAtMs - latest.receivedAtMs > 200) {
            headingReference.reset();++headingReferenceVersion;
        }
        latest = sample;
        valid = true;
        portEXIT_CRITICAL(&mux);
    }
    if (usb_host_transfer_submit(rx) == ESP_OK) rxPending = true;
    else { stopping = true; invalidate(); }
}

void controlled(usb_transfer_t *transfer) {
    controlPending = false;
    controlDone = true;
    controlOK = transfer->status == USB_TRANSFER_STATUS_COMPLETED;
}

bool submitControl(bool lineCoding) {
    memset(control->data_buffer, 0, 15);
    uint8_t *p = control->data_buffer;
    p[0] = 0x21;
    p[1] = lineCoding ? 0x20 : 0x22;
    p[2] = lineCoding ? 0 : 3;
    p[4] = commIf;
    p[6] = lineCoding ? 7 : 0;
    if (lineCoding) {
        const uint8_t coding[] = {0x00, 0x08, 0x07, 0x00, 0x00, 0x00, 0x08};
        memcpy(p + 8, coding, sizeof(coding));
    }
    control->device_handle = device;
    control->bEndpointAddress = 0;
    control->callback = controlled;
    control->num_bytes = lineCoding ? 15 : 8;
    controlDone = false;
    if (usb_host_transfer_submit_control(client, control) != ESP_OK) return false;
    controlPending = true;
    controlStarted = millis();
    return true;
}

bool openDevice(uint8_t address) {
    if (usb_host_device_open(client, address, &device) != ESP_OK) return false;
    const usb_device_desc_t *desc = nullptr;
    if (usb_host_get_device_descriptor(device, &desc) != ESP_OK
            || desc->idVendor != 0x1A86 || desc->idProduct != 0x55D4) return false;
    const usb_config_desc_t *config = nullptr;
    if (usb_host_get_active_config_descriptor(device, &config) != ESP_OK) return false;
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(config);
    int currentIf = -1, currentClass = -1;
    uint16_t packetSize = 0;
    for (size_t pos = 0; pos + 2 <= config->wTotalLength;) {
        const uint8_t len = bytes[pos], type = bytes[pos + 1];
        if (len < 2 || pos + len > config->wTotalLength) return false;
        const uint8_t *p = bytes + pos;
        if (type == 4 && len >= 9) {
            currentIf = p[3] == 0 ? p[2] : -1; // 只使用默认 alternate setting
            currentClass = p[5];
            if (currentIf >= 0 && currentClass == 2 && p[6] == 2) commIf = currentIf;
        } else if (type == 5 && len >= 7 && currentIf >= 0 && currentClass == 0x0A
                && (p[2] & 0x80) && (p[3] & 3) == 2) {
            dataIf = currentIf;
            endpoint = p[2];
            packetSize = p[4] | (uint16_t(p[5]) << 8);
        }
        pos += len;
    }
    if (commIf < 0 || dataIf < 0 || !packetSize || packetSize > 512) return false;
    if (usb_host_interface_claim(client, device, commIf, 0) != ESP_OK) return false;
    commClaimed = true;
    if (usb_host_interface_claim(client, device, dataIf, 0) != ESP_OK) return false;
    dataClaimed = true;
    if (usb_host_transfer_alloc(512, 0, &rx) != ESP_OK
            || usb_host_transfer_alloc(15, 0, &control) != ESP_OK) return false;
    rx->device_handle = device;
    rx->bEndpointAddress = endpoint;
    rx->num_bytes = (512 / packetSize) * packetSize;
    rx->callback = received;
    stage = 1;
    return submitControl(true);
}

void closeDevice() {
    invalidate();
    // 回调仍可能引用传输对象，必须先取消并通过客户端事件循环收回。
    if (rxPending) {
        usb_host_endpoint_halt(device, endpoint);
        usb_host_endpoint_flush(device, endpoint);
    }
    // EP0 由 Host 栈管理，控制超时后保留其对象直到完成/拔出回调。
    // 不释放在途对象，也不因外设异常重启整机。
    if (rxPending || controlPending) return;
    usb_host_transfer_free(rx); rx = nullptr;
    usb_host_transfer_free(control); control = nullptr;
    if (dataClaimed) usb_host_interface_release(client, device, dataIf);
    if (commClaimed) usb_host_interface_release(client, device, commIf);
    if (device) usb_host_device_close(client, device);
    device = nullptr;
    commIf = dataIf = -1;
    commClaimed = dataClaimed = false;
    endpoint = 0;
    stage = 0;
    controlDone = stopping = false;
    retryAt = millis() + 1000;
}

void libraryTask(void *) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (;;) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

void clientTask(void *) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (;;) {
        usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
        if (device && controlPending && millis() - controlStarted > 3000) {
            stopping = true;
            invalidate();
        }
        if (stopping) { closeDevice(); continue; }
        if (device && controlDone) {
            controlDone = false;
            if (!controlOK) stopping = true;
            else if (stage == 1) {
                stage = 2;
                if (!submitControl(false)) stopping = true;
            } else if (stage == 2) {
                stage = 3;
                if (usb_host_transfer_submit(rx) == ESP_OK) {
                    rxPending = true;
                    portENTER_CRITICAL(&mux);
                    online = true;
                    portEXIT_CRITICAL(&mux);
                } else stopping = true;
            }
            if (stopping) invalidate();
        }
        // 枚举后轮询地址，也覆盖任务启动前已插入的设备以及出错后的重试。
        if (!device && int32_t(millis() - retryAt) >= 0) {
            retryAt = millis() + 1000;
            uint8_t addresses[8];
            int count = 0;
            if (usb_host_device_addr_list_fill(8, addresses, &count) != ESP_OK) continue;
            for (int i = 0; i < count; ++i) {
                if (openDevice(addresses[i])) break;
                if (device) { stopping = true; closeDevice(); }
            }
        }
    }
}
} // namespace

bool H30UsbBegin() {
    if (initialized) return true;
    usb_host_config_t config = {};
    config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    if (usb_host_install(&config) != ESP_OK) return false;
    usb_host_client_config_t clientConfig = {};
    clientConfig.max_num_event_msg = 8;
    clientConfig.async.client_event_callback = events;
    if (usb_host_client_register(&clientConfig, &client) != ESP_OK) {
        usb_host_uninstall();
        return false;
    }
    // 任务先等待通知，两个核心上都不会在初始化完成前处理 USB。
    TaskHandle_t libTask = nullptr, ioTask = nullptr;
    const bool ok = xTaskCreate(libraryTask, "H30Host", 4096, nullptr, 4, &libTask) == pdPASS
        && xTaskCreate(clientTask, "H30Client", 6144, nullptr, 4, &ioTask) == pdPASS;
    if (!ok) {
        if (libTask) vTaskDelete(libTask);
        if (ioTask) vTaskDelete(ioTask);
    }
    portENTER_CRITICAL(&mux);
    initialized = ok;
    portEXIT_CRITICAL(&mux);
    if (ok) {
        xTaskNotifyGive(libTask);
        xTaskNotifyGive(ioTask);
    }
    if (!ok) {
        usb_host_client_deregister(client);
        client = nullptr;
        usb_host_device_free_all();
        usb_host_uninstall();
    }
    return ok;
}

const char *H30UsbGetAttitude(H30Attitude &sample, uint32_t &ageMs) {
    portENTER_CRITICAL(&mux);
    const bool ready = initialized, connected = online, hasSample = valid;
    sample = latest;
    portEXIT_CRITICAL(&mux);
    ageMs = millis() - sample.receivedAtMs;
    if (!ready) return "INIT";
    if (!connected) return "OFFLINE";
    if (!hasSample) return "NO_DATA";
    return ageMs > 200 ? "STALE" : nullptr;
}

namespace {
// 调用者必须持有 mux；标定与查询采用完全相同的新鲜度规则。
const char *headingSampleError(uint32_t &ageMs) {
    ageMs = millis() - latest.receivedAtMs;
    if (!initialized) return "INIT";
    if (!online) return "OFFLINE";
    if (!valid) return "NO_DATA";
    return ageMs > 200 ? "STALE" : nullptr;
}
}

const char *H30UsbCalibrateHeading(float mapHeading) {
    portENTER_CRITICAL(&mux);
    headingReference.reset();
    ++headingReferenceVersion;
    uint32_t age = 0;
    const char *error = headingSampleError(age);
    if (!error && !headingReference.calibrate(latest.yaw, mapHeading)) error = "RANGE";
    portEXIT_CRITICAL(&mux);
    return error;
}

const char *H30UsbGetHeading(float &heading, uint32_t &ageMs) {
    portENTER_CRITICAL(&mux);
    const char *error = headingSampleError(ageMs);
    if (!error && !headingReference.get(latest.yaw, heading)) error = "NOT_CALIBRATED";
    portEXIT_CRITICAL(&mux);
    return error;
}

void CalibrateH30MapHeading(float heading) {
    const char *error = H30UsbCalibrateHeading(heading);
    if (error) Serial.printf("{EVT,IMU,REFERENCE,ERR,%s}\n", error);
    else Serial.printf("{EVT,IMU,REFERENCE,OK,%.3f}\n", heading);
}

const char *H30UsbGetHeadingSnapshot(float &heading,uint32_t &sampleMs,uint32_t &version) {
    portENTER_CRITICAL(&mux);
    uint32_t age=0;
    const char *error=headingSampleError(age);
    sampleMs=latest.receivedAtMs;version=headingReferenceVersion;
    if(!error&&!headingReference.get(latest.yaw,heading))error="NOT_CALIBRATED";
    portEXIT_CRITICAL(&mux);
    return error;
}
