#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string>
struct NativeSerial {
    std::string text;
    void println(const char *value){text+=value;text+='\n';}
    int available(){return 0;}
    int read(){return -1;}
    void printf(const char *format,...) {
        char buffer[1024];va_list args;va_start(args,format);
        vsnprintf(buffer,sizeof(buffer),format,args);va_end(args);text+=buffer;
    }
};
extern NativeSerial Serial;
inline void delay(uint32_t) {}
inline uint32_t pdMS_TO_TICKS(uint32_t ms){return ms;}
void vTaskDelay(uint32_t ticks);
