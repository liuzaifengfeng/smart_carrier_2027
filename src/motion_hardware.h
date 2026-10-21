#pragma once
#include "motion_sequence.h"

class MotionHardware : public motion::SequenceIO {
    uint32_t job=0,turretStarted=0;
    bool (*cancel)()=nullptr;
public:
    void configure(uint32_t id,bool (*cancelled)()){job=id;cancel=cancelled;}
    uint32_t now() override;
    bool cancelled() override;
    const char *issue(const motion::Step &step) override;
    const char *ready(const motion::Step &step,bool &done) override;
    bool feedback(const motion::Step &step) override;
    void event(size_t index,const motion::Step &step,bool begin) override;
};
