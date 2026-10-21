#include "disc_position_grab.h"
#include "motion_executor.h"
int armSpeed=500;
namespace {
bool grab(uint8_t position,uint8_t color,uint8_t cargo) {
    MotionRequest r;r.kind=MotionKind::DISC_GRAB;r.position=position;r.color=color;r.cargo=cargo;
    return WaitMotion(SubmitMotion(r));
}
}
bool GrabDiscPosition1(uint8_t color,uint8_t cargo){return grab(1,color,cargo);}
bool GrabDiscPosition2(uint8_t color,uint8_t cargo){return grab(2,color,cargo);}
bool GrabDiscPosition3(uint8_t color,uint8_t cargo){return grab(3,color,cargo);}
