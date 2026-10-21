#pragma once
#include "robot_state.h"
#include "motion_job.h"
#include "vision_alignment.h"
#include <string.h>

struct CompetitionMap {
    uint8_t scan=2,disc=14,coarse=10,temp=22,home1=4,home2=0;
    float homeHeading=180;
};
struct CompetitionIO {
    virtual ~CompetitionIO()=default;
    virtual uint32_t now()=0;
    virtual RobotState state()=0;
    virtual void state(RobotState value)=0;
    virtual bool aborted()=0;
    virtual uint8_t zone()=0;
    virtual bool permission()=0;
    virtual int progress()=0;
    virtual void clearProgress()=0;
    virtual void task(MotionRequest &request,bool secondRound)=0;
    virtual uint32_t submit(const MotionRequest &request)=0;
    virtual uint32_t lidar()=0;
    virtual MotionStatus status(uint32_t id)=0;
    virtual void visionStart(VisionStartMode mode)=0;
    virtual AutoAlignmentState visionStatus()=0;
    virtual void visionStop(VisionStartMode mode)=0;
    virtual void materialStart()=0;
    virtual void materialStop()=0;
    virtual void taskReady()=0;
    virtual void runStart()=0;
    virtual void runDone()=0;
    virtual void fail(RobotState reason)=0;
    virtual void display(const char *text)=0;
};
class CompetitionFlow {
    CompetitionIO &io;
    CompetitionMap map;
    motion::Ticket ticket;
    RobotState observed=STATE_WAIT_START;
    uint8_t step=0,homeZone=0;
    int preparedProgress=0;
    uint32_t due=0;
    void next(RobotState state){if(io.aborted()||io.state()!=observed)return;io.state(state);observed=state;step=0;ticket.clear();}
    int motion(const MotionRequest &r,RobotState failure=STATE_TRANSFER_FAILED) {
        if(!ticket.submitOnce(r,[&](const MotionRequest &value){return io.submit(value);})) {io.fail(failure);return -1;}
        const auto s=io.status(ticket.id);
        if(io.aborted()||io.state()!=observed)return -1;
        if(!s.complete())return 0;
        ticket.clear();
        if(s.state!=MotionState::DONE){
            if(r.kind==MotionKind::NAVIGATE&&(!strcmp(s.phase,"ARM_PREPARE")||!strcmp(s.phase,"ARM_FAILED")))failure=STATE_TRANSFER_FAILED;
            io.fail(failure);return -1;
        }
        return 1;
    }
    MotionRequest pose(ArmObservation value) {
        MotionRequest r;r.kind=MotionKind::ARM_POSE;r.observation=value;r.zone=homeZone;return r;
    }
    MotionRequest path(uint8_t from,uint8_t to,ArmObservation observation=ArmObservation::NONE) {
        MotionRequest r;r.kind=MotionKind::NAVIGATE;r.startNode=from;r.endNode=to;r.observation=observation;r.zone=homeZone;r.theta=NAN;return r;
    }
    bool aligned(VisionStartMode mode) {
        const auto result=io.visionStatus();if(result==AutoAlignmentState::WAITING)return false;
        io.visionStop(mode);
        if(result!=AutoAlignmentState::DONE){io.fail(STATE_ALIGN_FAILED);return false;}
        return true;
    }
    void disc(bool second) {
        if(step==0) {
            io.display(second?"GRAB R2":"GRAB R1");
            if(!ticket.id){io.materialStop();io.clearProgress();}
            if(motion(path(second?map.temp:map.scan,map.disc,ArmObservation::LOOK2),STATE_ROUTE_FAILED)==1) {
                io.visionStart(VisionStartMode::DISC);step=1;
            }
        } else if(step==1) {
            if(aligned(VisionStartMode::DISC)){due=io.now()+10;step=2;}
        } else if(step==2) {
            if(static_cast<int32_t>(io.now()-due)<0)return;
            if(motion(pose(ArmObservation::LOOK3))==1){preparedProgress=0;io.materialStart();step=3;}
        } else {
            const int progress=io.progress();
            if(progress>=3){io.materialStop();io.clearProgress();next(second?STATE_PLACE_COARSE2:STATE_PLACE_COARSE1);return;}
            if(progress>preparedProgress&&motion(pose(ArmObservation::LOOK3))==1){preparedProgress=progress;io.materialStart();}
        }
    }
    void coarse(bool second) {
        if(step==0) {
            if(!ticket.id){io.materialStop();io.display(second?"GO COARSE2":"GO COARSE1");}
            if(motion(path(map.disc,map.coarse,ArmObservation::LOOK),STATE_ROUTE_FAILED)==1){io.visionStart(VisionStartMode::WORK_AREA);step=1;}
        } else if(step==1) {
            if(aligned(VisionStartMode::WORK_AREA))step=2;
        } else {
            MotionRequest r;r.kind=MotionKind::PLACE_ROUND;io.task(r,second);r.layer=1;
            io.display(second?"PLACE C2":"PLACE C1");
            if(motion(r)==1){io.clearProgress();next(second?STATE_STACK_TEMP2:STATE_PLACE_TEMP1);}
        }
    }
    void temporary(bool second) {
        const auto mode=second?VisionStartMode::WORK_AREA_LOADED:VisionStartMode::WORK_AREA;
        if(step==0){due=io.now()+10;step=1;io.display(second?"WAIT RETRIEVE C2":"WAIT RETRIEVE C1");return;}
        if(step==1) {
            if(static_cast<int32_t>(io.now()-due)<0)return;
            MotionRequest r;r.kind=MotionKind::RETRIEVE_ROUND;io.task(r,second);
            io.display(second?"RETRIEVE C2":"RETRIEVE C1");
            if(motion(r)==1){io.clearProgress();step=2;}
        } else if(step==2) {
            io.display(second?"GO TEMP2":"GO TEMP1");
            if(motion(path(map.coarse,map.temp,ArmObservation::LOOK),STATE_ROUTE_FAILED)==1){io.visionStart(mode);step=3;}
        } else if(step==3) {
            if(aligned(mode))step=4;
        } else {
            MotionRequest r;r.kind=MotionKind::PLACE_ROUND;io.task(r,second);r.layer=second?2:1;
            io.display(second?"STACK T2":"PLACE T1");
            if(motion(r)==1){io.clearProgress();next(second?STATE_RETURN_HOME:STATE_GRAB_ROUND2);}
        }
    }
    void home() {
        if(step==0) {
            if(!ticket.id)io.materialStop();
            if(motion(pose(ArmObservation::START))==1)step=1;
        } else if(step==1) {
            auto r=path(map.temp,homeZone==1?map.home1:map.home2);r.theta=map.homeHeading;
            if(motion(r,STATE_ROUTE_FAILED)==1)step=2;
        } else if(step==2) {
            MotionRequest r;r.kind=MotionKind::HOME_OBSERVE;r.zone=homeZone;
            if(motion(r)==1){io.visionStart(VisionStartMode::CORNER);step=3;}
        } else if(step==3) {
            if(aligned(VisionStartMode::CORNER))step=4;
        } else {
            MotionRequest r;r.kind=MotionKind::HOME_RETURN;r.zone=homeZone;
            if(motion(r)==1){io.clearProgress();next(STATE_DONE);if(io.state()==STATE_DONE)io.runDone();}
        }
    }
public:
    CompetitionFlow(CompetitionIO &runtime,const CompetitionMap &geometry):io(runtime),map(geometry){}
    void tick() {
        const auto current=io.state();
        if(current!=observed){observed=current;step=0;ticket.clear();}
        if(io.aborted())return;
        switch(current) {
        case STATE_WAIT_START:
            if(step==0){if(motion(pose(ArmObservation::START))==1){step=1;io.display("WAIT start_zone");}}
            else if(step==1){if(io.zone()){homeZone=io.zone();step=2;}}
            else if(step==2) {
                if(!ticket.id)ticket.id=io.lidar();
                if(!ticket.id||io.status(ticket.id).complete()){ticket.clear();step=3;io.display("WAIT START");}
            } else if(io.permission()){io.runStart();next(STATE_READ_TASK);}
            break;
        case STATE_READ_TASK: {
            MotionRequest r;r.kind=MotionKind::SCAN_TASK;r.zone=homeZone;
            io.display("READ TASK");
            if(motion(r,STATE_SCAN_FAILED)==1){io.taskReady();next(STATE_GRAB_ROUND1);}break;
        }
        case STATE_GRAB_ROUND1: disc(false);break;
        case STATE_GRAB_ROUND2: disc(true);break;
        case STATE_PLACE_COARSE1: coarse(false);break;
        case STATE_PLACE_COARSE2: coarse(true);break;
        case STATE_PLACE_TEMP1: temporary(false);break;
        case STATE_STACK_TEMP2: temporary(true);break;
        case STATE_RETURN_HOME: home();break;
        default: break;
        }
    }
};
