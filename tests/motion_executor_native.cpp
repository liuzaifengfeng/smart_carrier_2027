#include "competition_flow.h"
#include "motion_sequence.h"
#include <assert.h>
#include <map>
#include <vector>
#include <string.h>

struct SequenceIO : motion::SequenceIO {
    uint32_t time=0;bool stop=false,measured=false,readyNow=false;
    int issued=0,commits=0,begins=0,ends=0;
    uint32_t now()override{return time;}
    bool cancelled()override{return stop;}
    const char *issue(const motion::Step &s)override{++issued;if(s.kind==motion::StepKind::COMMIT)++commits;return nullptr;}
    const char *ready(const motion::Step &,bool &done)override{done=readyNow;return nullptr;}
    bool feedback(const motion::Step &s)override{return measured&&s.kind==motion::StepKind::ARM;}
    void event(size_t,const motion::Step &,bool begin)override{if(begin)++begins;else ++ends;}
};
void sequence() {
    motion::Program p;motion::Step move;move.kind=motion::StepKind::ARM;move.waitMs=1000;move.holdMs=50;p.append(move);
    motion::Step commit;commit.kind=motion::StepKind::COMMIT;p.append(commit);
    SequenceIO io;motion::Sequence arm;arm.begin(p,io);arm.tick();assert(io.issued==1&&!io.commits);
    int chassisTicks=0;
    for(int i=0;i<100;++i){io.time+=5;++chassisTicks;arm.tick();assert(io.issued==1&&!io.commits);}
    assert(chassisTicks==100&&arm.active());
    io.time=1000;arm.tick();assert(io.commits==0);arm.tick();assert(io.commits==1);arm.tick();assert(!arm.active()&&io.begins==io.ends);
    // 反馈模式不会把目标缓存或原固定时长当成实际到位。
    io=SequenceIO{};io.measured=true;arm.begin(p,io);arm.tick();
    io.time=5000;arm.tick();assert(!io.commits&&arm.active());
    io.readyNow=true;io.time=5050;arm.tick();assert(!io.commits);
    io.time=5099;arm.tick();assert(!io.commits);io.time=5100;arm.tick();arm.tick();assert(io.commits==1);
    // 停止或 OTA 取消后，即使时钟继续推进也不接续 COMMIT。
    io=SequenceIO{};arm.begin(p,io);arm.tick();io.stop=true;arm.tick();io.stop=false;io.time=20000;
    for(int i=0;i<20;++i)arm.tick();assert(!io.commits&&!arm.active()&&arm.failure());
}
void join() {
    motion::Join j;int starts=0;
    for(int i=0;i<10;++i)starts+=j.shouldStartArm(false,true,10,0);assert(starts==0);
    starts+=j.shouldStartArm(false,false,10,30);assert(starts==0);
    for(int i=0;i<20;++i)starts+=j.shouldStartArm(false,true,10,30);assert(starts==1);
    j.armReady=true;assert(!j.complete());j.shouldStartArm(true,false,0,30);assert(j.complete());
    j=motion::Join{};assert(j.shouldStartArm(true,false,0,0));assert(!j.complete());j.armReady=true;assert(j.complete());
}
void launch() {
    motion::LaunchGate gate;const uint32_t generation=gate.generation;gate.prepare(7,false);
    assert(!gate.runnable(7,generation,false,false)); // ACK 前不能运行。
    gate.release(7);assert(gate.runnable(7,generation,false,false));
    gate.cancel();assert(gate.cancelled(generation));
    gate.prepare(8,false);assert(gate.runnable(7,generation,false,false)); // 旧作业只能被消费为取消。
    assert(!gate.runnable(8,gate.generation,false,false));gate.release(8);assert(!gate.cancelled(gate.generation));
    motion::Ticket ticket;MotionRequest r;int submissions=0;
    for(int i=0;i<100;++i)assert(ticket.submitOnce(r,[&](const MotionRequest &){++submissions;return 42;}));
    assert(submissions==1&&ticket.id==42);
}
struct Runtime : CompetitionIO {
    RobotState current=STATE_WAIT_START;
    uint32_t time=0,nextId=0,pending=0;
    uint8_t startZone=0;bool runPermission=false,abort=false,material=false,taskPublished=false,finished=false;
    int progressValue=0,starts=0,visionPolls=0,materialStarts=0;
    AutoAlignmentState alignment=AutoAlignmentState::WAITING;
    std::vector<MotionRequest> requests;
    std::vector<VisionStartMode> vision;
    std::map<uint32_t,MotionStatus> jobs;
    uint32_t now()override{return time;}
    RobotState state()override{return current;}
    void state(RobotState value)override{current=value;}
    bool aborted()override{return abort;}
    uint8_t zone()override{return startZone;}
    bool permission()override{return runPermission;}
    int progress()override{return progressValue;}
    void clearProgress()override{progressValue=0;}
    void task(MotionRequest &r,bool second)override{
        const int colors[3]={second?6:1,second?1:6,second?6:1},positions[3]={3,1,2};
        for(int i=0;i<3;++i){r.colors[i]=colors[i];r.positions[i]=positions[i];}
    }
    uint32_t submit(const MotionRequest &r)override {
        assert(!pending);requests.push_back(r);pending=++nextId;
        MotionStatus s;s.id=pending;s.state=MotionState::QUEUED;jobs[pending]=s;return pending;
    }
    uint32_t lidar()override{MotionRequest r;r.kind=MotionKind::LIDAR_POSE;return submit(r);}
    MotionStatus status(uint32_t id)override{return jobs.at(id);}
    void visionStart(VisionStartMode mode)override{
        assert(!pending&&jobs.at(nextId).state==MotionState::DONE);
        if(mode==VisionStartMode::DISC||mode==VisionStartMode::WORK_AREA||mode==VisionStartMode::WORK_AREA_LOADED)
            assert(requests.back().kind==MotionKind::NAVIGATE&&jobs.at(nextId).chassisDone&&jobs.at(nextId).armReady);
        vision.push_back(mode);alignment=AutoAlignmentState::WAITING;visionPolls=0;
    }
    AutoAlignmentState visionStatus()override{return alignment;}
    void visionStop(VisionStartMode)override{}
    void materialStart()override{assert(!pending);assert(requests.back().observation==ArmObservation::LOOK3);material=true;++materialStarts;}
    void materialStop()override{material=false;}
    void taskReady()override{taskPublished=true;}
    void runStart()override{++starts;}
    void runDone()override{finished=true;}
    void fail(RobotState reason)override{current=reason;}
    void display(const char *)override{}
    void complete(bool success=true){assert(pending);auto &s=jobs.at(pending);s.state=success?MotionState::DONE:MotionState::FAILED;s.chassisDone=s.armReady=success;pending=0;}
};
void polling() {
    Runtime r;CompetitionFlow flow(r,CompetitionMap{});
    flow.tick();for(int i=0;i<30;++i)flow.tick();assert(r.requests.size()==1&&r.current==STATE_WAIT_START);
    r.complete();flow.tick();for(int i=0;i<30;++i)flow.tick();assert(r.requests.size()==1);
    r.startZone=1;flow.tick();flow.tick();assert(r.requests.size()==2);for(int i=0;i<30;++i)flow.tick();assert(r.requests.size()==2);
    r.complete();flow.tick();for(int i=0;i<30;++i)flow.tick();assert(r.starts==0);
    r.runPermission=true;flow.tick();flow.tick();assert(r.current==STATE_READ_TASK&&r.starts==1);
    r.jobs.at(r.pending).state=MotionState::RUNNING;for(int i=0;i<30;++i)flow.tick();assert(r.current==STATE_READ_TASK&&!r.taskPublished);
    r.complete();flow.tick();assert(r.current==STATE_GRAB_ROUND1&&r.taskPublished);
}
void fullFlow(uint8_t zone,MotionKind failed=MotionKind::VELOCITY,bool failVision=false,bool cancel=false) {
    Runtime r;r.startZone=zone;r.runPermission=true;CompetitionFlow flow(r,CompetitionMap{});
    bool failureInjected=false;int discGrabs=0,recognitions=0,delay=0;
    for(int tick=0;tick<1500&&!r.finished;++tick) {
        r.time+=10;
        if(r.pending&&++delay>=3) {
            const bool reject=!failureInjected&&r.requests.back().kind==failed;
            if(reject)failureInjected=true;
            if(cancel&&!failureInjected&&r.requests.back().kind==MotionKind::PLACE_ROUND){r.current=STATE_TIMEOUT_FAILED;r.abort=true;failureInjected=true;}
            r.complete(!reject);delay=0;
        }
        if(!r.vision.empty()&&r.alignment==AutoAlignmentState::WAITING&&++r.visionPolls>=3)
            r.alignment=failVision?AutoAlignmentState::FAILED:AutoAlignmentState::DONE;
        if(r.material&&!r.pending&&r.progressValue<3) {
            if(++recognitions>=3){++r.progressValue;++discGrabs;recognitions=0;r.material=false;}
        }
        flow.tick();
        if(r.current==STATE_SCAN_FAILED||r.current==STATE_ROUTE_FAILED||r.current==STATE_ALIGN_FAILED||
            r.current==STATE_TRANSFER_FAILED||r.current==STATE_TIMEOUT_FAILED) {
            const size_t count=r.requests.size();for(int i=0;i<100;++i)flow.tick();assert(count==r.requests.size());
            assert(failureInjected||failVision);assert(!r.finished);return;
        }
        if(tick>20)r.startZone=zone==1?2:1; // 返航必须使用开局锁定区号。
    }
    assert(failed==MotionKind::VELOCITY&&!failVision&&!cancel);
    assert(r.finished&&r.current==STATE_DONE&&discGrabs==6&&r.starts==1&&r.materialStarts==6);
    assert(r.vision==std::vector<VisionStartMode>({VisionStartMode::DISC,VisionStartMode::WORK_AREA,VisionStartMode::WORK_AREA,
        VisionStartMode::DISC,VisionStartMode::WORK_AREA,VisionStartMode::WORK_AREA_LOADED,VisionStartMode::CORNER}));
    int paths=0,places=0,retrieves=0;
    MotionKind previous=MotionKind::ARM_POSE;
    for(const auto &request:r.requests) {
        if(request.kind==MotionKind::NAVIGATE){++paths;if(request.endNode==0||request.endNode==4)assert(request.endNode==(zone==1?4:0));}
        if(request.kind==MotionKind::NAVIGATE&&request.startNode==10)assert(previous==MotionKind::RETRIEVE_ROUND);
        if(request.kind==MotionKind::PLACE_ROUND){++places;assert(request.positions[0]==3&&request.positions[1]==1&&request.positions[2]==2);}
        if(request.kind==MotionKind::PLACE_ROUND)assert(request.layer==(places==4?2:1));
        if(request.kind==MotionKind::RETRIEVE_ROUND)++retrieves;
        if(request.kind==MotionKind::HOME_RETURN||request.kind==MotionKind::HOME_OBSERVE)assert(request.zone==zone);
        previous=request.kind;
    }
    assert(paths==7&&places==4&&retrieves==2);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(!strcmp(argv[1],"sequence"))sequence();
    else if(!strcmp(argv[1],"join"))join();
    else if(!strcmp(argv[1],"launch"))launch();
    else if(!strcmp(argv[1],"polling"))polling();
    else if(!strcmp(argv[1],"rounds")){fullFlow(1);fullFlow(2);}
    else if(!strcmp(argv[1],"failures")) {
        for(auto kind:{MotionKind::ARM_POSE,MotionKind::SCAN_TASK,MotionKind::NAVIGATE,MotionKind::PLACE_ROUND,
            MotionKind::RETRIEVE_ROUND,MotionKind::HOME_OBSERVE,MotionKind::HOME_RETURN})fullFlow(1,kind);
    } else if(!strcmp(argv[1],"vision"))fullFlow(1,MotionKind::VELOCITY,true);
    else if(!strcmp(argv[1],"cancel"))fullFlow(1,MotionKind::VELOCITY,false,true);
    else assert(false);
}
