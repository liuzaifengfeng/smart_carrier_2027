#include "competition_flow.h"
#include "motion_sequence.h"
#include <assert.h>
#include <map>
#include <vector>
#include <string.h>

struct SequenceIO : motion::SequenceIO {
    uint32_t time=0;bool stop=false,measured=false,readyNow=false;
    int issued=0,commits=0,begins=0,ends=0,missedReads=0;
    uint32_t now()override{return time;}
    bool cancelled()override{return stop;}
    const char *issue(const motion::Step &s)override{++issued;if(s.kind==motion::StepKind::COMMIT)++commits;return nullptr;}
    const char *ready(const motion::Step &,bool &done)override{
        if(missedReads){--missedReads;return "MOTOR_FEEDBACK_TIMEOUT";}
        done=readyNow;return nullptr;
    }
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
    p.append(move);p.append(commit);
    io=SequenceIO{};arm.begin(p,io,2);arm.tick();io.time=1000;arm.tick();arm.tick();arm.tick();
    assert(!arm.active()&&io.issued==2&&io.commits==1);
    io=SequenceIO{};io.measured=io.readyNow=true;io.missedReads=1;
    arm.begin(p,io);arm.tick();arm.tick();
    assert(arm.active()&&!arm.failure()&&io.issued==1&&io.commits==0);
    io.time=50;arm.tick();arm.tick();io.time=100;arm.tick();arm.tick();
    assert(io.issued==2&&io.commits==1);
    io=SequenceIO{};io.measured=true;io.missedReads=2;
    arm.begin(p,io);arm.tick();arm.tick();io.time=50;arm.tick();
    assert(arm.failure()&&!arm.active()&&io.issued==1&&!io.commits);
}
void join() {
    motion::Join j;int starts=0;
    for(int i=0;i<10;++i)starts+=j.shouldStartArm(false,false);assert(starts==0);
    for(int i=0;i<20;++i)starts+=j.shouldStartArm(false,true);assert(starts==1);
    j.armReady=true;assert(!j.complete());j.shouldStartArm(true,false);assert(j.complete());
    j=motion::Join{};assert(j.shouldStartArm(true,false));assert(!j.complete());j.armReady=true;assert(j.complete());
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
    bool calibrationStarted=false,calibrationReady=true;
    void beginHeadingCalibration()override{assert(!pending);calibrationStarted=true;}
    bool headingCalibrationDone()override{return calibrationReady;}
    bool permission()override{return runPermission;}
    int progress()override{return progressValue;}
    void clearProgress()override{progressValue=0;}
    void task(MotionRequest &r,bool second)override{
        const int colors[3]={second?5:1,second?1:5,6},positions[3]={second?2:1,second?3:2,second?1:3};
        for(int i=0;i<3;++i){r.colors[i]=colors[i];r.positions[i]=positions[i];}
    }
    uint32_t submit(const MotionRequest &r)override {
        if(r.discDeparture)assert(!material&&r.kind==MotionKind::NAVIGATE&&r.startNode==14);
        if(r.kind==MotionKind::SCAN_TASK)assert(r.startNode==(startZone==1?4:0)&&r.endNode==2&&r.automatic);
        assert(!pending);requests.push_back(r);pending=++nextId;
        MotionStatus s;s.id=pending;s.state=MotionState::QUEUED;jobs[pending]=s;return pending;
    }
    uint32_t lidar(uint8_t zone)override{MotionRequest r;r.kind=MotionKind::LIDAR_POSE;r.zone=zone;return submit(r);}
    MotionStatus status(uint32_t id)override{return jobs.at(id);}
    void visionStart(VisionStartMode mode,bool invertCornerXY)override{
        assert(!pending&&jobs.at(nextId).state==MotionState::DONE);
        if(mode==VisionStartMode::DISC)
            assert(requests.back().kind==MotionKind::DISC_RECORD_POSITION);
        if(mode==VisionStartMode::WORK_AREA||mode==VisionStartMode::WORK_AREA_LOADED)
            assert(requests.back().kind==MotionKind::NAVIGATE&&jobs.at(nextId).chassisDone&&jobs.at(nextId).armReady);
        if(mode==VisionStartMode::CORNER)
            assert(requests.back().kind==MotionKind::HOME_OBSERVE&&requests.back().armPrepared);
        assert(invertCornerXY==(mode==VisionStartMode::CORNER&&requests.back().zone==2));
        vision.push_back(mode);alignment=AutoAlignmentState::WAITING;visionPolls=0;
    }
    AutoAlignmentState visionStatus()override{return alignment;}
    void visionStop(VisionStartMode)override{}
    bool transientBusy=false;
    int busyAttempts=0;
    uint32_t submit(const MotionRequest &r,MotionSubmitError &error)override {
        if(transientBusy) {++busyAttempts;error=MotionSubmitError::BUSY;return 0;}
        error=MotionSubmitError::NONE;
        return submit(r);
    }
    bool motionBusy()override{return transientBusy;}
    void materialStart()override{assert(!pending);assert(requests.back().observation==ArmObservation::LOOK3);material=true;++materialStarts;}
    void materialStop()override{material=false;}
    void taskReady()override{taskPublished=true;}
    void runStart()override{++starts;runPermission=true;}
    void runDone()override{assert(!finished);finished=true;}
    void fail(RobotState reason)override{current=reason;}
    void display(const char *)override{}
    void complete(bool success=true){assert(pending);auto &s=jobs.at(pending);s.state=success?MotionState::DONE:MotionState::FAILED;s.chassisDone=s.armReady=success;pending=0;}
};
void polling() {
    Runtime r;CompetitionFlow flow(r,CompetitionMap{});
    flow.tick();for(int i=0;i<30;++i)flow.tick();assert(r.requests.size()==1&&r.current==STATE_WAIT_START);
    r.complete();flow.tick();for(int i=0;i<30;++i)flow.tick();assert(r.requests.size()==1);
    r.startZone=1;flow.tick();flow.tick();flow.tick();assert(r.requests.size()==2);for(int i=0;i<30;++i)flow.tick();assert(r.requests.size()==2);
    r.complete();flow.tick();
    for(int i=0;i<30;++i)flow.tick();assert(r.starts==0&&r.current==STATE_WAIT_START);
    r.runPermission=true;flow.tick();
    flow.tick();assert(r.current==STATE_READ_TASK&&r.starts==1);
    r.jobs.at(r.pending).state=MotionState::RUNNING;for(int i=0;i<30;++i)flow.tick();assert(r.current==STATE_READ_TASK&&!r.taskPublished);
    r.complete();flow.tick();assert(r.current==STATE_GRAB_ROUND1&&r.taskPublished);
}
void fullFlow(uint8_t zone,MotionKind failed=MotionKind::VELOCITY,bool failVision=false,bool cancel=false) {
    Runtime r;r.startZone=zone;CompetitionFlow flow(r,CompetitionMap{});
    bool failureInjected=false;int discGrabs=0,recognitions=0,delay=0;
    for(int tick=0;tick<1500&&!r.finished;++tick) {
        r.time+=10;
        if(r.pending&&++delay>=3) {
            const bool reject=r.requests.back().kind==failed
                &&(!failureInjected||failed==MotionKind::DISC_RECORD_POSITION);
            if(reject)failureInjected=true;
            if(cancel&&!failureInjected&&r.requests.back().kind==MotionKind::PLACE_ROUND){r.current=STATE_TIMEOUT_FAILED;r.abort=true;failureInjected=true;}
            r.complete(!reject);delay=0;
            if(!reject&&r.requests.back().kind==MotionKind::LIDAR_POSE)r.runPermission=true;
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
    }
    assert(failed==MotionKind::VELOCITY&&!failVision&&!cancel);
    assert(r.finished&&r.current==STATE_DONE&&discGrabs==6&&r.starts==1&&r.materialStarts==6);
    assert(!r.pending&&r.requests.back().kind==MotionKind::PLACE_ROUND&&r.requests.back().layer==2);
    const size_t completedRequests=r.requests.size();
    for(int i=0;i<10;++i)flow.tick();
    assert(r.requests.size()==completedRequests);
    assert(r.vision==std::vector<VisionStartMode>({VisionStartMode::DISC,VisionStartMode::WORK_AREA,VisionStartMode::WORK_AREA,
        VisionStartMode::DISC,VisionStartMode::WORK_AREA,VisionStartMode::WORK_AREA_LOADED}));
    int paths=0,places=0,retrieves=0,discRecords=0,discDepartures=0;
    MotionKind previous=MotionKind::ARM_POSE;
    for(const auto &request:r.requests) {
        if(request.kind==MotionKind::NAVIGATE){
            ++paths;
            assert(request.endNode!=0&&request.endNode!=4);
            assert(request.observation==(request.endNode==14?ArmObservation::LOOK2:ArmObservation::LOOK));
            assert(request.cargo==3); // 到站即选首件料盘，早于视觉对齐和首件作业。
        }
        if(request.kind==MotionKind::NAVIGATE&&request.startNode==22)assert(request.endNode==10&&previous==MotionKind::RETRIEVE_ROUND);
        if(request.kind==MotionKind::NAVIGATE) {
            assert(request.discDeparture==(request.startNode==14));
            if(request.discDeparture)++discDepartures;
        }
        if(request.kind==MotionKind::DISC_RECORD_POSITION) {
            assert(previous==MotionKind::NAVIGATE);
            ++discRecords;
        }
        if(request.kind==MotionKind::PLACE_ROUND){
            ++places;
            const int expected[4][3]={{1,2,3},{1,2,3},{2,3,1},{2,1,3}};
            for(int i=0;i<3;++i)assert(request.positions[i]==expected[places-1][i]);
            assert(request.colors[0]==(places<=2?1:5)&&request.colors[1]==(places<=2?5:1)&&request.colors[2]==6);
        }
        if(request.kind==MotionKind::PLACE_ROUND) {
            assert(request.layer==(places==4?2:1));
            assert(request.ringCorrection==(places!=4));
        }
        if(request.kind==MotionKind::RETRIEVE_ROUND){
            ++retrieves;
            const int expected[2][3]={{1,2,3},{2,3,1}};
            for(int i=0;i<3;++i)assert(request.positions[i]==expected[retrieves-1][i]);
        }
        assert(request.kind!=MotionKind::HOME_RETURN&&request.kind!=MotionKind::HOME_OBSERVE);
        previous=request.kind;
    }
    assert(paths==6&&places==4&&retrieves==2&&discRecords==2&&discDepartures==2);
    assert(MergeDiscYIntoFirstStraight(5)&&MergeDiscYIntoFirstStraight(15)&&!MergeDiscYIntoFirstStraight(13));
}
void calibrationWait() {
    Runtime r;r.calibrationReady=false;
    CompetitionFlow flow(r,CompetitionMap{});
    flow.tick();assert(!r.calibrationStarted);r.complete();flow.tick();
    assert(r.calibrationStarted&&r.requests.size()==1);
    for(int i=0;i<600;++i){r.time+=10;flow.tick();assert(r.requests.size()==1&&r.starts==0);}
    r.calibrationReady=true;
    flow.tick();flow.tick();assert(r.requests.size()==1);
    r.startZone=2;
    for(int i=0;i<3;++i)flow.tick();
    assert(r.requests.size()==2&&r.requests.back().kind==MotionKind::LIDAR_POSE&&r.requests.back().zone==2);
}
void failedStartupDoesNotStart() {
    for(uint8_t zone:{1,2}) {
        Runtime r;r.startZone=zone;CompetitionFlow flow(r,CompetitionMap{});
        flow.tick();r.complete();flow.tick();flow.tick();flow.tick();flow.tick();
        assert(r.requests.back().kind==MotionKind::LIDAR_POSE&&r.starts==0);
        r.complete(false);flow.tick();
        assert(r.current==STATE_TRANSFER_FAILED&&r.starts==0&&!r.runPermission);
    }
}
void recoverableStages() {
    Runtime r;
    r.current=STATE_GRAB_ROUND1;
    CompetitionFlow flow(r,CompetitionMap{});
    r.transientBusy=true;
    for(int i=0;i<6;++i)flow.tick();
    assert(r.requests.empty()&&r.busyAttempts==1);
    r.transientBusy=false;
    flow.tick();
    assert(r.requests.size()==1&&r.requests.back().kind==MotionKind::NAVIGATE);
    r.complete();flow.tick();flow.tick();
    r.complete(false);flow.tick();flow.tick();
    assert(r.requests.size()==3&&r.requests.back().kind==MotionKind::DISC_RECORD_POSITION);
    r.complete();flow.tick();
    assert(r.vision.size()==1);
    r.alignment=AutoAlignmentState::IDLE;
    flow.tick();
    assert(r.vision.size()==2&&r.alignment==AutoAlignmentState::WAITING);
    r.alignment=AutoAlignmentState::IDLE;flow.tick();r.time+=10;flow.tick();
    assert(r.requests.back().observation==ArmObservation::LOOK3);
    r.complete(false);flow.tick();flow.tick();
    assert(r.requests.size()==5&&r.requests.back().observation==ArmObservation::LOOK3&&!r.material);
    r.complete();flow.tick();
    assert(r.material&&r.materialStarts==1);
    r.time+=14999;flow.tick();assert(r.materialStarts==1);
    ++r.time;flow.tick();assert(r.material&&r.materialStarts==2);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(!strcmp(argv[1],"sequence"))sequence();
    else if(!strcmp(argv[1],"join"))join();
    else if(!strcmp(argv[1],"launch"))launch();
    else if(!strcmp(argv[1],"polling"))polling();
    else if(!strcmp(argv[1],"recovery"))recoverableStages();
    else if(!strcmp(argv[1],"startup")){calibrationWait();failedStartupDoesNotStart();}
    else if(!strcmp(argv[1],"rounds")){fullFlow(1);fullFlow(2);}
    else if(!strcmp(argv[1],"failures")) {
        for(auto kind:{MotionKind::ARM_POSE,MotionKind::SCAN_TASK,MotionKind::NAVIGATE,MotionKind::DISC_RECORD_POSITION,
            MotionKind::PLACE_ROUND,MotionKind::RETRIEVE_ROUND})fullFlow(1,kind);
    } else if(!strcmp(argv[1],"vision"))fullFlow(1,MotionKind::VELOCITY,true);
    else if(!strcmp(argv[1],"cancel"))fullFlow(1,MotionKind::VELOCITY,false,true);
    else assert(false);
}
