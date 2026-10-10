/**********************************************************
*** 2027 工创大赛·智能搬运 主控程序框架
*** 角色：ESP32 主控（运动控制 + 任务编排 + 决策）
*** 外部协作：机载电脑负责视觉识别 + 下发指令
***          （颜色/位置识别、转盘物料定位）
***
***       标有 [TODO] 的部分，需按实际机械/定位方案实现。
**********************************************************/

#include "robot_runtime.h"

void Task_MainStateMachine(void *pvParameters) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // 以下变量在创建主任务时初始化一次，在 while 循环之间保留，用于记录各阶段进度。

    // 首轮前往圆盘区的路径是否执行成功：false=尚未完成，true=已完成，避免重复行驶。
    bool discRouteCompleted = false;
    // 已处理观察姿态恢复的抓取进度：0=初始阶段，1/2=第1/2件完成后已恢复观察姿态。
    // 与 roundProgress 比较，仅在新一件完成后触发一次恢复；它不代表实际已抓取数量。
    int discPreparedProgress = 0;
    // 首轮是否已首次请求圆盘物料识别：true 后跳过圆盘对齐到物料识别的初始化分支。
    // 此标志不代表相机当前正在识别，也不代表三件物料已抓完。
    bool discMaterialRequested = false;
    // 首轮前往粗加工区的路径是否执行成功：true 后不再重复请求和执行该路径。
    bool coarse1RouteCompleted = false;
    // 是否正在等待首轮粗加工区视觉对齐结果：启动后置 true，结束并处理结果时清为 false。
    bool coarse1VisionRequested = false;
    // TEMP1 在粗加工放料后短暂等待并直接取回，成功后才允许导航。
    bool temp1CargoRetrieved = false;
    bool temp1RouteCompleted = false;
    bool temp1VisionRequested = false;
    bool disc2RouteCompleted = false;
    int disc2PreparedProgress = 0;
    bool disc2MaterialRequested = false;
    bool coarse2RouteCompleted = false;
    bool coarse2VisionRequested = false;
    bool temp2CargoRetrieved = false;
    bool temp2RouteCompleted = false;
    bool temp2VisionRequested = false;
    bool homeRouteCompleted = false;
    bool homeVisionRequested = false;
    StartZone homeStartZone = START_ZONE_UNKNOWN;

    while (1) {
        switch (currentState) {


        case STATE_WAIT_START: // 等待开始区域
            Serial.println("TASK start");
            InitArm_start();// 初始化机械臂
            updateDisplay("DISPLAY", "DEBUG", "WAIT start_zone");
            while (currentStartZone == START_ZONE_UNKNOWN) vTaskDelay(100 / portTICK_PERIOD_MS);
            homeStartZone = currentStartZone;
            switch (homeStartZone) {
            case START_ZONE_1:
                updateDisplay("DISPLAY", "DEBUG", "start_zone: 1");
                // 右侧启停区车头朝左，即世界坐标 -X 方向。
                currentPose = {2250, 150, START_ZONE_HEADING};
                break;
            case START_ZONE_2:
                updateDisplay("DISPLAY", "DEBUG", "start_zone: 2");
                // 左侧启停区车头也朝左；向右扫码使用后退，航向仍为180°。
                currentPose = {150, 150, START_ZONE_HEADING};
                break;

            default:
                updateDisplay("DISPLAY", "DEBUG", "ERR:start_zone: unknown");
                break;
            }
            // Release 开局自动执行一次。动作未标定或执行失败时返回 ERR，
            // 仍停留在开局等待阶段，便于通过串口修正后再次手动调用。
            runLidarPoseAction();
            updateDisplay("DISPLAY", "DEBUG", "WAIT START");
            while (!enableRun) vTaskDelay(100 / portTICK_PERIOD_MS);
            // 启动总超时兜底(如 300s 内未回启停区)
            if (xHomeTimer != NULL) xTimerStart(xHomeTimer, 0);
            currentState = STATE_READ_TASK;
            break;

        case STATE_READ_TASK: { // 读取任务码
            updateDisplay("DISPLAY", "DEBUG", "READ TASK");
            InitArm_start();
            constexpr float SCAN_APPROACH_MM = 900.0f;
            constexpr float SCAN_RANGE_MM = 400.0f;
            constexpr float SCAN_SPEED_RPM = 60.0f;
            constexpr uint8_t SCAN_MAX_RETRIES = 3;
            // 按实车方向：启停区1向左扫码（+Y），启停区2向右扫码（-Y）。
            // 本次扫码固定使用同一个区号，接近和往返扫描统一应用方向。
            const StartZone scanStartZone = currentStartZone;
            const float scanDirection = scanStartZone == START_ZONE_1 ? 1.0f : -1.0f;
            const auto pollTaskCode = []() -> bool {
                if (taskReceived) return true;
                char scanCode[SCANNER_BUF_LEN];
                if (waitScannerCode(scanCode, sizeof(scanCode), 0)) {
                    const TaskCode scannedTask = parseTaskCode(scanCode);
                    portENTER_CRITICAL(&taskCodeMux);
                    // 手动码已提交时，迟到的扫码结果不能覆盖它。
                    if (!taskReceived && scannedTask.valid) {
                        currentTask = scannedTask;
                        taskReceived = true;
                    }
                    portEXIT_CRITICAL(&taskCodeMux);
                    Serial.printf("[SCANNER] task code %s\n", scannedTask.valid ? "OK" : "ERR");
                }
                return taskReceived;
            };
            if (!taskReceived) {
                if (scanStartZone != START_ZONE_1 && scanStartZone != START_ZONE_2) {
                    Serial.println("[SCANNER] ERR: invalid start zone");
                } else if (!MovePosition(0, scanDirection * SCAN_APPROACH_MM, 0, SCAN_SPEED_RPM)) {
                    Serial.println("[SCANNER] ERR: approach failed");
                } else {
                    // 首次沿接近方向扫描 400 mm；失败后在同一区域反向重试三遍。
                    // 沿接近方向距起点的端点依次为 1300、900、1300、900 mm。
                    for (uint8_t attempt = 0; attempt <= SCAN_MAX_RETRIES; ++attempt) {
                        if (pollTaskCode()) break;
                        const float distance = scanDirection * ((attempt % 2 == 0) ? SCAN_RANGE_MM : -SCAN_RANGE_MM);
                        Serial.printf("[SCANNER] scan %u/%u, distance=%.0f mm\n",
                                      static_cast<unsigned>(attempt),
                                      static_cast<unsigned>(SCAN_MAX_RETRIES), distance);
                        const bool completed = MovePosition(0, distance, 0, SCAN_SPEED_RPM/2, pollTaskCode);
                        if (pollTaskCode()) break;
                        if (!completed) {
                            Serial.println("[SCANNER] ERR: position move failed");
                            break;
                        }
                        if (attempt == SCAN_MAX_RETRIES) {
                            Serial.println("[SCANNER] ERR: retries exhausted (3/3)");
                        }
                    }
                }
            }
            if (!taskReceived) {
                updateDisplay("DISPLAY", "DEBUG", "TASK ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_SCAN_FAILED;
                break;
            }
            // 按已解析的 4 组三位数重新组装，避免二维码尾部杂字符进入显示帧。
            char taskCodeText[16];
            snprintf(taskCodeText, sizeof(taskCodeText), "%d%d%d+%d%d%d+%d%d%d+%d%d%d",
                     currentTask.round1_colors[0], currentTask.round1_colors[1], currentTask.round1_colors[2],
                     currentTask.round1_pos[0], currentTask.round1_pos[1], currentTask.round1_pos[2],
                     currentTask.round2_colors[0], currentTask.round2_colors[1], currentTask.round2_colors[2],
                     currentTask.round2_pos[0], currentTask.round2_pos[1], currentTask.round2_pos[2]);
            updateDisplay("DISPLAY", "TASK_CODE", taskCodeText);
            updateDisplay("DISPLAY", "DEBUG", "TASK OK");
            xQueueReset(xVisualTaskQueue); // 清残留信号
            currentState = STATE_GRAB_ROUND1;
            break;
        }

        case STATE_SCAN_FAILED: // 扫码失败
        case STATE_ROUTE_FAILED: // 路径失败，同样保持故障状态
        case STATE_ALIGN_FAILED:// 定位失败
        case STATE_TIMEOUT_FAILED:
        case STATE_TRANSFER_FAILED:
            // 保持故障状态，避免下一轮状态机再次启动扫码或执行残留路径。
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;

        case STATE_GRAB_ROUND1: { // 抓取第一轮物料
            // 此状态会轮询抓取进度，路径成功后只执行一次，避免重复行驶。
            // 原料区为旋转电动转盘(6-10s/圈, 转向随机, 物料120°分布)
            // COLOR 匹配下一件任务颜色时抓取，动作成功并登记载物台后推进 roundProgress。
            // 规则: 每次抓1个; 物料必须放到机器人上才能抓下一个
            //       不允许手爪夹持运送
            updateDisplay("DISPLAY", "DEBUG", discMaterialRequested ? "WAIT DISC MATERIAL" : "GRAB R1");
            //调取接口获取路径, 并移动到目标位置
            if (!discRouteCompleted) {
                if (!requestAndMoveNodePath(SCAN_AREA_NODE, DISC_AREA_NODE)) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);// 路径失败后, 停止定时器
                    currentState = STATE_ROUTE_FAILED;
                    break; // 失败时跳过下面的用户代码
                }
                discRouteCompleted = true;
                roundProgress = 0;
                firstDiscGrabReady = false;
                // 先清除旧反馈并开启连续闭环，再通知机载电脑发送 DISC 数据。
                // 路径执行完后请求圆盘定位视觉；上位机应切到圆盘定位流程。
                if (!InitArm_look3()) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                requestVisionStart(VisionStartMode::DISC);

            }
            //走到这里时路径已执行成功，可继续轮询后续任务进度。
            // 等待圆盘定位完成，再切换到物料识别；实际对齐由视觉对齐任务执行。
            // false 表示尚未请求物料识别；置为 true 后，后续循环跳过整个分支。
            if (!discMaterialRequested) {
                // 保存当前对齐结果；后面关闭对齐会重置内部状态，但不影响此局部副本。
                const auto alignmentState = getAutoAlignmentState();
                // 对齐仍在进行：只退出本轮 switch，主循环延时后再次检查，不执行下方动作。
                if (alignmentState == AutoAlignmentState::WAITING) break;
                // 对齐已结束：关闭底盘视觉闭环，清空反馈、重置控制器并停车。
                setAlignmentEnabled(false);
                // 通知机载电脑停止圆盘定位功能；这里只发送请求，不等待相机停止确认。
                requestVisionStop(VisionStartMode::DISC);
                // 只有 DONE 允许继续；FAILED 或意外的 IDLE 都按定位失败处理。
                if (alignmentState != AutoAlignmentState::DONE) {
                    // 在显示屏上提示圆盘定位失败。
                    updateDisplay("DISPLAY", "DEBUG", "DISC ALIGN ERR");
                    // 若总超时定时器已创建，则请求停止，避免其随后触发回家流程。
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    // 下一轮主循环进入定位故障状态，保持停车。
                    currentState = STATE_ALIGN_FAILED;
                    // 退出本轮 switch，不再执行下面的机械臂动作和物料识别启动。
                    break;
                }
                // 定位成功：清空视觉任务队列，丢弃定位阶段残留的颜色结果。
                xQueueReset(xVisualTaskQueue);
                // 圆盘定位成功后、物料识别前的位置调整，仅在首次切换时执行。
                vTaskDelay(pdMS_TO_TICKS(100));
                //GotoPose(-45, 0, 0, true);
                vTaskDelay(pdMS_TO_TICKS(100));
                // 观察姿态命令成功后才开放抓取，失败时不启动物料识别。
                if (!InitArm_look2()) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                firstDiscGrabReady = true;
                // 启用物料识别接收状态，并向机载电脑发送圆盘物料识别启动请求。
                requestVisionStart(VisionStartMode::DISC_MATERIAL);
                // 标记请求已发送，避免下一轮重复调整机械臂和启动识别；不代表识别已完成。
                discMaterialRequested = true;
                // 显示正在等待圆盘物料识别结果。
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL");
                // 后续由串口 COLOR 信息驱动抓取；第三件成功后关闭物料识别，
                // 并将 roundProgress 推进到 3，由下方判断切换主状态。
            }
            // COLOR 的匹配和抓取由串口任务执行，此处只衔接每件完成后的观察准备。
            // 抓取期间视觉保持开启，串口任务屏蔽颜色处理；成功放上载物台才增加进度。
            // 使用进度变化触发，避免等待下一种颜色时反复调用机械臂初始化。
            const int completedGrabs = roundProgress;
            if (completedGrabs > discPreparedProgress && completedGrabs < 3) {
                Serial.printf("[DISC] Restoring observation after grab %d\n", completedGrabs);
                if (!InitArm_look2()) {
                    firstDiscGrabReady = false;
                    requestVisionStop(VisionStartMode::DISC_MATERIAL);
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                // 恢复观察期间若总定时器已切换状态，不再恢复颜色处理。
                if (currentState != STATE_GRAB_ROUND1) break;
                xQueueReset(xVisualTaskQueue);
                discPreparedProgress = completedGrabs;
                // 不启停视觉；串口任务处理固定的旧缓存及旧半帧后开放下一件。
                discMessageResumeRequested = true;
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL");
            }


            if (roundProgress >= 3 ) {
                firstDiscGrabReady = false;
                roundProgress = 0;
                currentState = STATE_PLACE_COARSE1;
            }


            break;
        }

        // 第一批物料放置到粗加工区：导航至粗加工区 → 视觉对齐 → 放置物料 → 转入暂存区放置
        case STATE_PLACE_COARSE1:
        {
            // 按 round1_pos 顺序放置到粗加工区对应圆环
            // 圆环评分: 1环15分 2环10分 3环7分 ... 越中心分越高
            // 从粗加工区取回3个, 按 round1_pos 放到暂存区
            // 在粗加工区取回：
            updateDisplay("DISPLAY", "DEBUG", coarse1RouteCompleted ? "ALIGN COARSE1" : "GO COARSE1");

            if (!coarse1RouteCompleted) {
                // 离开圆盘前关闭物料识别，避免导航途中继续产生颜色结果。
                requestVisionStop(VisionStartMode::DISC_MATERIAL);
                xQueueReset(xVisualTaskQueue);
                if (!requestAndMoveNodePath(DISC_AREA_NODE, COARSE_AREA_NODE)) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ROUTE_FAILED;
                    break;
                }
                coarse1RouteCompleted = true;
                roundProgress = 0;
                // 到达粗加工区后开启一次自动 PID 对齐；定位视觉只负责空工位。
                // 先显式执行观察姿态，成功后再启用工位视觉定位。
                setAlignmentEnabled(false);
                if (!InitArm_look()) {
                    if (currentState != STATE_PLACE_COARSE1) break;
                    updateDisplay("DISPLAY", "DEBUG", "WORK AREA ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                vTaskDelay(100);
                if (currentState != STATE_PLACE_COARSE1 || taskMotionAborted) break;
                requestVisionStart(VisionStartMode::WORK_AREA);
                coarse1VisionRequested = true;
                break;
            }

            if (coarse1VisionRequested) {
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::WORK_AREA);
                coarse1VisionRequested = false;
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "COARSE ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
            }

            updateDisplay("DISPLAY", "DEBUG", "PLACE C1");
            // 批量接口会按任务码将载物台 1~3 全部放到第一层；返回 true
            // 只说明预设动作均已执行，不代表传感器确认物料实际放置成功。
            if (!PlaceTaskCargoToWorkArea(currentTask.round1_pos, 1)) {
                updateDisplay("DISPLAY", "DEBUG", "PLACE C1 ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            roundProgress = 0;
            if (currentState != STATE_PLACE_COARSE1 || taskMotionAborted) break;
            /* 暂时跳过粗加工放料后的带物料定位及观察姿态。
            // 先显式执行观察姿态，成功后再启用工位视觉定位。
            setAlignmentEnabled(false);
            if (!InitArm_look()) {
                if (currentState != STATE_PLACE_COARSE1) break;
                updateDisplay("DISPLAY", "DEBUG", "WORK AREA ARM ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            if (currentState != STATE_PLACE_COARSE1 || taskMotionAborted) break;
            requestVisionStart(VisionStartMode::WORK_AREA_LOADED);
            */
            currentState = STATE_PLACE_TEMP1;
            break;
        }

        case STATE_PLACE_TEMP1: {
            if (!temp1CargoRetrieved) {
                /* 暂时跳过带物料对齐等待；放料时已完成工位定位。
                // COARSE1 已开启 WORK_AREA_LOADED；等待达标，不能把请求当作完成。
                updateDisplay("DISPLAY", "DEBUG", "ALIGN LOADED C1");
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                // 达标后连续 PID 仍在运行，取料前必须先停车。
                setAlignmentEnabled(false);
                if (alignmentState != AutoAlignmentState::DONE) {
                    requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
                    updateDisplay("DISPLAY", "DEBUG", "LOADED ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
                */
                setAlignmentEnabled(false);
                if (taskMotionAborted) break;
                updateDisplay("DISPLAY", "DEBUG", "WAIT RETRIEVE C1");
                vTaskDelay(pdMS_TO_TICKS(100));
                if (currentState != STATE_PLACE_TEMP1 || taskMotionAborted) break;
                updateDisplay("DISPLAY", "DEBUG", "RETRIEVE C1");
                // round1_pos[i] 的物料取回载物台 i+1，保持颜色和位置的对应关系。
                const bool retrieved = RetrieveRoundToCargo(
                    currentTask.round1_colors, currentTask.round1_pos);
                // requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
                if (currentState != STATE_PLACE_TEMP1) break;
                if (!retrieved) {
                    updateDisplay("DISPLAY", "DEBUG", "RETRIEVE C1 ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                temp1CargoRetrieved = true;
                roundProgress = 0;
                break;
            }

            if (!temp1RouteCompleted) {
                updateDisplay("DISPLAY", "DEBUG", "GO TEMP1");
                const bool moved = requestAndMoveNodePath(COARSE_AREA_NODE, TEMP_AREA_NODE);
                if (currentState != STATE_PLACE_TEMP1) break;
                if (!moved) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ROUTE_FAILED;
                    break;
                }
                temp1RouteCompleted = true;
                // 路径入口已负责暂存区 180° 朝向；到达后再开启空工位对齐。
                // 先显式执行观察姿态，成功后再启用工位视觉定位。
                setAlignmentEnabled(false);
                if (!InitArm_look()) {
                    if (currentState != STATE_PLACE_TEMP1) break;
                    updateDisplay("DISPLAY", "DEBUG", "WORK AREA ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_PLACE_TEMP1 || taskMotionAborted) break;
                requestVisionStart(VisionStartMode::WORK_AREA);
                temp1VisionRequested = true;
                break;
            }

            if (temp1VisionRequested) {
                updateDisplay("DISPLAY", "DEBUG", "ALIGN TEMP1");
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::WORK_AREA);
                temp1VisionRequested = false;
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "TEMP ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
            }

            updateDisplay("DISPLAY", "DEBUG", "PLACE T1");
            const bool placed = PlaceTaskCargoToWorkArea(currentTask.round1_pos, 1);
            if (currentState != STATE_PLACE_TEMP1) break;
            if (!placed) {
                updateDisplay("DISPLAY", "DEBUG", "PLACE T1 ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            roundProgress = 0;
            currentState = STATE_GRAB_ROUND2;
            break;
        }

        case STATE_GRAB_ROUND2: { // 抓取第二轮物料
            // 此状态会轮询抓取进度，路径成功后只执行一次，避免重复行驶。
            // 原料区为旋转电动转盘(6-10s/圈, 转向随机, 物料120°分布)
            // COLOR 匹配下一件任务颜色时抓取，动作成功并登记载物台后推进 roundProgress。
            // 规则: 每次抓1个; 物料必须放到机器人上才能抓下一个
            //       不允许手爪夹持运送
            updateDisplay("DISPLAY", "DEBUG", disc2MaterialRequested ? "WAIT DISC MATERIAL" : "GRAB R2");
            //调取接口获取路径, 并移动到目标位置
            if (!disc2RouteCompleted) {
                if (!requestAndMoveNodePath(TEMP_AREA_NODE, DISC_AREA_NODE)) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);// 路径失败后, 停止定时器
                    currentState = STATE_ROUTE_FAILED;
                    break; // 失败时跳过下面的用户代码
                }
                disc2RouteCompleted = true;
                roundProgress = 0;
                firstDiscGrabReady = false;
                // 先清除旧反馈并开启连续闭环，再通知机载电脑发送 DISC 数据。
                // 路径执行完后请求圆盘定位视觉；上位机应切到圆盘定位流程。
                if (!InitArm_look2()) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                requestVisionStart(VisionStartMode::DISC);

            }
            //走到这里时路径已执行成功，可继续轮询后续任务进度。
            // 等待圆盘定位完成，再切换到物料识别；实际对齐由视觉对齐任务执行。
            // false 表示尚未请求物料识别；置为 true 后，后续循环跳过整个分支。
            if (!disc2MaterialRequested) {
                // 保存当前对齐结果；后面关闭对齐会重置内部状态，但不影响此局部副本。
                const auto alignmentState = getAutoAlignmentState();
                // 对齐仍在进行：只退出本轮 switch，主循环延时后再次检查，不执行下方动作。
                if (alignmentState == AutoAlignmentState::WAITING) break;
                // 对齐已结束：关闭底盘视觉闭环，清空反馈、重置控制器并停车。
                setAlignmentEnabled(false);
                // 通知机载电脑停止圆盘定位功能；这里只发送请求，不等待相机停止确认。
                requestVisionStop(VisionStartMode::DISC);
                // 只有 DONE 允许继续；FAILED 或意外的 IDLE 都按定位失败处理。
                if (alignmentState != AutoAlignmentState::DONE) {
                    // 在显示屏上提示圆盘定位失败。
                    updateDisplay("DISPLAY", "DEBUG", "DISC ALIGN ERR");
                    // 若总超时定时器已创建，则请求停止，避免其随后触发回家流程。
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    // 下一轮主循环进入定位故障状态，保持停车。
                    currentState = STATE_ALIGN_FAILED;
                    // 退出本轮 switch，不再执行下面的机械臂动作和物料识别启动。
                    break;
                }
                // 定位成功：清空视觉任务队列，丢弃定位阶段残留的颜色结果。
                xQueueReset(xVisualTaskQueue);
                // 圆盘定位成功后、物料识别前的位置调整，仅在首次切换时执行。
                vTaskDelay(pdMS_TO_TICKS(100));
                //GotoPose(-45, 0, 0, true);
                vTaskDelay(pdMS_TO_TICKS(100));
                // 观察姿态命令成功后才开放抓取，失败时不启动物料识别。
                if (!InitArm_look2()) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                firstDiscGrabReady = true;
                // 启用物料识别接收状态，并向机载电脑发送圆盘物料识别启动请求。
                requestVisionStart(VisionStartMode::DISC_MATERIAL);
                // 标记请求已发送，避免下一轮重复调整机械臂和启动识别；不代表识别已完成。
                disc2MaterialRequested = true;
                // 显示正在等待圆盘物料识别结果。
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL");
                // 后续由串口 COLOR 信息驱动抓取；第三件成功后关闭物料识别，
                // 并将 roundProgress 推进到 3，由下方判断切换主状态。
            }
            // COLOR 的匹配和抓取由串口任务执行，此处只衔接每件完成后的观察准备。
            // 抓取期间视觉保持开启，串口任务屏蔽颜色处理；成功放上载物台才增加进度。
            // 使用进度变化触发，避免等待下一种颜色时反复调用机械臂初始化。
            const int completedGrabs = roundProgress;
            if (completedGrabs > disc2PreparedProgress && completedGrabs < 3) {
                Serial.printf("[DISC] Restoring observation after grab %d\n", completedGrabs);
                if (!InitArm_look2()) {
                    firstDiscGrabReady = false;
                    requestVisionStop(VisionStartMode::DISC_MATERIAL);
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                // 恢复观察期间若总定时器已切换状态，不再恢复颜色处理。
                if (currentState != STATE_GRAB_ROUND2) break;
                xQueueReset(xVisualTaskQueue);
                disc2PreparedProgress = completedGrabs;
                // 不启停视觉；串口任务处理固定的旧缓存及旧半帧后开放下一件。
                discMessageResumeRequested = true;
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL");
            }


            if (roundProgress >= 3 ) {
                firstDiscGrabReady = false;
                roundProgress = 0;
                currentState = STATE_PLACE_COARSE2;
            }


            break;
        }

        // 第二批物料放置到粗加工区：导航至粗加工区 → 视觉对齐 → 放置物料 → 转入暂存区放置
        case STATE_PLACE_COARSE2:
        {
            // 按 round2_pos 顺序放置到粗加工区对应圆环
            // 圆环评分: 1环15分 2环10分 3环7分 ... 越中心分越高
            updateDisplay("DISPLAY", "DEBUG",
                          coarse2RouteCompleted ? "ALIGN COARSE2" : "GO COARSE2");

            if (!coarse2RouteCompleted) {
                // 离开圆盘前关闭物料识别，避免导航途中继续产生颜色结果。
                requestVisionStop(VisionStartMode::DISC_MATERIAL);
                xQueueReset(xVisualTaskQueue);
                if (!requestAndMoveNodePath(DISC_AREA_NODE, COARSE_AREA_NODE)) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ROUTE_FAILED;
                    break;
                }
                coarse2RouteCompleted = true;
                roundProgress = 0;
                // 到达粗加工区后开启一次自动 PID 对齐；定位视觉只负责空工位。
                // 先显式执行观察姿态，成功后再启用工位视觉定位。
                setAlignmentEnabled(false);
                if (!InitArm_look()) {
                    if (currentState != STATE_PLACE_COARSE2) break;
                    updateDisplay("DISPLAY", "DEBUG", "WORK AREA ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_PLACE_COARSE2 || taskMotionAborted) break;
                requestVisionStart(VisionStartMode::WORK_AREA);
                coarse2VisionRequested = true;
                break;
            }

            if (coarse2VisionRequested) {
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::WORK_AREA);
                coarse2VisionRequested = false;
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "COARSE ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
            }

            updateDisplay("DISPLAY", "DEBUG", "PLACE C2");
            // 批量接口会按任务码将载物台 1~3 全部放到第一层；返回 true
            // 只说明预设动作均已执行，不代表传感器确认物料实际放置成功。
            if (!PlaceTaskCargoToWorkArea(currentTask.round2_pos, 1)) {
                updateDisplay("DISPLAY", "DEBUG", "PLACE C2 ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            roundProgress = 0;
            if (currentState != STATE_PLACE_COARSE2 || taskMotionAborted) break;
            /* 暂时跳过粗加工放料后的带物料定位及观察姿态。
            // 先显式执行观察姿态，成功后再启用工位视觉定位。
            setAlignmentEnabled(false);
            if (!InitArm_look()) {
                if (currentState != STATE_PLACE_COARSE2) break;
                updateDisplay("DISPLAY", "DEBUG", "WORK AREA ARM ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            if (currentState != STATE_PLACE_COARSE2 || taskMotionAborted) break;
            requestVisionStart(VisionStartMode::WORK_AREA_LOADED);
            */
            currentState = STATE_STACK_TEMP2;
            break;
        }

        case STATE_STACK_TEMP2: {
            if (!temp2CargoRetrieved) {
                /* 暂时跳过带物料对齐等待；放料时已完成工位定位。
                // COARSE2 已开启 WORK_AREA_LOADED；等待达标，不能把请求当作完成。
                updateDisplay("DISPLAY", "DEBUG", "ALIGN LOADED C2");
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                // 达标后连续 PID 仍在运行，取料前必须先停车。
                setAlignmentEnabled(false);
                if (alignmentState != AutoAlignmentState::DONE) {
                    requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
                    updateDisplay("DISPLAY", "DEBUG", "LOADED ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
                */
                setAlignmentEnabled(false);
                if (taskMotionAborted) break;
                updateDisplay("DISPLAY", "DEBUG", "WAIT RETRIEVE C2");
                vTaskDelay(pdMS_TO_TICKS(100));
                if (currentState != STATE_STACK_TEMP2 || taskMotionAborted) break;
                updateDisplay("DISPLAY", "DEBUG", "RETRIEVE C2");
                // round2_pos[i] 的物料取回载物台 i+1，保持颜色和位置的对应关系。
                const bool retrieved = RetrieveRoundToCargo(
                    currentTask.round2_colors, currentTask.round2_pos);
                // requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
                if (currentState != STATE_STACK_TEMP2) break;
                if (!retrieved) {
                    updateDisplay("DISPLAY", "DEBUG", "RETRIEVE C2 ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                temp2CargoRetrieved = true;
                roundProgress = 0;
                break;
            }

            if (!temp2RouteCompleted) {
                updateDisplay("DISPLAY", "DEBUG", "GO TEMP2");
                const bool moved = requestAndMoveNodePath(COARSE_AREA_NODE, TEMP_AREA_NODE);
                if (currentState != STATE_STACK_TEMP2) break;
                if (!moved) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ROUTE_FAILED;
                    break;
                }
                temp2RouteCompleted = true;
                // 暂存区已有第一层物料，码垛前使用带物料工位识别。
                // 先显式执行观察姿态，成功后再启用工位视觉定位。
                setAlignmentEnabled(false);
                if (!InitArm_look()) {
                    if (currentState != STATE_STACK_TEMP2) break;
                    updateDisplay("DISPLAY", "DEBUG", "WORK AREA ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_STACK_TEMP2 || taskMotionAborted) break;
                requestVisionStart(VisionStartMode::WORK_AREA_LOADED);
                temp2VisionRequested = true;
                break;
            }

            if (temp2VisionRequested) {
                updateDisplay("DISPLAY", "DEBUG", "ALIGN TEMP2");
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
                temp2VisionRequested = false;
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "TEMP ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
            }

            updateDisplay("DISPLAY", "DEBUG", "STACK T2");
            const bool placed = PlaceTaskCargoToWorkArea(currentTask.round2_pos, 2);
            if (currentState != STATE_STACK_TEMP2) break;
            if (!placed) {
                updateDisplay("DISPLAY", "DEBUG", "STACK T2 ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            roundProgress = 0;
            currentState = STATE_RETURN_HOME;
            break;
        }

        case STATE_RETURN_HOME: {
            if (!homeRouteCompleted) {
                updateDisplay("DISPLAY", "DEBUG", "GO HOME");
                if (homeStartZone != START_ZONE_1 && homeStartZone != START_ZONE_2) {
                    updateDisplay("DISPLAY", "DEBUG", "HOME ZONE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ROUTE_FAILED;
                    break;
                }
                // 使用开局保存的区域，不受运行中 START_ZONE 修改影响。
                const uint8_t homeNode = homeStartZone == START_ZONE_1
                    ? HOME_ZONE1_NODE : HOME_ZONE2_NODE;
                const float homeHeading = START_ZONE_HEADING;
                firstDiscGrabReady = false;
                requestVisionStop(VisionStartMode::DISC_MATERIAL);
                setAlignmentEnabled(false);
                if (!InitArm_start()) {
                    updateDisplay("DISPLAY", "DEBUG", "HOME ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_RETURN_HOME) break;
                const bool moved = requestAndMoveNodePath(TEMP_AREA_NODE, homeNode, homeHeading);
                if (currentState != STATE_RETURN_HOME) break;
                if (!moved) {
                    updateDisplay("DISPLAY", "DEBUG", "HOME ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ROUTE_FAILED;
                    break;
                }
                homeRouteCompleted = true;
                // 回到启停区后先将机械臂调整到角点识别位，再启动 CORNER 视觉。
                if (!InitArm_look4()) {
                    updateDisplay("DISPLAY", "DEBUG", "HOME LOOK ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_RETURN_HOME || taskMotionAborted) break;
                requestVisionStart(VisionStartMode::CORNER);
                homeVisionRequested = true;
                break;
            }
            if (homeVisionRequested) {
                updateDisplay("DISPLAY", "DEBUG", "ALIGN HOME");
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break;
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::CORNER);
                homeVisionRequested = false;
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "HOME ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
            }
            // 角点达标后复位；指令成功仍需实车确认机械臂和底盘到位。
            if (!InitArm_start()) {
                updateDisplay("DISPLAY", "DEBUG", "RESET ARM ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            if (currentStartZone == START_ZONE_1) {
                GotoPose(-70, 0, 0, true);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, 200, 0, true);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            } else if (currentStartZone == START_ZONE_2) {
                GotoPose(-70, 0, 0, true);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, 200, 0, true);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            if (currentState != STATE_RETURN_HOME) break;
            roundProgress = 0;
            if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
            currentState = STATE_DONE;
            break;
        }

        case STATE_DONE:
            updateDisplay("DISPLAY", "DEBUG", "DONE");
            if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
            Emm_V5_En_Control_all(false);
            vTaskDelay(100000000 / portTICK_PERIOD_MS); // 保持
            break;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ================= setup  =================
void setup() {
    pinMode(BOOT0_PIN, INPUT_PULLUP);
    const bool ledPwmReady = LedPwm_Init();
    Serial.begin(115200);
    if (!ledPwmReady) Serial.println("[LED] PWM initialization failed");
    Serial.printf("version: %s\n", VERSION);
    Servo_Init();    // 总线舵机初始化 (默认 Serial2: RX=16, TX=15, 115200bps)
    Emm_V5_Init();   // 电机初始化
    Scanner_Init();  // 扫码模块初始化 (软串口 RX=IO4, 9600bps)
    // [TODO] 若有传感器/定位硬件, 在此初始化
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(10);

    currentPose = {0, 0, 0};   // [TODO] 初始位姿按实际
    currentArm = {0, 0, 0, 0}; // 升降/伸缩目标坐标对应 Emm_V5_Init 的开机清零，舵机角度由下方实读更新。
    // 与 MoveArm 的实际 ID 对应：2 号=转台，1 号=夹爪。
    // 在串口任务启动前读取，避免多个任务同时访问舵机总线。
    // 给上电中的舵机留出启动时间；总线已初始化但舵机不一定立即应答。
    vTaskDelay(pdMS_TO_TICKS(100));
    readStartupServoAngle(2, currentArm.turret_angle);
    readStartupServoAngle(1, currentArm.pawl_angle);

    // 注意: 原 initLidar() 已移除, 雷达/定位方案待定
     init_ota_service("null", "1234567899", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
    //init_ota_service("longggg", "asdfghjkl", OTA_HOSTNAME);//调试使用，正式比赛时注释掉
    leds[0] = CRGB::Red; FastLED.show();

    // 机载电脑指令队列(深度10)
    xVisualTaskQueue = xQueueCreate(10, sizeof(VisualCmd_t));
    xLidarPoseMutex = xSemaphoreCreateMutex();
    // 视觉 PID 对齐只保留 20 Hz 连续反馈中的最新一帧。
    xAlignmentQueue = xQueueCreate(1, sizeof(VisualAlignmentFrame_t));
    xAlignmentMotionMutex = xSemaphoreCreateMutex();
    if (xAlignmentQueue != NULL && xAlignmentMotionMutex != NULL) {
        xTaskCreate(
            Task_VisualAlignment, "Task_VisualAlignment",
            8192, NULL, 7, NULL
        );
    } else {
        Serial.println("[Align] ERR: failed to create alignment queue or mutex");
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
    // 上电固定进入 Debug；串口SYS RELEASE或空闲时按BOOT0切换到正式模式。
    {
        Serial.println("Debug mode");
        leds[0] = CRGB::Yellow; FastLED.show();
        // Debug 模式默认直接启用视觉闭环对齐，仍可通过
        // {CMD,VISION,ALIGN_STOP}/{CMD,VISION,ALIGN_START} 可停止或重启。
        setAlignmentEnabled(true);
        xTaskCreate(Task_Serial_CMD, "Task_Serial_CMD", 16384, NULL, 5, NULL);
    }
}

void loop() {
    PollDebugBootButton();
    // 不依赖主状态机：总超时暂停主任务后仍持续闪灯。
    const RobotState state = currentState;
    LedPwm_UpdateFaultBlink(state == STATE_SCAN_FAILED || state == STATE_ROUTE_FAILED
        || state == STATE_ALIGN_FAILED || state == STATE_TIMEOUT_FAILED
        || state == STATE_TRANSFER_FAILED);
    vTaskDelay(pdMS_TO_TICKS(20));
}
