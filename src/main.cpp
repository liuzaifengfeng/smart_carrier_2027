/**********************************************************
*** 2027 工创大赛·智能搬运 主控程序框架
*** 角色：ESP32 主控（运动控制 + 任务编排 + 决策）
*** 外部协作：机载电脑负责视觉识别 + 下发指令
***          （颜色/位置识别、转盘物料定位）
***
***       标有 [TODO] 的部分，需按实际机械/定位方案实现。
**********************************************************/

#include "robot_runtime.h"
#include "h30_usb.h"
#include "continuous_trial.h"

void Task_MainStateMachine(void *pvParameters) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // 以下变量在创建主任务时初始化一次，在 while 循环之间保留，用于记录各阶段进度。

    bool discRouteCompleted = false; // 首轮前往圆盘区的路径是否执行成功：false=尚未完成，true=已完成，避免重复行驶。
    // 已处理观察姿态恢复的抓取进度：0=初始阶段，1/2=第1/2件完成后已恢复观察姿态。
    // 与 roundProgress 比较，仅在新一件完成后触发一次恢复；它不代表实际已抓取数量。
    int discPreparedProgress = 0;
    // 首轮是否已首次请求圆盘物料识别：true 后跳过圆盘对齐到物料识别的初始化分支。
    // 此标志不代表相机当前正在识别，也不代表三件物料已抓完。
    bool discMaterialRequested = false;
    bool coarse1RouteCompleted = false; // 首轮前往粗加工区的路径是否执行成功：true 后不再重复请求和执行该路径。
    bool coarse1VisionRequested = false; // 是否正在等待首轮粗加工区视觉对齐结果：启动后置 true，结束并处理结果时清为 false。
    bool temp1CargoRetrieved = false; // TEMP1 在粗加工放料后短暂等待并直接取回，成功后才允许导航。
    bool temp1RouteCompleted = false; // 首轮暂存区导航完成标志，后续循环直接进入对齐或放料。
    bool temp1VisionRequested = false; // 首轮暂存区对齐已启动，等待结果后清除。
    bool disc2RouteCompleted = false;
    int disc2PreparedProgress = 0; // 第二轮独立记录观察姿态恢复进度，不沿用首轮记录。
    bool disc2MaterialRequested = false;
    bool coarse2RouteCompleted = false;
    bool coarse2VisionRequested = false;
    bool temp2CargoRetrieved = false; // 第二轮粗加工物料已全部取回，允许前往暂存区码垛。
    bool temp2RouteCompleted = false;
    bool temp2VisionRequested = false;
    bool homeRouteCompleted = false;
    bool homeVisionRequested = false; // 返航角点对齐已启动，尚待主状态机处理结果。
    StartZone homeStartZone = START_ZONE_UNKNOWN; // 保存本次开局区域，返航导航据此选择目标节点。

    while (1) {
        switch (currentState) {


        case STATE_WAIT_START: // 等待开始区域
            Serial.println("TASK start");
            InitArm_start();// 初始化机械臂
            updateDisplay("DISPLAY", "DEBUG", "WAIT start_zone");
            while (currentStartZone == START_ZONE_UNKNOWN) vTaskDelay(100 / portTICK_PERIOD_MS); // 等待收到启停区信息，期间让出 CPU。
            homeStartZone = currentStartZone; // 锁定本次返航使用的开局区域。
            switch (homeStartZone) {
            case START_ZONE_1:
                updateDisplay("DISPLAY", "DEBUG", "start_zone: 1");
                break;
            case START_ZONE_2:
                updateDisplay("DISPLAY", "DEBUG", "start_zone: 2");
                break;

            default:
                updateDisplay("DISPLAY", "DEBUG", "ERR:start_zone: unknown");
                break;
            }
            // 开局位姿由雷达共用入口持锁初始化，自动/串口入口不会相互覆盖。
            // Release 开局自动执行一次。动作未标定或执行失败时返回 ERR，
            // 仍停留在开局等待阶段，便于通过串口修正后再次手动调用。
            runLidarPoseAction();
            updateDisplay("DISPLAY", "DEBUG", "WAIT START");
            while (!enableRun) vTaskDelay(100 / portTICK_PERIOD_MS); // 收到运行许可后才进入扫码和正式搬运流程。
            if (xHomeTimer != NULL) xTimerStart(xHomeTimer, 0); // 启动任务总超时计时；超时回调会停止动作并等待人工处理。
            currentState = STATE_READ_TASK; // 开局等待结束，下一轮循环进入任务码读取。
            break;

        case STATE_READ_TASK: { // 读取任务码
            updateDisplay("DISPLAY", "DEBUG", "READ TASK");
            InitArm_start();
            constexpr float SCAN_APPROACH_MM = 700.0f;   // 扫码前沿指定方向接近二维码区域的距离（毫米）
            constexpr float SCAN_RANGE_MM = 400.0f;      // 每次往返搜索二维码时移动的距离（毫米）
            constexpr uint8_t SCAN_MAX_RETRIES = 3;      // 首次扫描失败后允许的最大重试次数
            // 按实车方向：启停区1向左扫码（+Y），启停区2向右扫码（-Y）。
            // 本次扫码固定使用同一个区号，接近和往返扫描统一应用方向。
            const StartZone scanStartZone = currentStartZone;
            const float scanDirection = scanStartZone == START_ZONE_1 ? 1.0f : -1.0f;
            const auto pollTaskCode = []() -> bool { // 移动扫码期间轮询；识别成功后通知运动接口提前结束搜索。
                if (taskReceived) return true;
                char scanCode[SCANNER_BUF_LEN];
                if (waitScannerCode(scanCode, sizeof(scanCode), 0)) { // 超时参数为 0，只读取当前已收到的扫码结果。
                    const TaskCode scannedTask = parseTaskCode(scanCode);
                    portENTER_CRITICAL(&taskCodeMux); // 与串口手动提交任务码共用临界区。
                    // 手动码已提交时，迟到的扫码结果不能覆盖它。
                    if (!taskReceived && scannedTask.valid) {
                        currentTask = scannedTask;
                        taskReceived = true; // 任务码提交完成，后续扫码回调可立即报告成功。
                    }
                    portEXIT_CRITICAL(&taskCodeMux);
                    Serial.printf("[SCANNER] task code %s\n", scannedTask.valid ? "OK" : "ERR");
                }
                return taskReceived;
            };
            if (!taskReceived) {
                if (scanStartZone != START_ZONE_1 && scanStartZone != START_ZONE_2) {
                    Serial.println("[SCANNER] ERR: invalid start zone");
                } else if (!MoveScanApproach(scanDirection * SCAN_APPROACH_MM,
                                            scanDirection * SCAN_RANGE_MM,
                                            SCAN_APPROACH_SPEED_RPM, SCAN_SEARCH_SPEED_RPM, pollTaskCode)) {
                    Serial.println("[SCANNER] ERR: approach/first scan failed");
                } else {
                    // 接近与首次扫码连续执行；失败后在同一区域反向重试三遍。
                    for (uint8_t attempt = 1; attempt <= SCAN_MAX_RETRIES; ++attempt) {
                        if (pollTaskCode()) break;
                        const float distance = scanDirection * ((attempt % 2 == 0) ? SCAN_RANGE_MM : -SCAN_RANGE_MM); // 奇数次反向搜索、偶数次正向搜索，往返覆盖同一区域。
                        Serial.printf("[SCANNER] scan %u/%u, distance=%.0f mm\n", static_cast<unsigned>(attempt), static_cast<unsigned>(SCAN_MAX_RETRIES), distance);
                        const bool completed = MoveScanPosition(distance, SCAN_SEARCH_SPEED_RPM, pollTaskCode);
                        if (pollTaskCode()) break; // 先判断是否已识别，避免把识别触发的提前结束当作移动失败。
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
                currentState = STATE_SCAN_FAILED; // 接近和重试结束仍未获得任务码，进入扫码失败状态。
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
            scanPosePending=true; // 首段导航沿用扫码停车位置，偏移并入后续路程。
            xQueueReset(xVisualTaskQueue); // 清残留信号
            currentState = STATE_GRAB_ROUND1; // 任务码就绪，开始首轮圆盘取料。
            break;
        }

        case STATE_SCAN_FAILED: // 扫码失败
        case STATE_ROUTE_FAILED: // 路径失败，同样保持故障状态
        case STATE_ALIGN_FAILED:// 定位失败
        case STATE_TIMEOUT_FAILED: // 总超时回调设置的故障状态。
        case STATE_TRANSFER_FAILED: // 机械臂或物料交接接口返回失败。
            // 保持故障状态，避免下一轮状态机再次启动扫码或执行残留路径。
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;

        case STATE_GRAB_ROUND1: { // 抓取第一轮物料
            // 此状态会轮询抓取进度，路径成功后只执行一次，避免重复行驶。
            // 原料区为旋转电动转盘(6-10s/圈, 转向随机, 物料120°分布)
            // COLOR 匹配下一件任务颜色时抓取，动作成功并登记载物台后推进 roundProgress。
            // 规则: 每次抓1个; 物料必须放到机器人上才能抓下一个
            // 不允许手爪夹持运送
            updateDisplay("DISPLAY", "DEBUG", discMaterialRequested ? "WAIT DISC MATERIAL" : "GRAB R1");
            //调取接口获取路径, 并移动到目标位置
            if (!discRouteCompleted) {
                if (!requestAndMoveNodePath(SCAN_AREA_NODE, DISC_AREA_NODE)) {
                    updateDisplay("DISPLAY", "DEBUG", "ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);// 路径失败后, 停止定时器
                    currentState = STATE_ROUTE_FAILED;
                    break; // 失败时跳过下面的用户代码
                }
                discRouteCompleted = true; // 记录本段导航成功，下一轮不再重复行驶。
                roundProgress = 0;
                firstDiscGrabReady = false; // 圆盘定位阶段暂不接受物料抓取。
                // 先清除旧反馈并开启连续闭环，再通知机载电脑发送 DISC 数据。
                // 路径执行完后请求圆盘定位视觉；上位机应切到圆盘定位流程。
                if (!InitArm_look2()) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                requestVisionStart(VisionStartMode::DISC); // 启动圆盘自动对齐，并请求机载电脑发送圆盘定位反馈。

            }
            //走到这里时路径已执行成功，可继续轮询后续任务进度。
            // 等待圆盘定位完成，再切换到物料识别；实际对齐由视觉对齐任务执行。
            // false 表示尚未请求物料识别；置为 true 后，后续循环跳过整个分支。
            if (!discMaterialRequested) {
                const auto alignmentState = getAutoAlignmentState(); // 保存当前对齐结果；后面关闭对齐会重置内部状态，但不影响此局部副本。
                if (alignmentState == AutoAlignmentState::WAITING) break; // 对齐仍在进行：只退出本轮 switch，主循环延时后再次检查，不执行下方动作。
                setAlignmentEnabled(false); // 对齐已结束：关闭底盘视觉闭环，清空反馈、重置控制器并停车。
                requestVisionStop(VisionStartMode::DISC); // 通知机载电脑停止圆盘定位功能；这里只发送请求，不等待相机停止确认。
                // 只有 DONE 允许继续；FAILED 或意外的 IDLE 都按定位失败处理。
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ALIGN ERR"); // 在显示屏上提示圆盘定位失败。
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0); // 若总超时定时器已创建，则请求停止，避免其随后触发回家流程。
                    currentState = STATE_ALIGN_FAILED; // 下一轮主循环进入定位故障状态，保持停车。
                    break; // 退出本轮 switch，不再执行下面的机械臂动作和物料识别启动。
                }
                xQueueReset(xVisualTaskQueue); // 定位成功：清空视觉任务队列，丢弃定位阶段残留的颜色结果。
                vTaskDelay(pdMS_TO_TICKS(10)); // 圆盘定位结束后等待 10 ms，再调整到物料观察姿态。
                // look3 实测转台进入 90°±5° 后才启动新识别；开放前继续丢弃旧帧。
                if (!InitArm_look3()) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                firstDiscGrabReady = false;
                requestVisionStart(VisionStartMode::DISC_MATERIAL); // 启用物料识别接收状态，并向机载电脑发送圆盘物料识别启动请求。
                // 串口任务先排空 look3 完成前已经到达的数据及跨边界半帧，
                // 再置 firstDiscGrabReady；因此只响应新识别会话的新鲜数据。
                discMessageResumeRequested = true;
                discMaterialRequested = true; // 标记请求已发送，避免下一轮重复调整机械臂和启动识别；不代表识别已完成。
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL"); // 显示正在等待圆盘物料识别结果。
                // 后续由串口 COLOR 信息驱动抓取；第三件成功后关闭物料识别，
                // 并将 roundProgress 推进到 3，由下方判断切换主状态。
            }
            // COLOR 的匹配和抓取由串口任务执行，此处只衔接每件完成后的观察准备。
            // 抓取开始时结束旧位置识别；成功放上载物台才增加进度。
            // 使用进度变化触发，避免等待下一种颜色时反复调用机械臂初始化。
            const int completedGrabs = roundProgress;
            if (completedGrabs > discPreparedProgress && completedGrabs < 3) {
                Serial.printf("[DISC] Restoring observation after grab %d\n", completedGrabs);
                if (!InitArm_look3()) {
                    firstDiscGrabReady = false;
                    requestVisionStop(VisionStartMode::DISC_MATERIAL);
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_GRAB_ROUND1) break; // 恢复观察期间若总定时器已切换状态，不再恢复颜色处理。
                xQueueReset(xVisualTaskQueue);
                discPreparedProgress = completedGrabs; // 记录本次已恢复的进度，避免同一件物料重复触发观察准备。
                // 圆盘已旋转：开启一轮新的物料位置识别，再清理启停边界附近的
                // 串口旧数据；只有新识别会话后续上报的位置才能触发下一件。
                requestVisionStart(VisionStartMode::DISC_MATERIAL);
                discMessageResumeRequested = true;
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL");
            }


            if (roundProgress >= 3 ) { // 三件物料均已完成抓取并登记载物台后，结束首轮取料。
                firstDiscGrabReady = false;
                roundProgress = 0;
                currentState = STATE_PLACE_COARSE1; // 载物台备齐首轮物料，转入粗加工区放料。
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
                requestVisionStop(VisionStartMode::DISC_MATERIAL); // 离开圆盘前关闭物料识别，避免导航途中继续产生颜色结果。
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
                vTaskDelay(10);
                if (currentState != STATE_PLACE_COARSE1 || taskMotionAborted) break; // 动作期间若发生超时或状态切换，不再启动下一阶段视觉。
                requestVisionStart(VisionStartMode::WORK_AREA); // 识别空工位，为首轮粗加工放料提供底盘对齐反馈。
                coarse1VisionRequested = true; // 只登记请求已发出，对齐是否成功由下方分支轮询。
                break;
            }

            if (coarse1VisionRequested) {
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break; // 等待期间退出本轮 switch，下次循环继续检查结果。
                setAlignmentEnabled(false); // 取出对齐结果后停车，避免放料时视觉继续驱动底盘。
                requestVisionStop(VisionStartMode::WORK_AREA);
                coarse1VisionRequested = false; // 本次对齐结果已取出，后续循环不再重复等待。
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

            currentState = STATE_PLACE_TEMP1; // 首轮粗加工放料完成，下一状态先取回物料再去暂存区。
            break;
        }

        case STATE_PLACE_TEMP1: { // 首轮：粗加工区取回 → 暂存区导航和对齐 → 第一层放料。
            if (!temp1CargoRetrieved) {
                setAlignmentEnabled(false);
                if (taskMotionAborted) break;
                updateDisplay("DISPLAY", "DEBUG", "WAIT RETRIEVE C1");
                vTaskDelay(pdMS_TO_TICKS(10)); // 粗加工放料后短暂等待，再执行取回动作。
                if (currentState != STATE_PLACE_TEMP1 || taskMotionAborted) break;
                updateDisplay("DISPLAY", "DEBUG", "RETRIEVE C1");
                // round1_pos[i] 的物料取回载物台 i+1，保持颜色和位置的对应关系。
                const bool retrieved = RetrieveRoundToCargo(
                    currentTask.round1_colors, currentTask.round1_pos);
                // requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
                if (currentState != STATE_PLACE_TEMP1) break; // 取料期间状态可能已被超时流程修改，先保留其处理结果。
                if (!retrieved) {
                    updateDisplay("DISPLAY", "DEBUG", "RETRIEVE C1 ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                temp1CargoRetrieved = true; // 三件取回动作均成功，后续循环跳过取料分支。
                roundProgress = 0;
                break; // 本轮只完成取料，下一轮再执行暂存区导航。
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
                temp1RouteCompleted = true; // 记录到达暂存区，后续循环不再重复导航。
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
                temp1VisionRequested = true; // 进入首轮暂存区对齐等待阶段。
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
            const bool placed = PlaceTaskCargoToWorkArea(currentTask.round1_pos, 1); // 按首轮位置码将三件物料放在暂存区第一层。
            if (currentState != STATE_PLACE_TEMP1) break;
            if (!placed) {
                updateDisplay("DISPLAY", "DEBUG", "PLACE T1 ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            roundProgress = 0;
            currentState = STATE_GRAB_ROUND2; // 首轮暂存完成，返回圆盘区抓取第二轮物料。
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
                disc2RouteCompleted = true; // 第二轮圆盘导航只执行一次，独立于首轮路径标志。
                roundProgress = 0;
                firstDiscGrabReady = false; // 第二轮同样先定位圆盘，再开放物料抓取。
                // 先清除旧反馈并开启连续闭环，再通知机载电脑发送 DISC 数据。
                // 路径执行完后请求圆盘定位视觉；上位机应切到圆盘定位流程。
                if (!InitArm_look2()) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                requestVisionStart(VisionStartMode::DISC); // 重新开启圆盘定位，使用第二次到达后的视觉反馈。

            }
            //走到这里时路径已执行成功，可继续轮询后续任务进度。
            // 等待圆盘定位完成，再切换到物料识别；实际对齐由视觉对齐任务执行。
            // false 表示尚未请求物料识别；置为 true 后，后续循环跳过整个分支。
            if (!disc2MaterialRequested) {
                const auto alignmentState = getAutoAlignmentState(); // 保存当前对齐结果；后面关闭对齐会重置内部状态，但不影响此局部副本。
                if (alignmentState == AutoAlignmentState::WAITING) break; // 对齐仍在进行：只退出本轮 switch，主循环延时后再次检查，不执行下方动作。
                setAlignmentEnabled(false); // 对齐已结束：关闭底盘视觉闭环，清空反馈、重置控制器并停车。
                requestVisionStop(VisionStartMode::DISC); // 通知机载电脑停止圆盘定位功能；这里只发送请求，不等待相机停止确认。
                // 只有 DONE 允许继续；FAILED 或意外的 IDLE 都按定位失败处理。
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ALIGN ERR"); // 在显示屏上提示圆盘定位失败。
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0); // 若总超时定时器已创建，则请求停止，避免其随后触发回家流程。
                    currentState = STATE_ALIGN_FAILED; // 下一轮主循环进入定位故障状态，保持停车。
                    break; // 退出本轮 switch，不再执行下面的机械臂动作和物料识别启动。
                }
                xQueueReset(xVisualTaskQueue); // 定位成功：清空视觉任务队列，丢弃定位阶段残留的颜色结果。
                vTaskDelay(pdMS_TO_TICKS(10)); // 圆盘定位结束后等待 10 ms，再调整到物料观察姿态。
                // look3 实测转台进入 90°±5° 后才启动新识别；开放前继续丢弃旧帧。
                if (!InitArm_look3()) {
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                firstDiscGrabReady = false;
                requestVisionStart(VisionStartMode::DISC_MATERIAL); // 启用物料识别接收状态，并向机载电脑发送圆盘物料识别启动请求。
                // 串口任务先排空 look3 完成前已经到达的数据及跨边界半帧，
                // 再置 firstDiscGrabReady；因此只响应新识别会话的新鲜数据。
                discMessageResumeRequested = true;
                disc2MaterialRequested = true; // 标记请求已发送，避免下一轮重复调整机械臂和启动识别；不代表识别已完成。
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL"); // 显示正在等待圆盘物料识别结果。
                // 后续由串口 COLOR 信息驱动抓取；第三件成功后关闭物料识别，
                // 并将 roundProgress 推进到 3，由下方判断切换主状态。
            }
            // COLOR 的匹配和抓取由串口任务执行，此处只衔接每件完成后的观察准备。
            // 抓取开始时结束旧位置识别；成功放上载物台才增加进度。
            // 使用进度变化触发，避免等待下一种颜色时反复调用机械臂初始化。
            const int completedGrabs = roundProgress;
            if (completedGrabs > disc2PreparedProgress && completedGrabs < 3) {
                Serial.printf("[DISC] Restoring observation after grab %d\n", completedGrabs);
                if (!InitArm_look3()) {
                    firstDiscGrabReady = false;
                    requestVisionStop(VisionStartMode::DISC_MATERIAL);
                    updateDisplay("DISPLAY", "DEBUG", "DISC ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_GRAB_ROUND2) break; // 恢复观察期间若总定时器已切换状态，不再恢复颜色处理。
                xQueueReset(xVisualTaskQueue);
                disc2PreparedProgress = completedGrabs; // 更新第二轮观察准备记录，等待下一件的新识别数据。
                // 圆盘已旋转：开启一轮新的物料位置识别，再清理启停边界附近的
                // 串口旧数据；只有新识别会话后续上报的位置才能触发下一件。
                requestVisionStart(VisionStartMode::DISC_MATERIAL);
                discMessageResumeRequested = true;
                updateDisplay("DISPLAY", "DEBUG", "WAIT DISC MATERIAL");
            }


            if (roundProgress >= 3 ) { // 第二轮三件物料全部上车后，再进入粗加工流程。
                firstDiscGrabReady = false;
                roundProgress = 0;
                currentState = STATE_PLACE_COARSE2; // 第二轮载物完成，前往粗加工区放料。
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
                requestVisionStop(VisionStartMode::DISC_MATERIAL); // 离开圆盘前关闭物料识别，避免导航途中继续产生颜色结果。
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
                coarse2VisionRequested = true; // 登记第二轮粗加工对齐请求，后续循环等待结果。
                break;
            }

            if (coarse2VisionRequested) {
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break; // 对齐仍在进行，暂不执行粗加工放料。
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::WORK_AREA);
                coarse2VisionRequested = false; // 本次对齐结果已取出，结束等待阶段。
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

            currentState = STATE_STACK_TEMP2; // 第二轮粗加工放料完成，转入取回和暂存区码垛。
            break;
        }

        case STATE_STACK_TEMP2: { // 第二轮：粗加工区取回 → 暂存区带物料对齐 → 第二层码垛。
            if (!temp2CargoRetrieved) {

                setAlignmentEnabled(false);
                if (taskMotionAborted) break;
                updateDisplay("DISPLAY", "DEBUG", "WAIT RETRIEVE C2");
                vTaskDelay(pdMS_TO_TICKS(10)); // 粗加工放料后短暂等待，再取回第二轮物料。
                if (currentState != STATE_STACK_TEMP2 || taskMotionAborted) break;
                updateDisplay("DISPLAY", "DEBUG", "RETRIEVE C2");
                // round2_pos[i] 的物料取回载物台 i+1，保持颜色和位置的对应关系。
                const bool retrieved = RetrieveRoundToCargo(
                    currentTask.round2_colors, currentTask.round2_pos);
                // requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
                if (currentState != STATE_STACK_TEMP2) break; // 取回动作结束后先检查状态，避免覆盖异步故障处理结果。
                if (!retrieved) {
                    updateDisplay("DISPLAY", "DEBUG", "RETRIEVE C2 ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                temp2CargoRetrieved = true; // 第二轮三件物料已取回，允许开始码垛区导航。
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
                temp2RouteCompleted = true; // 记录到达暂存区，下一阶段等待带物料工位对齐。
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
                requestVisionStart(VisionStartMode::WORK_AREA_LOADED); // 暂存区已有首轮物料，切换到带物料工位识别和自动对齐。
                temp2VisionRequested = true; // 请求发出后先等待对齐结果，不立即执行码垛。
                break;
            }

            if (temp2VisionRequested) {
                updateDisplay("DISPLAY", "DEBUG", "ALIGN TEMP2");
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break; // 等待带物料对齐达标，期间不启动机械臂码垛。
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::WORK_AREA_LOADED);
                temp2VisionRequested = false; // 本次对齐结果已取出，结束码垛前的等待阶段。
                if (alignmentState != AutoAlignmentState::DONE) {
                    updateDisplay("DISPLAY", "DEBUG", "TEMP ALIGN ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ALIGN_FAILED;
                    break;
                }
            }

            updateDisplay("DISPLAY", "DEBUG", "STACK T2");
            const bool placed = PlaceTaskCargoToWorkArea(currentTask.round2_pos, 2); // 按第二轮位置码放置第二层，高度由搬运接口叠加层间偏移。
            if (currentState != STATE_STACK_TEMP2) break;
            if (!placed) {
                updateDisplay("DISPLAY", "DEBUG", "STACK T2 ERR");
                if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                currentState = STATE_TRANSFER_FAILED;
                break;
            }
            roundProgress = 0;
            currentState = STATE_RETURN_HOME; // 两轮暂存及码垛完成，进入返航流程。
            break;
        }

        case STATE_RETURN_HOME: { // 返航：回到开局节点 → 角点对齐 → 机械臂复位及末段位移。
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
                firstDiscGrabReady = false; // 返航前关闭圆盘抓取接收，避免途中触发取料。
                requestVisionStop(VisionStartMode::DISC_MATERIAL);
                setAlignmentEnabled(false);
                if (!InitArm_start()) {
                    updateDisplay("DISPLAY", "DEBUG", "HOME ARM ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_RETURN_HOME) break;
                const bool moved = requestAndMoveNodePath(TEMP_AREA_NODE, homeNode, homeHeading); // 从暂存区返回保存的开局节点，并使用指定终点朝向。
                if (currentState != STATE_RETURN_HOME) break;
                if (!moved) {
                    updateDisplay("DISPLAY", "DEBUG", "HOME ROUTE ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_ROUTE_FAILED;
                    break;
                }
                homeRouteCompleted = true; // 返航路径执行完成，后续循环只处理角点对齐与收尾。
                // 回到启停区后先将机械臂调整到角点识别位，再启动 CORNER 视觉。
                if (!InitArm_look4()) {
                    updateDisplay("DISPLAY", "DEBUG", "HOME LOOK ERR");
                    if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
                    currentState = STATE_TRANSFER_FAILED;
                    break;
                }
                if (currentState != STATE_RETURN_HOME || taskMotionAborted) break;
                requestVisionStart(VisionStartMode::CORNER); // 开启角点自动对齐，请求机载电脑发送 CORNER 定位反馈。
                homeVisionRequested = true; // 角点请求已发送，下一轮开始轮询对齐结果。
                break;
            }
            if (homeVisionRequested) {
                updateDisplay("DISPLAY", "DEBUG", "ALIGN HOME");
                const auto alignmentState = getAutoAlignmentState();
                if (alignmentState == AutoAlignmentState::WAITING) break; // 角点未达标时继续等待，不提前执行复位和末段位移。
                setAlignmentEnabled(false);
                requestVisionStop(VisionStartMode::CORNER);
                homeVisionRequested = false; // 本次角点对齐结果已取出，退出等待阶段。
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
            if (currentStartZone == START_ZONE_1) { // 末段位移按当前区号选择横向方向，GotoPose 的 true 表示相对移动。
                GotoPose(-70, 0, 0, true); // 沿当前车体 X 方向相对移动 -70 mm。
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, -200, 0, true); // 区 1 沿当前车体 Y 方向相对移动 -200 mm。
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            } else if (currentStartZone == START_ZONE_2) {
                GotoPose(-70, 0, 0, true); // 沿当前车体 X 方向相对移动 -70 mm。
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                GotoPose(0, 200, 0, true); // 区 2 沿当前车体 Y 方向相对移动 +200 mm。
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            if (currentState != STATE_RETURN_HOME) break; // 末段动作结束后再次确认状态，避免将异步故障覆盖为完成。
            roundProgress = 0;
            if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0); // 正常结束前停止总超时计时。
            currentState = STATE_DONE; // 所有收尾动作结束，切换到完成状态。
            break;
        }

        case STATE_DONE:
            updateDisplay("DISPLAY", "DEBUG", "DONE");
            if (xHomeTimer != NULL) xTimerStop(xHomeTimer, 0);
            Emm_V5_En_Control_all(false); // 完成后调用电机统一接口关闭使能。
            vTaskDelay(100000000 / portTICK_PERIOD_MS); // 保持
            break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // 状态机每轮让出 CPU，视觉和串口任务可继续更新结果。
    }
}

// ================= setup  =================
void setup() {
    pinMode(BOOT0_PIN, INPUT_PULLUP);
    const bool ledPwmReady = LedPwm_Init();
    Serial.begin(115200);
    if (!ApplyCompiledParameterDefaults()) {
        Serial.println("[CFG] ERR: compiled defaults not applied; continuing startup");
    }
    if (!ledPwmReady) Serial.println("[LED] PWM initialization failed");
    Serial.printf("version: %s\n", VERSION);
    Servo_Init();    // 总线舵机初始化 (默认 Serial2: RX=16, TX=15, 115200bps)
    Emm_V5_Init();   // 电机初始化
    Scanner_Init();  // 扫码模块初始化 (软串口 RX=IO4, 9600bps)
    if (!H30UsbBegin()) Serial.println("[H30] USB Host initialization failed");
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(10);

    currentPose = {0, 0, 0};   // [TODO] 初始位姿按实际
    currentArm = {0, 0, 0, 0}; // 升降/伸缩目标坐标对应驱动器回零后的原点，舵机角度由下方实读更新。
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

    xVisualTaskQueue = xQueueCreate(10, sizeof(VisualCmd_t)); // 机载电脑指令队列(深度10)
    xLidarPoseMutex = xSemaphoreCreateMutex();
    xAlignmentQueue = xQueueCreate(1, sizeof(VisualAlignmentFrame_t)); // 视觉 PID 对齐只保留 20 Hz 连续反馈中的最新一帧。
    xAlignmentMotionMutex = xSemaphoreCreateMutex();
    if (!ContinuousTrialInit()) Serial.println("[Trial] task initialization failed");
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
