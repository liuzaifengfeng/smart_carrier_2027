type Field = [label: string, initial: string, options?: Record<string, string>]

export interface Operation {
  title: string
  category: string
  action: string
  fields: Field[]
}
const op = (title: string, category: string, action: string, fields: Field[] = []): Operation =>
  ({ title, category, action, fields })

const visionMode: Field = ['视觉模式', 'DISC', {
  DISC: '圆盘定位',
  DISC_MATERIAL: '圆盘物料识别',
  WORK_AREA: '粗加工区／暂存区定位',
  WORK_AREA_LOADED: '粗加工区／暂存区带物料定位',
  CORNER: '角点视觉识别',
}]
export const operations: Record<string, Operation[]> = {
  姿态: [
    op('启停区', 'NAV', 'START_ZONE', [['区域 1 / 2', '1']]),
    op('手动校准理想位置', 'POSE', 'SET', [['X / mm', '2250'], ['Y / mm', '150'], ['航向 / °', '180']]),
    op('相对目标姿态', 'POSE', 'GOTO_REL', [['X / mm', '0'], ['Y / mm', '0'], ['转角 / °', '0']]),
    op('调整到雷达扫描位姿', 'VISION', 'LIDAR_POSE'),
    op('扫描完成，继续运行', 'SYS', 'START'),
    op('准备运行', 'SYS', 'READY'),
  ],
  底盘: [
    op('麦轮位置移动', 'CHASSIS', 'MOVE_POSITION', [['X / mm', '0'], ['Y / mm', '0'], ['转角 / °', '0'], ['轮速 / RPM', '80']]),
    op('方向运动', 'CHASSIS', 'MOVE', [['方向 0前 1后 2左 3右 4左转 5右转', '0'], ['RPM', '80'], ['停止 0 / 1', '0']]),
    op('电机使能', 'ARM', 'ENABLE'),
    op('连续视觉对齐', 'VISION', 'ALIGN_START', [visionMode]),
    op('停止并清零对齐', 'VISION', 'ALIGN_STOP'),
    op('节点路径', 'NAV', 'ROUTE', [['节点（逗号分隔）', '0,1,8']]),
    op('停止导航', 'NAV', 'STOP'),
    op('固件帮助', 'SYS', 'HELP'),
  ],
  机械臂: [
    op('起始姿态', 'ARM', 'INIT_START'), op('观察姿态', 'ARM', 'INIT_LOOK'), op('观察姿态 2', 'ARM', 'INIT_LOOK2'),
    op('升降 / 伸出', 'ARM', 'MOVE1', [['高度 / mm（-1跳过）', '-1'], ['伸出 / mm（-1跳过）', '-1'], ['速度', '80']]),
    op('转台 / 夹爪', 'ARM', 'MOVE2', [['转台 / °（-1跳过）', '-1'], ['夹爪 / °（-1跳过）', '-1'], ['速度', '80']]),
    op('机械臂角度控制', 'ARM', 'SERVO', [['逻辑轴 1 夹爪 / 2 EMM 转台', '1'], ['角度 / °', '0']]),
    op('机械臂当前位置设零点', 'ARM', 'SERVO_ZERO', [['逻辑轴 1 夹爪 / 2 EMM 转台 / 5 升降 / 6 伸出', '2']]),
    ...['放料', '码放', 'Demo 3', '暂存区取回'].map((name, i) => op(name, 'ARM', 'DEMO' + (i + 1))),
    op('读取舵机 PID', 'ARM', 'SERVO_PID_GET', [['总线地址 1 夹爪 / 3 料盘', '1']]),
    op('写入舵机 PID', 'ARM', 'SERVO_PID_SET', [['总线地址 1 夹爪 / 3 料盘', '1'], ...['Kp', 'Kd', 'Ki', 'PwmBias', 'HoldKp', 'HoldKd', 'HoldPwmBias'].map(name => [name, ''] as [string, string])]),
  ],
  视觉: [
    op('下发任务码', 'TASK', 'SET', [['任务码', '156+123+516+231']]),
    op('开启视觉请求', 'VISION', 'START_REQUEST', [visionMode]),
    op('结束视觉请求', 'VISION', 'STOP_REQUEST', [visionMode]),
    op('回传对齐偏差', 'VISION', 'ALIGN_DATA', [['角度', '0'], ['X', '0'], ['Y', '0']]),
    op('回传颜色', 'VISION', 'COLOR', [['颜色 1～6', '1']]),
    op('抓取', 'VISION', 'GRAB', [['颜色', '1'], ['载物台', '1']]),
    op('强制抓取', 'VISION', 'FORCE_GRAB', [['颜色', '1'], ['载物台', '1']]),
    op('上报可抓位置', 'VISION', 'DISC_POSITIONS', [['位置,颜色（可多组）', '1,1,2,2,3,3']]),
    op('视觉确认', 'VISION', 'CONFIRM'),
  ],
}
export function frame(category: string, action: string, values: (string | number)[] = []) {
  return '{CMD,' + [category, action, ...values].join(',') + '}'
}
