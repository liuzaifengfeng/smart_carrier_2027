type RouteState = 'idle' | 'waiting' | 'running' | 'done' | 'failed' | 'cancelled'

export interface MapTelemetry {
  base: number[]
  arm: number[]
  poseSource: '' | '回读' | '估算' | '导航估算'
  phase: string
  roundProgress?: number
  routeJob?: number
  routeState: RouteState
  routeNodes: number[]
  endpoints: number[]
  routePhase: string
  distance?: number
}

export function emptyMapTelemetry(): MapTelemetry {
  return { base: [], arm: [], poseSource: '', phase: '', routeState: 'idle',
    routeNodes: [], endpoints: [], routePhase: '' }
}

function numbers(fields: string[], count: number): number[] | undefined {
  if (fields.length !== count || fields.some(value => !value.trim())) return
  const values = fields.map(Number)
  return values.every(Number.isFinite) ? values : undefined
}

export function updateMapTelemetry(state: MapTelemetry, fields: string[]) {
  const [kind, group, action, ...data] = fields
  const setBase = (base: number[], source: MapTelemetry['poseSource']) => {
    state.base = base
    state.poseSource = source
  }
  if (kind === 'RSP' && group === 'POSE') {
    if (action === 'GET' && data[0] === 'OK') {
      const pose = numbers(data.slice(1), 7)
      if (pose) {
        setBase(pose.slice(0, 3), '回读')
        state.arm = pose.slice(3)
      }
    } else if (action === 'SET' && data[0] === 'ACK') {
      const base = numbers(data.slice(1), 3)
      if (base) setBase(base, '估算')
    }
    return
  }
  if (kind !== 'EVT') return
  if (group === 'POSE' && action === 'ESTIMATE') {
    const base = numbers(data, 3)
    if (base) setBase(base, '估算')
  } else if (group === 'ARM' && action === 'POSE') {
    const arm = numbers(data, 4)
    if (arm) state.arm = arm
  } else if (group === 'MOTION') {
    if (action === 'PHASE' && data[0] === '0' && data[2]) state.phase = data[2]
    if (action === 'FLAG' && data[0] === '0' && data[2] === 'ROUND_PROGRESS') {
      state.roundProgress = numbers(data.slice(3), 1)?.[0]
    }
    if (action === 'RESULT' && state.routeJob === Number(data[0]) &&
        ['DONE', 'FAILED', 'CANCELLED'].includes(data[2] ?? '')) {
      state.routeState = data[2]!.toLowerCase() as RouteState
    }
  } else if (group === 'NAV') {
    if (action === 'ROUTE_WAITING' || action === 'ROUTE_REQUEST') {
      state.routeJob = undefined
      state.routeState = 'waiting'
      state.routeNodes = []
      state.endpoints = numbers(data, 2) ?? []
      state.routePhase = ''
      state.distance = undefined
    } else if (action === 'ROUTE_PATH') {
      const nodes = numbers(data.slice(1), data.length - 1)
      if (nodes && nodes.length >= 2 && nodes.every(node => Number.isInteger(node) && node >= 0 && node < 25)) {
        state.routeJob = Number(data[0])
        state.routeNodes = nodes
        state.routeState = 'running'
        state.routePhase = ''
        state.distance = undefined
      }
    } else if (action === 'ROUTE_RUNNING') {
      if (state.routeState !== 'running') {
        state.routeJob = undefined
        state.routeNodes = []
        state.routePhase = ''
        state.distance = undefined
      }
      state.routeState = 'running'
    } else if (action === 'ROUTE_PHASE') {
      state.routePhase = data[1] ?? ''
    } else if (action === 'ROUTE_POSITION') {
      const position = numbers(data, 5)
      if (position) {
        state.distance = position[0]
        setBase([position[1]!, position[2]!, position[4]!], '导航估算')
      }
    } else if (action === 'ROUTE_RESULT' || action === 'TRIAL_RESULT') {
      const base = numbers(data.slice(3), 3)
      if (base) setBase(base, '导航估算')
      if (action === 'ROUTE_RESULT') state.routeState = data[1] === 'DONE' ? 'done' : 'failed'
    } else if (action === 'ROUTE_DONE' || action === 'ROUTE_FAILED') {
      state.routeState = action === 'ROUTE_DONE' ? 'done' : 'failed'
    }
  }
}
