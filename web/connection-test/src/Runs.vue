<script setup lang="ts">
import { computed, nextTick, onMounted, ref, watch } from 'vue'
import { CloudError, cloudRequest, type LogSnapshot, type RunDetail, type RunOutline } from './cloudRuns'
import { activityName, date, delta, duration, phaseActivities, phaseName, phases, runStatus, runTiming, seconds } from './runDisplay'
import RunTimeline from './components/RunTimeline.vue'
import RunActionStats from './components/RunActionStats.vue'
import RunNote from './components/RunNote.vue'
import './runs.css'

const outlines = ref<RunOutline[]>([])
const snapshots = ref<LogSnapshot[]>([])
const snapshotError = ref('')
const first = ref('')
const second = ref('')
const runDetails = ref<RunDetail[]>([])
const snapshotDetails = ref<RunDetail[]>()
const snapshotIndex = ref(0)
const analysisSection = ref<HTMLElement>()
const details = computed(() => snapshotDetails.value
  ? snapshotDetails.value.slice(snapshotIndex.value, snapshotIndex.value + 1) : runDetails.value)
const comparing = computed(() => !snapshotDetails.value && Boolean(second.value))
const password = ref('')
const unauthorized = ref(false)
const loading = ref(true)
const error = ref('')
const busy = ref(false)
const focus = ref('')
const steps = ref(false)
const windowRange = computed(() => {
  const rows = details.value.flatMap(run => run.phases.filter(([name]) => name === focus.value))
  const ends = details.value.map(run => run.timeline_ms ?? run.total_ms ?? 0)
  if (!rows.length) return { start: 0, end: Math.max(1, ...ends) }
  const start = Math.min(...rows.map(row => row[1]))
  const end = details.value.map((run, index) => {
    const row = run.phases.find(([name]) => name === focus.value)
    return row ? row[2] ?? ends[index]! : 0
  })
  return { start, end: Math.max(start + 1, ...end) }
})
const shownPhases = computed(() => Array.from(new Set([
  ...details.value.flatMap(run => run.phases.filter(row => !phases.includes(row[0])).map(row => row[0])), ...phases,
])))
const phaseRows = computed(() => shownPhases.value.map(name => {
  const times = details.value.map(run => run.phases.find(phase => phase[0] === name)?.[3])
  const activities = details.value.map(run => phaseActivities(run, name))
  const names = Array.from(new Set(activities.flatMap(group => [...group.keys()]))).sort()
  return { name, times, activities: names.map(activity => ({
    name: activity,
    times: activities.map(group => group.get(activity)),
  })) }
}))
const longest = computed(() => Math.max(1, ...phaseRows.value.flatMap(phase => phase.times.map(ms => ms ?? 0))))
const focused = computed(() => {
  const row = phaseRows.value.find(row => row.name === focus.value)
  if (!row) return
  const coverage = (activity: typeof row.activities[number]) =>
    Math.max(...activity.times.map(time => time?.coverage ?? 0))
  return { ...row, activities: [...row.activities].sort((a, b) => coverage(b) - coverage(a)) }
})
const tickLabel = (tick: number) => {
  const { start, end } = windowRange.value
  return ((start + (end - start) * tick / 5) / 1000).toFixed(focus.value ? 1 : 0) + ' s'
}

function showError(reason: unknown) {
  if (reason instanceof CloudError && reason.status === 401) {
    unauthorized.value = true
    outlines.value = []
    snapshots.value = []
    runDetails.value = []
    snapshotDetails.value = undefined
    first.value = second.value = ''
  }
  error.value = reason instanceof Error ? reason.message : String(reason)
}

function notifyAuth() {
  const channel = new BroadcastChannel('carrier-cloud-auth')
  channel.postMessage('session-changed')
  channel.close()
}

async function load() {
  loading.value = true
  error.value = ''
  try {
    outlines.value = await cloudRequest<RunOutline[]>('runs')
    unauthorized.value = false
    if (!outlines.value.some(run => run.id === first.value)) first.value = outlines.value[0]?.id ?? ''
    if (!outlines.value.some(run => run.id === second.value)) second.value = ''
    snapshotError.value = ''
    try {
      snapshots.value = await cloudRequest<LogSnapshot[]>('runs/snapshots')
    } catch (reason) {
      snapshotError.value = reason instanceof Error ? reason.message : String(reason)
    }
  } catch (reason) {
    showError(reason)
  } finally {
    loading.value = false
  }
}

async function authenticate(logout = false) {
  busy.value = true
  error.value = ''
  try {
    await cloudRequest('carrier-session', logout ? { logout: true } : { password: password.value })
    password.value = ''
    if (logout) {
      unauthorized.value = true
      runDetails.value = []
      snapshotDetails.value = undefined
      outlines.value = []
      snapshots.value = []
      first.value = second.value = ''
    }
    notifyAuth()
    if (!logout) await load()
  } catch (reason) {
    showError(reason)
  } finally {
    busy.value = false
  }
}

function view(id: string) {
  snapshotDetails.value = undefined
  focus.value = ''
  if (id === second.value) second.value = ''
  first.value = id
}

async function viewSnapshot(id: string) {
  error.value = ''
  try {
    snapshotDetails.value = await cloudRequest<RunDetail[]>(`runs/snapshots/${id}/analysis`)
    snapshotIndex.value = 0
    focus.value = ''
    await nextTick()
    analysisSection.value?.scrollIntoView()
  } catch (reason) {
    showError(reason)
  }
}

watch([first, second], async ([a, b], _, onCleanup) => {
  let current = true
  onCleanup(() => { current = false })
  runDetails.value = []
  if (!a) return
  error.value = ''
  try {
    const runs = await Promise.all([a, b].filter(Boolean).map(id => cloudRequest<RunDetail>('runs/' + id)))
    if (current) runDetails.value = runs
  } catch (reason) {
    if (current) showError(reason)
  }
})

onMounted(async () => {
  await load()
  if (location.hash === '#log-snapshots') {
    await nextTick()
    document.getElementById('log-snapshots')?.scrollIntoView()
  }
})
</script>

<template>
  <main class="runs">
    <header>
      <div><a href="./console.html">智能搬运车上位机</a><h1>云端运行日志</h1></div>
      <button v-if="!unauthorized && !loading && !error" :disabled="busy" @click="authenticate(true)">退出队内登录</button>
    </header>
    <p v-if="error" class="error" role="alert">{{ error }}</p>
    <form v-if="unauthorized" class="panel login" @submit.prevent="authenticate()">
      <label>队内密码<input v-model="password" type="password" autocomplete="current-password" required></label>
      <button class="primary" :disabled="busy">登录</button>
    </form>
    <p v-else-if="loading" role="status">正在读取云端归档</p>
    <template v-else>
      <section id="log-snapshots" class="panel">
        <div class="section-head"><h2>手动上传的 60 秒日志</h2><button @click="load">刷新</button></div>
        <p v-if="snapshotError" class="error" role="alert">{{ snapshotError }}</p>
        <p v-else-if="!snapshots.length">暂无手动上传的日志</p>
        <ul v-else class="snapshot-list">
          <li v-for="snapshot in snapshots" :key="snapshot.id">
            <div>
              <time :datetime="snapshot.captured_at">{{ date(snapshot.captured_at) }}</time>
              · {{ snapshot.count }} 条<span v-if="snapshot.device"> · {{ snapshot.device }}</span>
              <p v-if="snapshot.note">{{ snapshot.note }}</p>
            </div>
            <div class="outline-actions">
              <button @click="viewSnapshot(snapshot.id)">分析日志</button>
              <a :href="`/api/runs/snapshots/${snapshot.id}`" :aria-label="`下载 ${date(snapshot.captured_at)} 的日志 JSON`">下载 JSON</a>
            </div>
          </li>
        </ul>
      </section>
      <section class="panel">
        <div class="section-head"><h2>运行记录</h2><button @click="load">刷新</button></div>
        <p v-if="!outlines.length">暂无云端运行记录</p>
        <div v-else class="archive">
          <details v-for="run in outlines" :key="run.id">
            <summary>
              <time :datetime="run.captured_at">{{ date(run.captured_at) }}</time>
              <strong>{{ runStatus(run) }} · {{ runTiming(run) }}</strong>
              <small v-if="run.missing.length" class="warning">{{ run.missing.length }} 项数据提示</small>
            </summary>
            <div class="outline">
              <dl><div v-for="[name, , , ms] in run.phases" :key="name"><dt>{{ phaseName(name) }}</dt><dd>{{ duration(ms) }}</dd></div></dl>
              <ul v-if="run.missing.length" class="warning"><li v-for="message in run.missing" :key="message">{{ message }}</li></ul>
              <div class="outline-actions">
                <button @click="view(run.id)">分析此场</button>
                <RunNote :run-id="run.id" :note="run.note ?? ''" @saved="run.note = $event" />
              </div>
            </div>
          </details>
        </div>
      </section>
      <section v-if="outlines.length || snapshotDetails" ref="analysisSection" class="panel">
        <div class="section-head">
          <h2>{{ snapshotDetails ? '60 秒日志分析' : '耗时分析' }}</h2>
          <button v-if="snapshotDetails" @click="snapshotDetails = undefined; focus = ''">返回运行分析</button>
        </div>
        <label v-if="snapshotDetails && snapshotDetails.length > 1">日志片段
          <select v-model.number="snapshotIndex" @change="focus = ''">
            <option v-for="(run, index) in snapshotDetails" :key="run.id" :value="index">{{ index + 1 }} · {{ date(run.captured_at) }} · {{ runStatus(run) }}</option>
          </select>
        </label>
        <div v-if="!snapshotDetails" class="selectors">
          <label>场次 A<select v-model="first" @change="view(first)">
            <option v-for="run in outlines" :key="run.id" :value="run.id">{{ date(run.captured_at) }} · {{ runStatus(run) }} · {{ runTiming(run) }}</option>
          </select></label>
          <label>场次 B<select v-model="second">
            <option value="">单场查看</option>
            <option v-for="run in outlines.filter(run => run.id !== first)" :key="run.id" :value="run.id">{{ date(run.captured_at) }} · {{ runStatus(run) }} · {{ runTiming(run) }}</option>
          </select></label>
        </div>
        <template v-if="details.length">
          <div class="timeline-controls">
            <button type="button" :aria-pressed="!focus" @click="focus = ''">全程</button>
            <label>放大阶段<select v-model="focus">
              <option value="">全程</option>
              <option v-for="name in shownPhases" :key="name" :value="name">{{ phaseName(name) }}</option>
            </select></label>
            <label v-if="focus"><input v-model="steps" type="checkbox">显示作业和机械臂步骤</label>
          </div>
          <div class="timeline-scroll" tabindex="0" aria-label="运行时间轴，可横向滚动">
            <div class="timeline">
              <div class="timeline-row ruler"><span>时间轴</span><div><span v-for="tick in 6" :key="tick">{{ tickLabel(tick - 1) }}</span></div></div>
              <RunTimeline v-for="(run, i) in details" :key="run.id + focus" :run="run" :label="i ? 'B' : 'A'"
                v-bind="windowRange" :phase="focus" :steps="steps" @focus="focus = $event" />
            </div>
          </div>
          <h3>阶段耗时</h3>
          <div class="table-scroll">
            <table class="phase-table">
              <thead><tr><th scope="col">业务阶段</th><th scope="col">耗时对照</th><th scope="col">场次 A</th><th v-if="comparing" scope="col">场次 B</th><th v-if="comparing" scope="col">B − A</th></tr></thead>
              <tbody>
                <tr class="total"><th scope="row">正式总耗时</th><td></td><td>{{ duration(details[0]?.total_ms) }}</td><td v-if="comparing">{{ duration(details[1]?.total_ms) }}</td><td v-if="comparing">{{ delta(details[0]?.total_ms, details[1]?.total_ms) }}</td></tr>
                <tr v-if="details.some(run => run.observed_ms !== undefined)"><th scope="row">已记录时长</th><td></td><td>{{ duration(details[0]?.observed_ms) }}</td><td v-if="comparing">{{ duration(details[1]?.observed_ms) }}</td><td v-if="comparing"></td></tr>
                <tr v-for="phase in phaseRows" :key="phase.name" :class="{ 'focused-row': focus === phase.name }">
                  <th scope="row"><button class="stage-link" :aria-pressed="focus === phase.name" @click="focus = focus === phase.name ? '' : phase.name">{{ phaseName(phase.name) }}</button></th>
                  <td class="stage-bars" aria-hidden="true"><span v-for="(ms, i) in phase.times" :key="i" :class="{ comparison: i === 1 }" :style="{ width: (ms ?? 0) / longest * 100 + '%' }"></span></td>
                  <td>{{ duration(phase.times[0]) }}</td><td v-if="comparing">{{ duration(phase.times[1]) }}</td><td v-if="comparing">{{ delta(phase.times[0], phase.times[1]) }}</td>
                </tr>
              </tbody>
            </table>
          </div>
          <section v-if="focused" class="stage-breakdown">
            <h3>{{ phaseName(focused.name) }} · 分项耗时</h3>
            <p v-if="!focused.activities.length">该阶段无可计算的动作区间。</p>
            <div v-else class="table-scroll">
                        <table :aria-label="phaseName(focused.name) + '分项耗时'">
                          <thead><tr><th scope="col">动作</th><th scope="col">A 累计 / 覆盖</th><th v-if="comparing" scope="col">B 累计 / 覆盖</th><th v-if="comparing" scope="col">覆盖 B − A</th></tr></thead>
                          <tbody><tr v-for="activity in focused.activities" :key="activity.name">
                            <th scope="row">{{ activityName(activity.name) }}</th>
                            <td>{{ activity.times[0] ? seconds(activity.times[0].sum) + ' / ' + seconds(activity.times[0].coverage) : '缺数据' }}</td>
                            <td v-if="comparing">{{ activity.times[1] ? seconds(activity.times[1].sum) + ' / ' + seconds(activity.times[1].coverage) : '缺数据' }}</td>
                            <td v-if="comparing">{{ delta(activity.times[0]?.coverage, activity.times[1]?.coverage) }}</td>
                          </tr></tbody>
                        </table>
            </div>
          </section>
          <RunActionStats :runs="details" />
          <section v-for="(run, index) in details" :key="run.id" class="run-detail">
            <h3>{{ index ? 'B' : 'A' }} · 作业与任务事件</h3>
            <p>作业累计 {{ seconds(run.job_sum_ms) }}，实际覆盖 {{ seconds(run.job_wall_ms) }}</p>
            <ul v-if="run.missing.length" class="warning"><li v-for="message in run.missing" :key="message">{{ message }}</li></ul>
            <details><summary>查看 {{ run.frames.length }} 条原始任务事件</summary><pre>{{ run.frames.join('\n') }}</pre></details>
            <details v-if="run.logs?.length"><summary>查看 {{ run.logs.length }} 条日志原文</summary><pre>{{ run.logs.map(entry => `${entry.time} [${entry.direction}] ${entry.text}`).join('\n') }}</pre></details>
          </section>
        </template>
        <p v-else-if="first && !error" role="status">正在读取场次详情</p>
      </section>
    </template>
  </main>
</template>
