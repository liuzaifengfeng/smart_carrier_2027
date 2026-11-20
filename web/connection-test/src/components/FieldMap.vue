<script setup lang="ts">
import { onMounted, ref, watch } from 'vue'

const props = defineProps<{ pose: number[]; target: number[]; nodes: number[][];
  preview?: number[][]; route?: number[]; routeActive?: boolean }>()
const emit = defineEmits<{ target: [x: number, y: number] }>()
const canvas = ref<HTMLCanvasElement>()
const width = 760
const height = 647
const left = 132
const top = 86
const side = 487
const bottom = top + side
const scale = side / 2400
const x = (mm: number) => left + mm * scale
const y = (mm: number) => bottom - mm * scale
const radians = (degrees: number) => degrees * Math.PI / 180
const font = '"Microsoft YaHei UI", "Microsoft YaHei", sans-serif'
let pointer: [number, number] | undefined

function draw() {
  const context = canvas.value?.getContext('2d')
  if (!context) return
  const ctx = context
  const line = (ax: number, ay: number, bx: number, by: number, color: string, weight = 1) => {
    ctx.strokeStyle = color; ctx.lineWidth = weight
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke()
  }
  const label = (text: string, px: number, py: number, size: number, color: string, bold = false, align: CanvasTextAlign = 'center') => {
    ctx.fillStyle = color; ctx.font = (bold ? 'bold ' : '') + size + 'px ' + font
    ctx.textAlign = align; ctx.textBaseline = 'middle'; ctx.fillText(text, px, py)
  }
  const circle = (px: number, py: number, radius: number, outline: string, weight: number, fill?: string) => {
    ctx.beginPath(); ctx.arc(px, py, radius, 0, Math.PI * 2)
    if (fill) { ctx.fillStyle = fill; ctx.fill() }
    ctx.strokeStyle = outline; ctx.lineWidth = weight; ctx.stroke()
  }
  const arrow = (ax: number, ay: number, bx: number, by: number, color: string, weight: number, size: number) => {
    line(ax, ay, bx, by, color, weight)
    const angle = Math.atan2(by - ay, bx - ax)
    ctx.save(); ctx.translate(bx, by); ctx.rotate(angle)
    ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(0, 0)
    ctx.lineTo(-size, -size * .45); ctx.lineTo(-size, size * .45); ctx.closePath(); ctx.fill()
    ctx.restore()
  }
  const zone = (x0: number, x1: number, y0: number, y1: number, fill: string, outline?: string) => {
    const box = [x(x0), y(y1), (x1 - x0) * scale, (y1 - y0) * scale] as const
    ctx.fillStyle = fill; ctx.fillRect(...box)
    if (outline) { ctx.strokeStyle = outline; ctx.lineWidth = 1; ctx.strokeRect(...box) }
  }

  ctx.fillStyle = '#eef1f4'; ctx.fillRect(0, 0, width, height)
  zone(0, 2400, 0, 2400, '#d8dadd')
  for (const a of [550, 1400]) for (const b of [550, 1400]) zone(a, a + 450, b, b + 450, '#fffde2')
  for (let mm = 480; mm < 2400; mm += 480) {
    line(x(mm), top, x(mm), bottom, '#aeb4ba')
    line(left, y(mm), x(2400), y(mm), '#aeb4ba')
  }
  ctx.setLineDash([8, 8])
  line(x(1200), top, x(1200), bottom, '#747b82')
  line(left, y(1200), x(2400), y(1200), '#747b82')
  ctx.setLineDash([])
  ctx.strokeStyle = '#384047'; ctx.lineWidth = 2; ctx.strokeRect(left, top, side, side)
  zone(0, 300, 0, 300, '#1859d1')
  zone(2100, 2400, 0, 300, '#1859d1')
  zone(910, 1490, 2250, 2400, '#cfd2d5', '#707780')
  zone(0, 150, 910, 1490, '#cfd2d5', '#707780')

  for (const [id, px, py] of props.nodes) {
    circle(x(px!), y(py!), 105 * scale, '#aeb4ba', 2)
    label(String(id), x(px!), y(py!), 22, '#9ca3aa', true)
  }
  function ring(px: number, py: number) {
    circle(x(px), y(py), 38 * scale, '#111', 2, '#fff')
    circle(x(px), y(py), 38 * .42 * scale, '#60666d', 1)
  }
  for (const mm of [1010, 1200, 1390]) { ring(mm, 2325); ring(75, mm) }
  circle(x(2400), y(1200), 150 * scale, '#60666d', 2, '#f4f4f4')
  line(x(1100), bottom, x(1300), bottom, '#1f252a', 6)
  for (const [px, py, name] of [
    [2250, 150, '启停区1'], [150, 150, '启停区2'], [2350, 1500, '原料区'],
    [1200, 2180, '暂存区'], [210, 1100, '粗加工区'], [1200, 80, '二维码板'],
  ] as const) label(name, x(px), y(py), 13, '#111', true)

  for (let mm = 0; mm <= 2400; mm += 300) {
    const major = mm % 600 === 0
    const tick = major ? 8 : 5
    line(x(mm), bottom, x(mm), bottom + tick, '#384047')
    line(left, y(mm), left - tick, y(mm), '#384047')
    if (major) {
      label(String(mm), x(mm), bottom + 18, 10, '#343a40')
      label(String(mm), left - 12, y(mm), 10, '#343a40', false, 'right')
    }
  }
  label('场地总尺寸：2400 × 2400 mm    小车：300 × 300 mm', left, top - 28, 13, '#343a40', true, 'left')
  label('X 坐标 / mm（向右增大）', x(1200), bottom + 41, 12, '#343a40')
  ctx.save(); ctx.translate(left - 54, y(1200)); ctx.rotate(-Math.PI / 2)
  label('Y 坐标 / mm（向上增大）', 0, 0, 12, '#343a40'); ctx.restore()

  if (props.preview?.length) {
    ctx.beginPath()
    props.preview.forEach((point, i) => {
      if (i) ctx.lineTo(x(point[1]!), y(point[2]!))
      else ctx.moveTo(x(point[1]!), y(point[2]!))
    })
    ctx.strokeStyle = '#2563eb'; ctx.lineWidth = 2; ctx.stroke()
  }

  const route = props.route?.map(id => props.nodes.find(node => node[0] === id))
  if (route?.length && route.every(point => point !== undefined)) {
    const color = props.routeActive ? '#087e72' : '#537c76'
    ctx.beginPath()
    route.forEach((point, index) => {
      const px = x(point![1]!), py = y(point![2]!)
      if (index === 0) ctx.moveTo(px, py)
      else ctx.lineTo(px, py)
    })
    ctx.strokeStyle = color; ctx.lineWidth = 4
    if (!props.routeActive) ctx.setLineDash([7, 5])
    ctx.stroke(); ctx.setLineDash([])
    for (const point of route) {
      const [id, px, py] = point!
      circle(x(px!), y(py!), 11, color, 2, '#eef8f5')
      label(String(id), x(px!), y(py!), 11, '#164b45', true)
    }
  }

  function robot(pose: number[], target = false) {
    const [px, py, heading, high = 0, length = 0, turret = 0, pawl = 0] = pose as [number, number, number, ...number[]]
    const color = target ? '#e67e22' : '#145a86'
    ctx.save(); ctx.translate(x(px), y(py)); ctx.rotate(-radians(heading))
    const half = 150 * scale
    if (!target) {
      ctx.fillStyle = '#4f9dd9'; ctx.fillRect(-half, -half, half * 2, half * 2)
    }
    ctx.strokeStyle = color; ctx.lineWidth = 3
    if (target) ctx.setLineDash([7, 4])
    ctx.strokeRect(-half, -half, half * 2, half * 2)
    ctx.setLineDash([])
    if (!target) {
      ctx.fillStyle = '#111'
      for (const f of [-82.5, 82.5]) for (const l of [-150, 150]) {
        ctx.fillRect((f - 38) * scale, (l - 13) * scale, 76 * scale, 26 * scale)
      }
    }
    const lane = target ? 0 : -58 * scale
    arrow(target ? 0 : -52 * scale, lane, (target ? 204 : 102) * scale, lane, target ? color : '#111', 4, 13)
    ctx.save(); ctx.translate(target ? 0 : -62 * scale, 0); ctx.rotate(radians(heading))
    label(target ? '目标' : '小车', 0, 0, 12, target ? color : '#fff', true)
    ctx.restore()
    if (!target && pose.length >= 7) {
      ctx.translate(20 * scale, 135 * scale)
      const heightArc = (span: number, stroke: string) => {
        ctx.beginPath(); ctx.arc(0, 0, 84 * scale, Math.PI - span, Math.PI + span)
        ctx.strokeStyle = stroke; ctx.lineWidth = 27 * scale; ctx.lineCap = 'round'; ctx.stroke()
      }
      heightArc(radians(64), '#747a80')
      if (high > 0) heightArc(radians(64 * Math.min(high / 175, 1)), '#f5a45d')
      circle(0, 0, 43 * scale, '#ef2525', 4)
      ctx.rotate(radians(turret))
      const travel = Math.max(0, Math.min(length / 170, 1)) * 65
      ctx.fillStyle = '#ffe000'; ctx.fillRect((travel - 140) * scale, -17 * scale, 280 * scale, 34 * scale)
      ctx.translate((travel + 140) * scale, 0)
      for (const sign of [-1, 1]) {
        ctx.save(); ctx.rotate(-sign * radians(Math.max(-80, Math.min(pawl, 80))))
        ctx.beginPath(); ctx.arc(35 * scale, 0, 35 * scale, sign < 0 ? 0 : Math.PI, sign < 0 ? Math.PI : Math.PI * 2)
        ctx.strokeStyle = '#111'; ctx.lineWidth = 4; ctx.stroke(); ctx.restore()
      }
    }
    ctx.restore()
  }

  const axis = side * .16
  arrow(left, bottom, left + axis, bottom, '#d9342b', 2, 8)
  arrow(left, bottom, left, bottom - axis, '#d9342b', 2, 8)
  label('+X', left + axis, bottom - 14, 12, '#d9342b')
  label('+Y', left + 18, bottom - axis, 12, '#d9342b')
  if (props.target.length >= 3) robot(props.target, true)
  if (props.pose.length >= 3) robot(props.pose)

  if (pointer) {
    const [px, py] = pointer
    ctx.setLineDash([6, 4])
    line(x(px), top, x(px), bottom, '#d9342b')
    line(left, y(py), x(2400), y(py), '#d9342b')
    ctx.setLineDash([])
    const node = nodeAt(px, py)
    const boxX = x(px) + (px > 1450 ? -192 : 10)
    const boxY = y(py) + (py < 270 ? -48 : 10)
    ctx.fillStyle = '#fffdf2'; ctx.fillRect(boxX, boxY, 182, node ? 38 : 22)
    ctx.strokeStyle = '#a76b24'; ctx.lineWidth = 1; ctx.strokeRect(boxX, boxY, 182, node ? 38 : 22)
    label('X ' + Math.round(px) + ' mm   Y ' + Math.round(py) + ' mm', boxX + 6, boxY + 11, 12, '#17232d', true, 'left')
    if (node) label('节点 ' + node[0] + ' 中心：' + node[1] + ' / ' + node[2], boxX + 6, boxY + 27, 12, '#17232d', true, 'left')
  }
}

function nodeAt(x: number, y: number) {
  return props.nodes.find(node => Math.hypot(node[1]! - x, node[2]! - y) <= 105)
}
function position(event: MouseEvent): [number, number] | undefined {
  const rect = canvas.value!.getBoundingClientRect()
  const px = ((event.clientX - rect.left) / rect.width * width - left) / scale
  const py = (bottom - (event.clientY - rect.top) / rect.height * height) / scale
  if (px >= 0 && px <= 2400 && py >= 0 && py <= 2400) return [px, py]
}
function hover(event?: PointerEvent) {
  pointer = event?.pointerType === 'mouse' ? position(event) : undefined
  draw()
}
function pick(event: MouseEvent) {
  const point = position(event)
  if (!point) return
  const [px, py] = point
  const node = nodeAt(px, py)
  emit('target', node?.[1] ?? Math.round(px), node?.[2] ?? Math.round(py))
}
onMounted(draw)
watch(() => [props.pose, props.target, props.nodes, props.preview, props.route, props.routeActive], draw, { deep: true })
</script>

<template>
  <canvas ref="canvas" :width="width" :height="height" aria-label="2400 毫米场地图。点击选取目标，可在姿态页输入坐标"
    @pointermove="hover" @pointerleave="hover()" @click="pick" />
</template>

<style scoped>
canvas { display: block; width: 100%; height: auto; }
</style>

