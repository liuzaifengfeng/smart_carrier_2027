<script setup lang="ts">
import { computed, onMounted, ref, watch } from 'vue'
import { firmwareFieldMap } from '../fieldMapDefaults'

const props = defineProps<{ pose: number[]; target: number[]; nodes: number[][];
  preview?: number[][]; route?: number[]; routeActive?: boolean }>()
const emit = defineEmits<{ target: [x: number, y: number] }>()
const canvas = ref<HTMLCanvasElement>()
const width = 760
const height = 647
const pixelRatio = window.devicePixelRatio
const left = 132
const top = 86
const side = 487
const bottom = top + side
const { fieldSize, bodyWidth } = firmwareFieldMap
const scale = side / fieldSize
const center = fieldSize / 2
const x = (mm: number) => left + mm * scale
const y = (mm: number) => bottom - mm * scale
const radians = (degrees: number) => degrees * Math.PI / 180
const font = '"Microsoft YaHei UI", "Microsoft YaHei", sans-serif'
let pointer: [number, number] | undefined
const corridorLayer = document.createElement('canvas')
corridorLayer.width = Math.round(width * pixelRatio)
corridorLayer.height = Math.round(height * pixelRatio)
const corridorContext = corridorLayer.getContext('2d')!
corridorContext.scale(pixelRatio, pixelRatio)

function addPolyline(path: Path2D, points: number[][]) {
  for (const [index, point] of points.entries()) {
    const px = x(point[1]!), py = y(point[2]!)
    if (index === 0) path.moveTo(px, py)
    else path.lineTo(px, py)
  }
}

const roadNetwork = computed(() => {
  const nodes = new Map(props.nodes.map(node => [node[0], node]))
  const nodeAtGrid = (row: number, column: number) => nodes.get(row * 5 + (row % 2 === 0 ? column : 4 - column))!
  const indices = [0, 1, 2, 3, 4]
  const path = new Path2D()
  // The outer loop and the middle cross are the field roads.
  for (const lane of [0, 2, 4]) {
    addPolyline(path, indices.map(column => nodeAtGrid(lane, column)))
    addPolyline(path, indices.map(row => nodeAtGrid(row, lane)))
  }
  return path
})

function draw() {
  const context = canvas.value?.getContext('2d')
  if (!context) return
  const ctx = context
  ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0)
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
  const corridor = (path: Path2D, centerColor: string, edgeColor: string, fillColor: string,
    lineWidth: number, dash: number[] = []) => {
    // Draw the outline on its own layer so clearing its interior preserves the field underneath.
    const layer = corridorContext
    const bandWidth = bodyWidth * scale
    layer.clearRect(0, 0, width, height)
    layer.lineJoin = 'round'
    layer.lineCap = 'round'
    layer.lineWidth = bandWidth
    layer.strokeStyle = edgeColor
    layer.stroke(path)
    layer.globalCompositeOperation = 'destination-out'
    layer.lineWidth = bandWidth - 2.5
    layer.strokeStyle = '#000'
    layer.stroke(path)
    layer.globalCompositeOperation = 'destination-over'
    layer.lineWidth = bandWidth
    layer.strokeStyle = fillColor
    layer.stroke(path)
    layer.globalCompositeOperation = 'source-over'
    ctx.drawImage(corridorLayer, 0, 0, width, height)

    ctx.save()
    ctx.lineJoin = 'round'
    ctx.strokeStyle = centerColor
    ctx.lineWidth = lineWidth
    ctx.setLineDash(dash)
    ctx.stroke(path)
    ctx.restore()
  }

  ctx.fillStyle = '#eef1f4'; ctx.fillRect(0, 0, width, height)
  zone(0, fieldSize, 0, fieldSize, '#d8dadd')
  for (const a of [550, 1400]) for (const b of [550, 1400]) zone(a, a + 450, b, b + 450, '#fffde2')
  for (let mm = 480; mm < fieldSize; mm += 480) {
    line(x(mm), top, x(mm), bottom, '#aeb4ba')
    line(left, y(mm), x(fieldSize), y(mm), '#aeb4ba')
  }
  ctx.strokeStyle = '#384047'; ctx.lineWidth = 2; ctx.strokeRect(left, top, side, side)
  zone(0, 300, 0, 300, '#1859d1')
  zone(2100, 2400, 0, 300, '#1859d1')
  zone(910, 1490, 2250, 2400, '#cfd2d5', '#707780')
  zone(0, 150, 910, 1490, '#cfd2d5', '#707780')

  function ring(px: number, py: number) {
    circle(x(px), y(py), 38 * scale, '#111', 2, '#fff')
    circle(x(px), y(py), 38 * .42 * scale, '#60666d', 1)
  }
  for (const mm of [1010, 1200, 1390]) { ring(mm, 2325); ring(75, mm) }
  circle(x(2400), y(1200), 150 * scale, '#60666d', 2, '#f4f4f4')
  line(x(1100), bottom, x(1300), bottom, '#1f252a', 6)
  for (let mm = 0; mm <= fieldSize; mm += 300) {
    const major = mm % 600 === 0
    const tick = major ? 8 : 5
    line(x(mm), bottom, x(mm), bottom + tick, '#384047')
    line(left, y(mm), left - tick, y(mm), '#384047')
    if (major) {
      label(String(mm), x(mm), bottom + 18, 10, '#343a40')
      label(String(mm), left - 12, y(mm), 10, '#343a40', false, 'right')
    }
  }
  label(`场地 ${fieldSize} × ${fieldSize} mm    车宽 ${bodyWidth} mm`, left, top - 28, 13, '#343a40', true, 'left')
  label('X 坐标 / mm（向右增大）', x(center), bottom + 41, 12, '#343a40')
  ctx.save(); ctx.translate(left - 54, y(center)); ctx.rotate(-Math.PI / 2)
  label('Y 坐标 / mm（向上增大）', 0, 0, 12, '#343a40'); ctx.restore()

  corridor(roadNetwork.value, '#677684', '#7e8e9c', '#aabacb57', 1, [4, 4])
  if (props.preview && props.preview.length >= 2) {
    const path = new Path2D()
    addPolyline(path, props.preview)
    corridor(path, '#2563eb', '#507bb6', '#2563eb26', 2)
  }
  const route = props.route?.map(id => props.nodes.find(node => node[0] === id)) ?? []
  const routeColor = props.routeActive ? '#087e72' : '#537c76'
  if (route.length >= 2 && route.every(point => point !== undefined)) {
    const path = new Path2D()
    addPolyline(path, route)
    corridor(path, routeColor, '#418277', '#0c8e772c', 3, props.routeActive ? [] : [7, 5])
  }
  for (const [px, py, name] of [
    [2250, 150, '启停区1'], [150, 150, '启停区2'], [2350, 1500, '原料区'],
    [1200, 2180, '粗加工区'], [210, 1100, '暂存区'], [1200, 80, '二维码板'],
  ] as const) label(name, x(px), y(py), 13, '#111', true)
  for (const [id, px, py] of props.nodes) {
    const onRoute = props.route?.includes(id!)
    circle(x(px!), y(py!), 11, onRoute ? routeColor : '#727e86', onRoute ? 2 : 1.2,
      onRoute ? '#eef8f5' : '#f4f6f7')
    label(String(id), x(px!), y(py!), 11, onRoute ? '#164b45' : '#384047', true)
  }

  function robot(pose: number[], target = false) {
    const [px, py, heading, high = 0, length = 0, turret = 0, pawl = 0] = pose as [number, number, number, ...number[]]
    const color = target ? '#e67e22' : '#145a86'
    ctx.save(); ctx.translate(x(px), y(py)); ctx.rotate(-radians(heading))
    const half = bodyWidth * scale / 2
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
    line(left, y(py), x(fieldSize), y(py), '#d9342b')
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
  if (px >= 0 && px <= fieldSize && py >= 0 && py <= fieldSize) return [px, py]
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
  <canvas ref="canvas" :width="Math.round(width * pixelRatio)" :height="Math.round(height * pixelRatio)"
    :aria-label="`${fieldSize} 毫米场地图。点击选取目标，可在姿态页输入坐标`"
    @pointermove="hover" @pointerleave="hover()" @click="pick" />
</template>

<style scoped>
canvas { display: block; width: 100%; height: auto; }
</style>

