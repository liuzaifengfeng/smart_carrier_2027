export interface Link {
  send(bytes: Uint8Array<ArrayBuffer>): Promise<void>
  close(): Promise<void>
}

export type Receive = (bytes: Uint8Array) => void
export type Disconnected = (error?: unknown) => void

export async function openUsbSerial(baudRate: number, receive: Receive, disconnected: Disconnected) {
  const device = await navigator.usb.requestDevice({ filters: [] })
  const info = describeUsb(device)
  const configuration = device.configurations.find(config =>
    config.interfaces.some(item => item.alternates.some(alt => alt.interfaceClass === 2 && alt.interfaceSubclass === 2))
    && config.interfaces.some(item => item.alternates.some(alt => alt.interfaceClass === 10)),
  )
  if (!configuration) throw new Error('设备没有 CDC ACM 串口接口。')
  const control = configuration.interfaces.find(item =>
    item.alternates.some(alt => alt.interfaceClass === 2 && alt.interfaceSubclass === 2),
  )!
  const data = configuration.interfaces.find(item =>
    item.alternates.some(alt => alt.interfaceClass === 10
      && alt.endpoints.some(endpoint => endpoint.type === 'bulk' && endpoint.direction === 'in')
      && alt.endpoints.some(endpoint => endpoint.type === 'bulk' && endpoint.direction === 'out')),
  )
  if (!data) throw new Error('未找到 CDC 数据收发端点。')
  const alternate = data.alternates.find(alt => alt.interfaceClass === 10
    && alt.endpoints.some(endpoint => endpoint.type === 'bulk' && endpoint.direction === 'in')
    && alt.endpoints.some(endpoint => endpoint.type === 'bulk' && endpoint.direction === 'out'))!
  const input = alternate.endpoints.find(endpoint => endpoint.type === 'bulk' && endpoint.direction === 'in')!
  const output = alternate.endpoints.find(endpoint => endpoint.type === 'bulk' && endpoint.direction === 'out')!
  try {
    await device.open()
    if (!device.configuration || device.configuration.configurationValue !== configuration.configurationValue) {
      await device.selectConfiguration(configuration.configurationValue)
    }
    await device.claimInterface(control.interfaceNumber)
    await device.claimInterface(data.interfaceNumber)
    if (data.alternate.alternateSetting !== alternate.alternateSetting) {
      await device.selectAlternateInterface(data.interfaceNumber, alternate.alternateSetting)
    }
    const coding = new ArrayBuffer(7)
    const view = new DataView(coding)
    view.setUint32(0, baudRate, true)
    view.setUint8(6, 8)
    const setup: USBControlTransferParameters = {
      requestType: 'class', recipient: 'interface', request: 0x20,
      value: 0, index: control.interfaceNumber,
    }
    const lineResult = await device.controlTransferOut(setup, coding)
    if (lineResult.status !== 'ok') throw new Error('设置串口参数失败：' + lineResult.status)
    const signalResult = await device.controlTransferOut({ ...setup, request: 0x22, value: 1 })
    if (signalResult.status !== 'ok') throw new Error('设置串口信号失败：' + signalResult.status)
  } catch (error) {
    if (device.opened) await device.close()
    throw error
  }
  let closing = false
  const onDisconnect = (event: USBConnectionEvent) => {
    if (event.device === device && !closing) disconnected()
  }
  navigator.usb.addEventListener('disconnect', onDisconnect)
  const read = async () => {
    try {
      while (!closing) {
        const result = await device.transferIn(input.endpointNumber, input.packetSize)
        if (result.status !== 'ok') throw new Error('USB 接收失败：' + result.status)
        if (result.data?.byteLength) {
          receive(new Uint8Array(result.data.buffer, result.data.byteOffset, result.data.byteLength))
        }
      }
    } catch (error) {
      if (!closing) disconnected(error)
    }
  }
  void read()
  const link: Link = {
    async send(bytes) {
      const result = await device.transferOut(output.endpointNumber, bytes)
      if (result.status !== 'ok' || result.bytesWritten !== bytes.byteLength) {
        throw new Error('USB 发送未完成：' + result.status)
      }
    },
    async close() {
      closing = true
      navigator.usb.removeEventListener('disconnect', onDisconnect)
      if (device.opened) await device.close()
    },
  }
  return { link, info: { ...info, baudRate, framing: '8N1' } }
}

export async function openSerial(baudRate: number, receive: Receive, disconnected: Disconnected) {
  const port = await navigator.serial.requestPort()
  await port.open({ baudRate })
  const reader = port.readable!.getReader()
  const writer = port.writable!.getWriter()
  let closing = false
  const read = async () => {
    try {
      while (true) {
        const { value, done } = await reader.read()
        if (done) break
        receive(value)
      }
      if (!closing) disconnected()
    } catch (error) {
      if (!closing) disconnected(error)
    } finally {
      reader.releaseLock()
    }
  }
  const reading = read()
  const link: Link = {
    send: bytes => writer.write(bytes),
    async close() {
      closing = true
      await Promise.allSettled([reader.cancel(), reading])
      writer.releaseLock()
      await port.close()
    },
  }
  return { link, info: port.getInfo() }
}

export function parseServices(input: string): BluetoothServiceUUID[] {
  return input.split(/[\s,，]+/).filter(Boolean).map(value =>
    /^(0x)?[0-9a-f]{4}$/i.test(value) ? Number.parseInt(value, 16) : value,
  )
}

export async function openBluetooth(services: BluetoothServiceUUID[], disconnected: Disconnected) {
  const device = await navigator.bluetooth.requestDevice({
    acceptAllDevices: true,
    optionalServices: services,
  })
  if (!device.gatt) throw new Error('此设备未提供 GATT。')
  const server = await device.gatt.connect()
  const onDisconnect = () => disconnected()
  device.addEventListener('gattserverdisconnected', onDisconnect, { once: true })
  return {
    device,
    server,
    async close() {
      device.removeEventListener('gattserverdisconnected', onDisconnect)
      server.disconnect()
    },
  }
}

export async function inspectGatt(server: BluetoothRemoteGATTServer, requested: BluetoothServiceUUID[]) {
  const services = requested.length ? await server.getPrimaryServices() : []
  return Promise.all(services.map(async service => ({
    uuid: service.uuid,
    characteristics: await service.getCharacteristics(),
  })))
}

export async function subscribe(characteristic: BluetoothRemoteGATTCharacteristic, receive: Receive) {
  const onValue = () => {
    const value = characteristic.value!
    receive(new Uint8Array(value.buffer, value.byteOffset, value.byteLength))
  }
  characteristic.addEventListener('characteristicvaluechanged', onValue)
  try {
    await characteristic.startNotifications()
  } catch (error) {
    characteristic.removeEventListener('characteristicvaluechanged', onValue)
    throw error
  }
  return async () => {
    characteristic.removeEventListener('characteristicvaluechanged', onValue)
    if (characteristic.service.device.gatt?.connected) await characteristic.stopNotifications()
  }
}

export function sendBluetooth(characteristic: BluetoothRemoteGATTCharacteristic, bytes: Uint8Array<ArrayBuffer>) {
  return characteristic.properties.write
    ? characteristic.writeValueWithResponse(bytes)
    : characteristic.writeValueWithoutResponse(bytes)
}

export function describeUsb(device: USBDevice) {
  return {
    product: device.productName,
    manufacturer: device.manufacturerName,
    vid: device.vendorId.toString(16).padStart(4, '0'),
    pid: device.productId.toString(16).padStart(4, '0'),
    configurations: device.configurations.map(configuration => ({
      value: configuration.configurationValue,
      interfaces: configuration.interfaces.map(item => ({
        number: item.interfaceNumber,
        alternates: item.alternates.map(alternate => ({
          setting: alternate.alternateSetting,
          class: alternate.interfaceClass,
          subclass: alternate.interfaceSubclass,
          protocol: alternate.interfaceProtocol,
          endpoints: alternate.endpoints.map(endpoint => ({
            number: endpoint.endpointNumber,
            direction: endpoint.direction,
            type: endpoint.type,
            packetSize: endpoint.packetSize,
          })),
        })),
      })),
    })),
  }
}
