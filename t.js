// 创建 ArrayBuffer
function createCommand(cmd, value) {
  const buffer = new ArrayBuffer(7);
  const view = new Uint8Array(buffer);
  view[0] = 0xA5;   // 帧头
  view[1] = cmd;     // 命令
  view[2] = value;   // 数值
  view[3] = 0;
  view[4] = 0;
  view[5] = 0;
  view[6] = 0x5A;   // 帧尾
  return buffer;
}

// 发送起立命令
wx.writeBLECharacteristicValue({
  deviceId: deviceId,
  serviceId: 'ba0d1b7e-7ad8-11ef-b864-0242ac120002',
  characteristicId: 'c7ebcf24-7ad8-11ef-b864-0242ac120002',  // RX
  value: createCommand(0x06, 0),
  success: function(res) {
    console.log('发送成功');
  }
})