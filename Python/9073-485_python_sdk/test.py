import device_model
import time


def updateData(DeviceModel):
    print(DeviceModel.deviceName + " : " + DeviceModel.deviceData)


if __name__ == "__main__":
    # 拿到设备模型 Get the device model
    device = device_model.DeviceModel("测试设备", "COM51", 115200, 0x50, updateData)
    # 开启设备 Turn on the device
    device.openDevice()
    # 开启轮询 Enable loop reading
    device.startLoopRead()

    # 读取寄存器 从0x3a读取1个寄存器 Read Register Read 1 register from 0x3a
    # device.readReg(0x3a, 1)
    # 获得读取结果 Obtaining read results
    # device.get(str(0x3a))

    # 写入寄存器 向0x65写入50 即修改检测周期为50hz Write a register to 0x65 and write 50 to modify the detection cycle to 50Hz
    # device.writeReg(0x65, 50)
