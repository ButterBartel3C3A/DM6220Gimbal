# 达妙 USB-CAN 后端动态库（仅 USB2CANFD 备选）

> **当前主路径是串口型 USB2CAN（DM-USBCU1 一类，COM + 921600 + `55 AA` 帧），不需要本目录的 DLL。**
> 只有确认实物是 USB2CANFD_S/D 或 LinkX4C 时，才使用 `dmcan-sdk` 和这里的 `libdmcan.dll`。
> 串口型协议见 [`docs/usb2can-serial.md`](../../docs/usb2can-serial.md)。

本目录用于放置 `dmcan-sdk` 所需的 `libdmcan` 动态库及其依赖。二进制文件不提交到 Git（见 `.gitignore`）。

## 版本匹配

| Python 包 | 后端库 |
|---|---|
| `dmcan-sdk 2.1.1` | `libdmcan 2.1.0` |

`DmCanContext()` 构造时会校验主版本一致、次版本 ≥ 2.1；不匹配会直接报错。

## Windows 必需文件

以下 5 个 DLL 必须放在**同一目录**：

- `libdmcan.dll`
- `libusb-1.0.dll`
- `libstdc++-6.dll`
- `libgcc_s_seh-1.dll`
- `libwinpthread-1.dll`

来源：DM_DeviceSDK 仓库 `Python/demo/dlls/`：

<https://gitee.com/kit-miao/dm-device-sdk/tree/master/Python/demo/dlls>

## 放置与加载

方式一（推荐）：把 DLL 放在本目录，启动程序时设置环境变量：

```powershell
$env:DMCAN_LIBRARY_PATH = "C:\Users\szsm\Documents\Github\DM6220Gimbal\vendor\dmcan\libdmcan.dll"
```

方式二：复制到 Python 包的 `dmcan/dlls/` 目录：

```powershell
python -c "import dmcan, pathlib; print(pathlib.Path(dmcan.__file__).parent / 'dlls')"
```

方式三：放在程序工作目录下的 `dlls/`。

## 验证

```python
from dmcan import DmCanContext
with DmCanContext() as ctx:
    print(ctx.sdk_version())
    print(ctx.find_devices())
```

`find_devices() == 0` 时依次检查：

1. DLL 是否齐全、版本是否匹配；
2. Windows 是否已绑定 libusb/WinUSB 驱动；
3. 设备是否被达妙调试助手或其他进程占用；
4. USB 线/供电是否正常。

## 许可

`dmcan-sdk` 与 `libdmcan` 使用 Apache License 2.0。分发时请保留其许可证与版权声明。
