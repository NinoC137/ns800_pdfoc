# NS800 WaveView

轻量串口实时波形上位机。MCU 输出逗号分隔整数行，例如：

```c
rt_kprintf("%d,%d,%d,%d\r\n", a, b, c, d);
```

启动：

```sh
.venv/bin/python tools/waveview/nswaveview.py --port /dev/cu.usbmodemXXXX --baud 115200 --channels 4
```

无 MCU 演示：

```sh
.venv/bin/python tools/waveview/nswaveview.py --demo --channels 4
```

启动后浏览器打开本地页面，Canvas 实时显示滚动波形。页面支持暂停、清空、自动缩放和固定 Y 轴范围。
