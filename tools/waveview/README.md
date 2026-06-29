# NS800 WaveView

轻量串口实时波形上位机。页面左侧 1/4 显示三组 SVM 合成矢量，右侧 3/4 显示实时滚动波形。

右侧波形区域兼容原来的逗号分隔整数行：

```c
rt_kprintf("%d,%d,%d,%d\r\n", a, b, c, d);
```

也可以显式加 `wave:` 前缀：

```c
rt_kprintf("wave:%d,%d,%d,%d\r\n", a, b, c, d);
```

左侧 SVM 矢量区域使用 `svm:<target>,<angle_deg>,<length>` 格式，`target` 可选 `total`、`upper`、`lower`。第三个字段就是矢量长度，页面会把它同时作为圆半径和 `L=...` 长度标注显示：

```c
rt_kprintf("svm:total,%d,%d\r\n", total_angle_deg, total_len);
rt_kprintf("svm:upper,%d,%d\r\n", upper_angle_deg, upper_len);
rt_kprintf("svm:lower,%d,%d\r\n", lower_angle_deg, lower_len);
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
