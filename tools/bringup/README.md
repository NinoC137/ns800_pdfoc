# 自动化联调 CLI

工程提供 `tools/bringup/nsbringup.py`，用于把编译、烧录、串口日志、FinSH 命令
和 GDB 快照串成可重复的 bring-up 流程。

安装串口依赖：

```sh
.venv/bin/pip install -r requirements-bringup.txt
```

常用命令：

```sh
python3 tools/bringup/nsbringup.py list-serial
python3 tools/bringup/nsbringup.py build --jobs 4
python3 tools/bringup/nsbringup.py flash
python3 tools/bringup/nsbringup.py serial-log --port /dev/cu.xxx
python3 tools/bringup/nsbringup.py msh --port /dev/cu.xxx --cmd adc_sample --cmd adc_regs
python3 tools/bringup/nsbringup.py gdb-snapshot --mem 0x40030000:16 --mem 0x40032000:16
python3 tools/bringup/nsbringup.py smoke --port /dev/cu.xxx
```

每次运行都会在 `build/bringup/YYYYMMDD-HHMMSS/` 下生成日志和 `summary.md`。
