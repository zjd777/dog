# STM32H723：Makefile + GCC + ST-Link

在工程根目录运行（Windows PowerShell 或 VS Code 终端）：

```powershell
make -j8
make flash
```

`make flash` 会先编译，成功后通过 OpenOCD/ST-Link 写入、校验并复位。

- `make clean`：只清理 `build` 目录。
- `make rebuild`：清理后完整编译。
- `make probe`：检测 ST-Link 和目标芯片。
- Ctrl+Shift+B：使用 Make 编译整个工程。
- F5：通过 Cortex-Debug/OpenOCD 下载并调试，停在 `main`。
- 只下载：运行 VS Code 任务 `STM32: Download (ST-Link)`。

输出文件为 `build/CtrBoard-H7_IMU.elf`、`.hex`、`.bin` 和 `.map`。
默认使用 `-Og` 并保留调试信息。所有工程源码均重新使用 GCC 编译，不混用 Keil 对象文件。

工具已安装：GNU Make 4.4.1、Arm GNU Toolchain 14.3.Rel1、OpenOCD 0.12.0。
VS Code 脚本会配置本机工具路径；终端直接调用 make 时使用系统 PATH。

构建支持文件位于 `build_support`：GNU 启动文件保留原工程的中断向量顺序；链接配置为 1MiB Flash、128KiB DTCM、320KiB AXI SRAM。
现有 `.AXI_SRAM` DMA 缓冲区随初始化数据复制至 AXI SRAM，C++ 全局构造函数在进入 main 前执行。

当前工程不调用的 `bsp.c`、`bsp_key.c`、`bsp_beep.c` 不参与编译，原文件保留。
经确认允许的源码兼容改动：注释未启用 UART1/7/10 的引用；使用标准整数类型；调整一个初始化列表的字段顺序。
原文件与配置备份位于 `.codex-backup/environment-repair-20261031-155025`。

ST-Link 使用 SWD，需连接 GND、SWDIO、SWCLK，目标板需供电；接好 NRST 可便于复位。
