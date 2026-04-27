# robocon-library

欢迎来到大一 Robocon 比赛第二组的库仓库！本项目基于 STM32F407 微控制器，使用 CMake 构建系统和 Modern C++ 开发。

## 📋 项目概述

- **目标设备**: STM32F407VET6
- **构建系统**: CMake + Ninja
- **编程语言**: C++
- **依赖库**: FreeRTOS、STM32 HAL Driver、GDUT 内部库定义

## 🚀 快速开始

### 1. Clone 仓库

使用 `--recursive` 标志递归克隆所有子模块：

```bash
git clone --recursive https://github.com/gdut-robocon11-second-group/robocon-library.git
cd robocon-library
```

如果你已经 clone 过但忘记了 `--recursive` 标志，可以手动初始化子模块：

```bash
git submodule update --init --recursive
```

### 2. 构建项目

本项目使用 CMake 构建系统，请遵循以下步骤：

#### 方法 A: 使用 CMake 命令行（推荐）

```bash
# 创建构建目录（如果不存在）
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

# 构建项目
cmake --build build
```

#### 方法 B: 使用 CMake GUI（可选）

```bash
cmake-gui .
```

**注意**: ❌ 不要直接使用 `make` 命令，我们使用 CMake 的构建抽象层，这样更加跨平台且易于维护。

## 💻 开发环境配置

### 快速启动

要完整配置 VS Code 开发环境，请参考 [VS_Code_Configuration.md](./docs/VS_Code_Configuration.md)，包括：

- **OpenOCD 调试器安装**
  - Windows 下载：https://github.com/openocd-org/openocd/releases
  - 环境变量配置
  - 连接调试器

- **VS Code 调试配置**
  - 使用 `debug/launch.json` 配置文件模板
  - 使用 `debug/settings.json` 编辑器设置模板
  - GDB 调试工作流

- **调试工具使用**
  - 断点设置与管理
  - 单步执行、变量查看
  - 调用栈跟踪

### 最小化快速设置

如果只想快速开始构建：

1. **安装必要工具**
   - ARM GCC Toolchain
   - CMake >= 3.20
   - Ninja 构建工具
   - clang-format（可选，用于代码格式化）

2. **构建项目**
   ```bash
   cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
   cmake --build build
   ```

3. **格式化代码**
   ```bash
   # Windows
   .\format_all.ps1
   
   # Linux/macOS
   ./format_all.sh
   ```

详细配置见 [VS_Code_Configuration.md](./docs/VS_Code_Configuration.md)。

## 📖 开发文档

本项目提供了详细的开发指南，请根据需要查阅：

### 📘 核心文档

| 文档 | 说明 |
|------|------|
| [代码规范.md](./docs/代码规范.md) | **必读** - 详细的代码规范和开发规范，包括命名约定、类型安全、内存管理等 |
| [VS_Code_Configuration.md](./docs/VS_Code_Configuration.md) | **必读** - VS Code 开发环境配置，包括 OpenOCD 调试器安装、调试配置等 |
| [docs/BSP/](./docs/BSP/) | 硬件抽象层文档（UART、SPI、CAN、TIM、GPIO 等） |
| [docs/Async/](./docs/Async/) | 异步与 RTOS 抽象文档（线程、互斥锁、信号量、事件、消息队列） |
| [docs/Components/](./docs/Components/) | 通用组件文档（function、uncopyable、clock、memory_resource、pid_controller、utils_type_traits、verification_algorithm） |
| [docs/Modules/](./docs/Modules/) | 功能模块文档（universal_wheel_kinematics、transfer_protocol） |

### 代码规范快速参考

详见 [代码规范.md](./docs/代码规范.md)，主要要点：

- **命名约定**: 使用蛇形命名法 (snake_case)
  - 变量和函数: `my_variable`、`calculate_sum()`
  - 类和结构体: `network_manager`、`packet_header`
  - 私有成员变量: `m_socket_fd`

- **代码风格**: Modern C++ (C++17+)
  - 使用 STL 容器而非原生数组
  - 优先使用智能指针 (`std::unique_ptr`、`std::shared_ptr`)
  - 避免 C 风格转换，使用 `static_cast` 等

- **文件组织**:
  - 头文件 (.h): `Core/Inc/`
  - 源文件 (.cpp/.c): `Core/Src/`
  - 驱动文件: `Drivers/`
  - 中间件: `Middlewares/`

## 📚 重要概念与库说明

### GDUT_RC_Library - 组内库

本项目使用组内维护的 `GDUT_RC_Library` 作为主要的嵌入式库，提供：

- **硬件抽象层（BSP）**：UART、SPI、CAN、GPIO、TIM 等硬件驱动接口
- **异步抽象层（Async）**：线程、互斥锁、信号量、事件标志、消息队列
- **组件层（Components）**：函数对象、不可拷贝基类、时钟与内存资源等通用组件
- **性能优化**：针对 STM32 的特化实现

详见 `Middlewares/GDUT_RC_Library/`、`docs/BSP/`、`docs/Async/` 和 `docs/Components/`。

### FreeRTOS 实时操作系统

项目集成了 FreeRTOS 用于多任务管理：

```cpp
#include "FreeRTOS.h"
#include "task.h"

void my_task(void* pvParameters) {
    while (1) {
        // 任务代码
        vTaskDelay(pdMS_TO_TICKS(100));  // 延时 100ms
    }
}

// 在 freertos.c 中创建任务
```

**配置文件**：`Core/Inc/FreeRTOSConfig.h`

### STM32 HAL 与 CMSIS

- **STM32 HAL Driver**：提供硬件寄存器抽象
  - 位置：`Drivers/STM32F4xx_HAL_Driver/`
  - 用于访问 GPIO、定时器、UART 等外设

- **CMSIS**：ARM 通用硬件接口
  - 位置：`Drivers/CMSIS/`
  - 提供核心库和设备定义

## 🛠️ 开发工作流与最佳实践

### 标准开发流程

1. **环境准备** → 按 [VS_Code_Configuration.md](./docs/VS_Code_Configuration.md) 配置
2. **编写代码** → 遵循 [代码规范.md](./docs/代码规范.md) 的所有规则
3. **格式化代码** → 运行 `format_all.ps1` 或 `format_all.sh`
4. **构建项目** → 使用 `cmake --build build`
5. **调试测试** → 使用 GDB 和 OpenOCD 进行硬件调试
6. **提交代码** → 确保通过格式化检查和编译

### 命令参考

| 任务 | 命令 |
|------|------|
| **构建项目** | `cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake && cmake --build build` |
| **清理构建** | `cmake --build build --target clean` 或 `rm -r build` |
| **格式化代码（Windows）** | `.\format_all.ps1` |
| **格式化代码（Linux/macOS）** | `./format_all.sh` |
| **启动调试** | 按 Ctrl+Shift+D，选择 "Debug with OpenOCD"，按 F5 |

## ⚙️ 环境要求

- **CMake**: >= 3.20
- **编译器**: arm-none-eabi-gcc (ARM Embedded GCC Toolchain)
- **构建工具**: Ninja（推荐；也可使用其他 CMake 支持的生成器，但请始终通过 `cmake --build` 构建，而不要直接运行 `make`）
- **代码格式化**: clang-format >= 14.0

## 📖 推荐阅读顺序

**首次使用本项目，请按以下顺序阅读文档：**

1. **本文件** (README.md) - 了解项目概况和构建方法
2. **[VS_Code_Configuration.md](./docs/VS_Code_Configuration.md)** - 配置开发环境
3. **[代码规范.md](./docs/代码规范.md)** - 学习编码规范
4. **[docs/BSP/](./docs/BSP/)** - 了解硬件抽象层接口（按需）
5. **[docs/Async/](./docs/Async/)** - 了解异步与 RTOS 抽象接口（按需）
6. **[docs/Components/](./docs/Components/)** - 了解通用组件接口（按需）
7. **[docs/DSP/](./docs/DSP/)** - 了解数字信号处理接口（按需）

## 🔗 外部资源

- [STM32F407 数据手册](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) - 设备配置工具
- [FreeRTOS 官方文档](https://www.freertos.org/)
- [OpenOCD 项目](https://openocd-org/openocd/) - 调试工具
- [ARM GCC 工具链](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)

## 💬 团队协作规范

开发前必读：

- 📌 **代码风格**：遵守 [代码规范.md](./docs/代码规范.md) 中的所有规则
  - 蛇形命名法（snake_case）
  - Modern C++ (C++17+)
  - 禁止 C 风格转换
  - 智能指针优先

- 🎨 **代码格式化**：提交前务必运行格式化脚本
  - Windows: `.\format_all.ps1`
  - Linux/macOS: `./format_all.sh`
