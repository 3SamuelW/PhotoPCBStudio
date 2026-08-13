# PhotoPCB Studio

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
[![Qt](https://img.shields.io/badge/Framework-Qt%205.x-blue.svg?style=for-the-badge)](https://www.qt.io/)
[![Platform](https://img.shields.io/badge/Platform-Windows-lightgrey.svg?style=for-the-badge)](https://github.com/3SamuelW/PhotoPCBStudio/releases)
[![GitHub Downloads](https://img.shields.io/github/downloads/3SamuelW/PhotoPCBStudio/total.svg?style=for-the-badge)](https://github.com/3SamuelW/PhotoPCBStudio/releases)

**将图片一键拆分为可制造 PCB 生产层 · Convert photos into manufacturable PCB layer images**

[English](#english) · [中文](#中文)

</div>

---

## 中文

### 项目简介

PhotoPCB Studio 是一款 Windows 桌面工具，专为将 2D 插画或照片快速转换为 PCB 透光艺术画生产层而设计。目标是把"视觉设计"与"可生产工艺"之间的转换流程自动化，让创客、硬件工程师、插画师都能轻松制作透光 PCB 艺术画。

### 核心功能

- **图像预处理**：高斯去噪 + 颜色量化，减少噪点，让颜色分层更干净
- **四层自动拆分**：Top Copper（线路层）、Top Mask（阻焊层）、Top Silk（丝印层）、Bottom Mask（透光层）
- **边缘操作**：Canny 描边 / 拉普拉斯增强，强化图案轮廓
- **LED 灯光预览**：模拟背面 LED 透光效果，支持自动/手动布灯
- **裸露基材绑定**：利用 PCB 基材本色作为第五种颜色
- **工程保存/加载**：`.pcblg` 格式（ZIP 打包原图+参数）
- **实时外部编辑**：与画图程序联动，Ctrl+S 后自动重新处理

### 使用流程

1. 打开程序，点击 **导入图片**
2. 在 **图像预处理** 区段调整去噪强度和色块数量
3. 调整基础参数（金属阈值、丝印阈值、透光阈值）
4. 按需开启 **边缘操作** 强化轮廓
5. 右侧展开 **生产层预览** 查看四层效果
6. 点击 **导出生产层** 保存 PNG 文件用于 EDA 工具

### 快速开始（Windows 一键运行）

直接下载 [Releases](https://github.com/3SamuelW/PhotoPCBStudio/releases) 页面的 `PhotoPCBStudio_windows_x64.zip`，解压后双击 `PhotoPCBStudio.exe` 即可运行，无需安装。

### 从源码构建

**环境要求：**
- MSYS2 UCRT64
- `mingw-w64-ucrt-x86_64-gcc`
- `mingw-w64-ucrt-x86_64-qt5-base`
- `mingw-w64-ucrt-x86_64-qt5-tools`

**构建步骤：**
```powershell
C:\msys64\ucrt64\bin\qmake-qt5.exe -spec win32-g++ PhotoPCBStudio.pro
C:\msys64\ucrt64\bin\mingw32-make.exe -f Makefile.Debug -j4
```

### 致谢

本项目参考并受到启发于 [tomatorigid/PCB_lightgraph](https://github.com/tomatorigid/PCB_lightgraph)（MIT License）。
感谢原作者的开源贡献，本项目在其基础上进行了大量重构与功能扩展，详见 [NOTICE.md](NOTICE.md)。

---

## English

### Overview

PhotoPCB Studio is a Windows desktop tool for converting 2D illustrations or photos into manufacturable PCB translucent art production layers. It automates the conversion between visual design and PCB fabrication constraints, making it easy for makers, hardware engineers, and illustrators to create translucent PCB artworks.

### Features

- **Image preprocessing**: Gaussian denoise + color quantization for cleaner layer classification
- **4-layer auto split**: Top Copper, Top Mask, Top Silk, Bottom Mask
- **Edge operations**: Canny stroke / Laplacian enhancement for sharper outlines
- **LED light preview**: Simulate back-lit LED diffusion, auto/manual placement
- **Bare substrate binding**: Use the PCB substrate color as a fifth color
- **Project save/load**: `.pcblg` format (ZIP of source image + parameters)
- **Live external editing**: Links with MS Paint; auto-reloads on Ctrl+S

### Workflow

1. Open the app and click **Import Image**
2. Tune denoise strength and color block count in **Image Preprocessing**
3. Adjust fabrication thresholds (metal, silk, translucent)
4. Optionally enable **Edge Operations** for sharper outlines
5. Expand **Production Layer Preview** on the right panel
6. Click **Export Production Layers** to save PNGs for EDA tools

### Quick Start (Windows pre-built)

Download `PhotoPCBStudio_windows_x64.zip` from [Releases](https://github.com/3SamuelW/PhotoPCBStudio/releases), unzip and double-click `PhotoPCBStudio.exe`. No installation needed.

### Build from Source

**Requirements:**
- MSYS2 UCRT64
- `mingw-w64-ucrt-x86_64-gcc`
- `mingw-w64-ucrt-x86_64-qt5-base`
- `mingw-w64-ucrt-x86_64-qt5-tools`

**Build:**
```powershell
C:\msys64\ucrt64\bin\qmake-qt5.exe -spec win32-g++ PhotoPCBStudio.pro
C:\msys64\ucrt64\bin\mingw32-make.exe -f Makefile.Debug -j4
```

### Attribution

This project was inspired by and references [tomatorigid/PCB_lightgraph](https://github.com/tomatorigid/PCB_lightgraph) (MIT License).
Many thanks to the original author for their open-source contribution. This project has been substantially restructured and extended. See [NOTICE.md](NOTICE.md) for details.

### License

MIT License — see [LICENSE](LICENSE) for details.
