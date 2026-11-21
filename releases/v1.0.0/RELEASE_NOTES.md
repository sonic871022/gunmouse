# GunMouse v1.0.0 — 稳定发布版

发布日期：2026-11-21
状态：已实机验证、用户确认可用（开枪识别正常、侧握横移已优化）

这是第一个功能完整、经过多轮实机调试的稳定版本，适合作为开源仓库的首个 tag。

## 本版功能

- **陀螺仪瞄准**：BMI270 角速度 → USB HID 鼠标移动，含滤波/死区/灵敏度可调
- **枪声射击**：内置 ES8311 麦克风检测枪声 → 鼠标左键，三套射击算法：
  - SINGLE 单发（一发只打一次、无尾部补触）
  - RAPID 连发（跟进射速，约 20 发/秒）
  - THR 粗暴阈值（超阈值即开火、持续超持续开火，推荐）
- **自动学习轴**（`L` 键）：按实际摆臂动作自动把横移/俯仰绑到对应陀螺仪轴，解决侧握映射别扭
- **可调参数掉电保存**：灵敏度、阈值、轴映射、方向、归中位移、屏幕方向全部存 NVS
- **归中/标定**：`R` 归中 + `2/3/4/5` 微调 + `t` 撞墙标定
- **横竖屏切换**、调试显示、G0 按钮手动开枪

## 编译环境（重要）

- **esp32 core 3.2.1**（IDF 5.4）—— 切勿用 3.3.x，其 legacy I2S 回归会让 ES8311 麦克风返回恒定假数据
- M5Unified 0.3.0 / M5Cardputer（无需改动）
- FQBN：
  ```
  esp32:esp32:m5stack_cardputer:FlashSize=8M,PSRAM=disabled,PartitionScheme=default_8MB,USBMode=default,CDCOnBoot=default
  ```
  `USBMode=default,CDCOnBoot=default` 必带，否则被识别成串口而非 HID 鼠标。

## 烧录

整片镜像（8MB，含 bootloader + 分区表 + app，从 0x0 刷）：
```
python -m esptool --port COM3 --chip esp32s3 write_flash 0x0 gunmouse-v1.0.0-merged.bin
```

或按源码自行编译（见仓库根目录 README.md）。烧录后若 COM3 消失属正常（设备变 HID 鼠标），
重新烧录需进下载模式：电源 OFF → 按住 G0 → 拨回 ON → 松开。

## 首次使用

1. 开机静止 1.2 秒自动校准（测零偏 + 噪声底）
2. 按 `L` 学习轴：左右摆臂 2 秒 + 上下摆 2 秒；方向反了用 `i`/`k` 翻
3. THR 模式下扣扳机看屏幕 `P` 值，用 `-`/`=` 把阈值调到枪声峰值略下方
4. 用 `r` 归中，`2/3/4/5` 按屏幕实测微调
5. 进游戏后电脑静音或戴耳机（防止游戏音效被麦克风听到误触发）

## 文件与校验

| 文件 | 说明 |
|---|---|
| `gunmouse-v1.0.0.ino` | 本版源码快照（Arduino sketch，主文件） |
| `gunmouse-v1.0.0.bin` | 应用固件（643168 字节，烧到 0x10000） |
| `gunmouse-v1.0.0-merged.bin` | 8MB 整片镜像（从 0x0 整刷） |
| `SHA256SUMS.txt` | 以上文件 SHA256 校验和 |

校验：
```
sha256sum -c SHA256SUMS.txt        # Linux/macOS
certutil -hashfile <文件> SHA256    # Windows
```

## 已知事项

- 模板存储结构已改（TPL_BLOB_VER=3），旧模板不兼容，需按 `g` 重录
- 备份的原厂 CarHUD 固件见仓库 `backup/`（含设备私有 NVS，开源提交时建议排除该目录）
- 本固件不使用 PSRAM
