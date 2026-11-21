/*
 * GunMouse —— Cardputer ADV 光枪鼠标固件
 * ============================================================
 * 用途：把 Cardputer ADV 绑在电动玩具枪上，当 PC 射击游戏的"光枪"：
 *   - BMI270 陀螺仪角速度  -> USB 鼠标移动（游戏内瞄准）
 *   - 内置麦克风听枪击声   -> USB 鼠标左键（游戏内射击）
 *   - 屏幕实时显示准星/音量包络/阈值/参数
 *
 * 硬件：M5Stack Cardputer ADV（Stamp-S3A / BMI270 / ES8311 麦克风）
 * 工具链：arduino-cli + esp32 core 3.2.1 + M5Unified 0.3.0
 *         （★ 必须用 core 3.2.x/IDF 5.4：core 3.3.x 的 IDF 5.5 legacy I2S
 *           有回归 bug，ES8311 麦克风返回恒定假数据 esp-idf#18621）
 *         （严禁直写 ES8311 寄存器 —— 之前写错地址导致假静音）
 *
 * 按键说明（直接按字母/符号键）：
 *   ,  / .        灵敏度 - / +
 *   [ / ]         有模板: 匹配度阈值 - / +（默认 85%）| 无模板: 枪声触发阈值 - / +
 *   - / =         ★ 枪声触发阈值 - / +（THR 模式专用，任何模式下都生效，×1.3 每档）
 *   y / z         切换"水平转角" / "俯仰"所用陀螺仪轴（z 保留其原有功能）
 *   i / k         反转水平 / 俯仰方向
 *   2 / 3         归中 X 偏移 - / +（微调，实时生效并存盘，每次 10 count）
 *   4 / 5         归中 Y 偏移 - / +（微调，实时生效并存盘，每次 10 count）
 *   o / p         测试点击：o=鼠标左键 p=鼠标右键（HID 通断自测）
 *   r             归中：把电脑鼠标指针移到屏幕正中央（X/Y 偏移用 2/3、4/5 微调）
 *   t             归中标定：撞墙归零后，按住 o 推到水平中心按 t，再按住 p 推到垂直中心按 t
 *   f             切换射击模式（三套独立算法，循环）：
 *                   SINGLE=单发：命中即清窗口+锁180ms+等谷底，一发只打一次、无尾部补触
 *                   RAPID =连发：命中等谷底即解锁、不清窗口，跟进射速
 *                   THR   =★粗暴阈值：不看模板/相关系数，实时音量>阈值即开火，
 *                          持续超阈值就持续开火（按扣扳机节奏连发）。阈值用 - / = 实时调
 *   g             追加枪声模板：按后录 1 秒，期间开一枪，存为一个样本（最多 8 个）
 *                 同一支枪在不同角度/距离/力度多录几次可显著提高识别率
 *   h             清空模板列表（切回无模板的能量阈值触发模式）
 *   c             重新校准（陀螺仪零偏 + 环境噪声）
 *   d             开/关 调试信息
 *   1             旋转屏幕方向 90°（横屏 240x135 <-> 竖屏 135x240，掉电保存；
 *                 竖屏时画布软件旋转铺满，不会出现分屏/镜像；避开 z 键冲突）
 *   G0 按钮       手动开一枪（测试用）
 *
 * 重要：本固件工作在 USB-OTG(TinyUSB) HID 模式，CDC 关闭。
 *   烧录后 COM3 会消失（正常现象，因为设备变成了 USB 鼠标）。
 *   再次烧录：关机 -> 按住 G0 -> 开机（进入下载模式）-> COM3 出现即可上传。
 */

#include "M5Cardputer.h"
#include "USB.h"          // 关键：TinyUSB 设备栈，必须 include 且调用 USB.begin()
#include "USBHIDMouse.h"
#include <Preferences.h>

/* ============================================================
 * ★★★ 可调参数：改这两个数字即可微调鼠标归中位置 ★★★
 * 作用：按 R 键时，把光标推到屏幕中心所需的相对位移量。
 * 你的屏幕 2560x1440 + Windows 鼠标加速开启，实测反推值如下。
 * 若按 R 后落点偏右 → 调大 CENTER_X；偏左 → 调小。
 * 若落点偏下 → 调大 CENTER_Y；偏上 → 调小。
 * 改完重新编译烧录即可；也可以用 2/3、4/5 键在设备上实时微调（会自动保存并覆盖这里）。
 * ============================================================ */
static constexpr int32_t CENTER_X = 934;   // 归中水平位移 count（用户在设备上微调确认的最终值）
static constexpr int32_t CENTER_Y = 532;   // 归中垂直位移 count
// 配置版本：上面两个常量改动后，把这个数字 +1，烧录后即可强制覆盖设备里的旧值。
// （日常用 2/3、4/5 键微调即可，会自动存盘；这个版本号主要防止旧 NVS 值盖掉新常量）
static constexpr uint8_t  CENTER_CFG_VER = 4;

/* ============================ 常量 ============================ */
static constexpr uint32_t MIC_RATE     = 16000;  // 麦克风采样率 Hz
static constexpr size_t   MIC_BLOCK    = 64;     // 每块样本数 (64/16k = 4ms)
static constexpr float    GYRO_DZ_DPS = 2.0f;    // 陀螺仪死区 (deg/s)：过滤零偏导致的缓慢漂移
static constexpr uint32_t PRESS_MS    = 45;     // 单发模式左键按下时长
static constexpr uint32_t REARM_MS    = 400;     // 无模板兜底模式：最长恢复时间
static constexpr uint32_t HOLD_OFF_MS = 150;     // 连发模式安静多久后松开
static constexpr float    QUIET_RATIO  = 0.55f;  // "安静"判定 = 阈值 x 该系数

// ---- 枪声模板匹配 ----
static constexpr int      TPL_FRAMES   = 25;     // 模板帧数：25 x 4ms = 100ms 包络
static constexpr int      TPL_PEAK_IDX = 12;     // ★ 模板峰值锚定在中间帧（原为 tpl[1]）：
                                                 //   峰值刚出现在实时窗口末尾(pk=24)时 off=pk-12=12，
                                                 //   仍有 13 帧重叠可算相关 → 几乎零延迟；
                                                 //   锚在 tpl[1] 要等峰值滑到窗口中前部(~40ms)才够重叠，
                                                 //   连发每一发都被这个延迟跳过。
static constexpr int      MAX_TPL      = 8;      // 模板列表容量：同一支枪可预存多个声音样本
static constexpr uint32_t REC_MS       = 1000;   // G 键录音时长
static constexpr uint32_t COOLDOWN_MS  = 50;     // 连发(RAPID)最短射击间隔兜底（约 20 发/秒上限）。
static constexpr uint32_t SINGLE_LOCK_MS = 180;  // 单发(SINGLE)命中后最短锁定：防一枪多点/尾部补触
static constexpr float    REARM_RATIO  = 0.35f;  // 谷底复位：能量回落到门限×该系数以下才允许下一发
static constexpr uint32_t REARM_TIMEOUT_MS = 500;// 谷底复位超时兜底：防止枪声连成一片时永不解锁
static constexpr int      TPL_PK_SEARCH = 3;     // 峰值锚定对齐：在 ±3 帧内搜索最佳相关（容忍峰值定位误差）
static constexpr uint16_t TPL_BLOB_VER = 3;      // NVS 模板版本（3=峰值锚中间+裁切跳前导噪声；旧模板作废需重录）
static constexpr int32_t  TPL_MIN_PEAK = 800;    // 录音峰值下限：低于此认为没录到声
// ★ ADV 绑在墙上离枪声极近、信号极强 —— 门槛要高才能屏蔽房间环境音/说话声。
//   高门限只会"听不见远处的杂音"，贴近的枪声信号远超门限，不会漏判。
static constexpr float    TPL_GATE_R   = 0.35f;  // 能量门限 = 模板峰值×系数（0.18→0.35，挡环境音）
static constexpr float    CORR_THR_DEFAULT = 0.85f; // 匹配度阈值（0.78→0.85，环境音形状再像也够不到）
static constexpr uint8_t  CORR_CFG_VER = 3;      // 匹配度阈值配置版本：+1 强制用新默认值覆盖旧 NVS

// ---- THR 粗暴阈值模式（不看模板/相关系数，纯音量阈值）----
// ★ 16 位采样峰值物理封顶 32767，阈值上限若卡在 32000 就再也调不高。
//   把「音量与阈值」的显示/比较量纲统一放大 THR_SCALE 倍：
//   - 阈值可设到百万级（细粒度调节，满足"更大声音才开火"）
//   - 用户已存的旧阈值在加载时等比放大，触发语义完全不变
static constexpr int32_t THR_SCALE    = 32;              // 音量/阈值量纲放大倍数
static constexpr int32_t THR_MIN      = 400   * THR_SCALE;    // 12800
static constexpr int32_t THR_MAX      = 32000 * THR_SCALE;    // 1024000（约百万上限）
static constexpr float   THR_STEP     = 1.3f;            // - / = 每档倍率（拉高快、回调也快）
static constexpr int32_t THR_AUTO_MIN = 3000  * THR_SCALE;    // 96000：自动校准默认下限，防环境音误触

// ---- ES8311 麦克风硬件增益（本次下调：恢复动态范围、避免贴脸枪声削波）----
// 官方 CardputerADV 默认：PGA=0x10(最小)、ADC_VOL=0xBF(0dB)。
// 若降低增益后枪声音量太小触发不了，可把 MIC_PGA_GAIN 往上调（每 +1 码约 +3dB）：
//   0x10=0dB, 0x13=+9dB, 0x17=+21dB(旧版削波值)。ADC_VOL 0xBF=0dB, 每 +1 约 +0.5dB。
static constexpr uint8_t MIC_PGA_GAIN = 0x10;
static constexpr uint8_t MIC_ADC_VOL  = 0xBF;
// 增益版本：+1 时强制丢弃旧阈值并重新校准（增益变了，旧阈值量纲不再适用）
static constexpr uint8_t MIC_GAIN_VER = 1;

// HID 键码（Keyboard.keysState().hid_keys 里的值，实测即标准 USB HID 字母码）
//   A=04 B=05 C=06 D=07 E=08 F=09 G=0A H=0B ... Y=1C Z=1D 1=1E 2=1F ...
//   实测确认：Z 键=0x1D，Y 键=0x1C，数字1=0x1E，数字2=0x1F ...
enum : uint8_t {
  KEY_C = 0x06, KEY_D = 0x07, KEY_G = 0x0A, KEY_F = 0x09, KEY_H = 0x0B,
  KEY_I = 0x0C, KEY_K = 0x0E, KEY_L = 0x0F,                          // L=自动学习轴
  KEY_O = 0x12, KEY_P = 0x13, KEY_R = 0x15, KEY_T = 0x17,
  KEY_Y = 0x1C, KEY_Z = 0x1D,                          // Y=水平轴切换  Z=俯仰轴切换（保留原功能）
  KEY_1 = 0x1E, KEY_2 = 0x1F, KEY_3 = 0x20, KEY_4 = 0x21, KEY_5 = 0x22,  // 数字键 1~5
  KEY_MINUS = 0x2D, KEY_EQUAL = 0x2E,                        // '-' / '=' 专门调 THR 触发阈值
  KEY_LBRACKET = 0x2F, KEY_RBRACKET = 0x30,
  KEY_COMMA = 0x36, KEY_PERIOD = 0x37,
};

/* ======================= 可调设置 (NVS) ======================= */
enum FireMode : uint8_t { MODE_SINGLE = 0, MODE_RAPID = 1, MODE_THR = 2 };
static constexpr uint8_t MODE_COUNT = 3;

static Preferences   prefs;
static USBHIDMouse   Mouse;          // 相对鼠标：游戏瞄准 + R 键归中都用它

// 归中用的屏幕分辨率（2560×1440 中心 = 1280,720）
static constexpr int16_t SCREEN_W = 2560;
static constexpr int16_t SCREEN_H = 1440;
// 归中第 2 步推送节奏：标定(handleCalibPush) 与 归中(moveAxis) 必须完全一致，
// 否则 Windows 加速系数不同，会出现"标定准、归中偏"。
// PUSH_DELAY 越小归中越快，但 Windows 加速放大越多 → 落点会变，需重新微调 CENTER_X/Y。
static constexpr uint8_t PUSH_STEP  = 16;   // 每步 count（小步，避开加速 kill 曲线）
static constexpr uint8_t PUSH_DELAY = 4;    // 步间隔 ms（原 20ms≈1.8s；4ms≈0.5s，快 3.5 倍）

static float    sens      = 8.0f;   // 灵敏度：每度转角对应鼠标计数
static int32_t  thr       = 0;      // 枪声触发阈值（0 = 开机自动校准）
static bool     thrManual = false;  // ★ 用户是否用 - / = 手动调过阈值：true 时开机校准不再覆盖（防止调好的值被重置）
static constexpr uint8_t THR_SCALE_VER = 1;  // 阈值量纲版本：+1 时把旧 thr 一次性 ×32 迁移到新量纲
static uint8_t  yawAx     = 2;      // 水平瞄准所用陀螺仪轴 0=X 1=Y 2=Z
static uint8_t  pitAx     = 0;      // 俯仰所用轴
static bool     invYaw    = true;   // 屏幕朝上、枪口指向屏幕顶边时的默认方向
static bool     invPitch  = true;
static uint8_t  fireMode  = MODE_SINGLE;
static bool     debugInfo = false;
static uint8_t  scrRot    = 1;      // 屏幕旋转：1 或 3（两个横屏方向）

/* ========================= 运行状态 ========================= */
static float    gyroZero[3]  = {0, 0, 0};   // 陀螺仪零偏 (deg/s)
static float    noiseFloor   = 0;           // 环境噪声底
static int16_t  lastPeak     = 0;          // 最近一块的峰值（全局，供 draw() 显示）
static float    envelope     = 0;          // 平滑后的包络
static bool     mouseDown    = false;
static uint32_t tPress       = 0;          // 按下时刻
static uint32_t tLastLoud    = 0;          // 最近一次"响"的时刻
static uint32_t tRelease     = 0;          // 松开时刻
static bool     refract      = false;      // 单发模式：恢复期
static uint32_t shots        = 0;
static uint32_t fireFlashUntil = 0;
static uint32_t centerFlashUntil = 0;     // R 键归中反馈：屏幕显示 "CENTERED" 到该时刻
static float    dxAcc = 0, dyAcc = 0;       // 鼠标位移累加器

// ---- 归中自校准（标定"从左上角到屏幕中心"所需的鼠标 count，消除 Windows 加速影响）----
static uint8_t  calState = 0;   // 0=空闲 1=标定X 2=标定Y
static int32_t  calCount = 0;   // 当前标定已推送的 count
static int32_t  centerX  = CENTER_X;   // 归中 X 位移（可被 2/3 键微调）
static int32_t  centerY  = CENTER_Y;   // 归中 Y 位移（可被 4/5 键微调）
static bool     centerDirty = false;   // 归中值被微调过，待空闲后补存
static uint32_t lastCenterEdit = 0;    // 最近一次微调时刻
static uint32_t bootAt   = 0;   // 开机时刻：前 2.5 秒抑制鼠标输出，消除开机漂移
// ---- 自动学习轴（L 键）----
static float    learnRange[3] = {0, 0, 0};  // 学习阶段各轴角速度的 max-min 跨度
static bool     learnActive = false;        // 学习流程进行中（抑制摇杆/枪声干扰显示）
static float    aimX = 120, aimY = 70;      // 屏幕上的装饰准星位置
static float    rYaw = 0, rPit = 0;         // 平滑后的角速度
static uint32_t stillMs = 0;               // 持续静止时长（用于自适应零偏）
static bool     micOk = true;              // 麦克风数据是否在变化
static int16_t  flatSample  = 0;
static uint32_t flatCount   = 0;
static uint32_t lastSaveMs  = 0;

// ---- 枪声模板列表（G 追加 / H 清空 / NVS 持久化）----
static int16_t  recBuf[MIC_RATE];               // 录音缓冲：1 秒 @16kHz = 32KB（静态分配）
static float    tplRms[MAX_TPL][TPL_FRAMES];    // 模板列表：每个模板 25 帧 RMS 包络（旧 -> 新）
static float    tplPeak[MAX_TPL] = {0};         // 每个模板的最高单帧 RMS
static int      tplCount = 0;                   // 已存模板数（0 = 能量阈值模式）
static bool     tplValid = false;               // = tplCount>0，保留该变量简化各处判断
static float    corrThr   = CORR_THR_DEFAULT;   // 匹配度阈值（[ ] 键调节）
static float    lastCorr  = 0;                  // 最近一次匹配度（显示用）
static int      lastHitTpl = -1;                // 最近命中的模板序号（显示用）
static float    histRms[TPL_FRAMES];       // 实时帧 RMS 滑动窗口（环形）
static int      histHead  = 0;             // 环形写指针（指向最旧帧）
static int      histLen   = 0;             // 已积累帧数（不足 TPL_FRAMES 不判定）
static uint32_t tLastTrig = 0;            // 最近一次匹配触发时刻（冷却计时基准）
static uint32_t tThrRepeat = 0;           // THR 模式：上次"持续超阈值连发"的时刻
static bool     shotLatched = false;      // 本发枪声已命中并锁存，等待能量回落到谷底才解锁
                                          //   SINGLE: 额外要求锁定 SINGLE_LOCK_MS + 谷底双条件
                                          //   RAPID : 只等谷底，跟进射速
static uint32_t tShotStart = 0;           // 进入锁存的时刻
static float    gateRms   = 0;            // 当前生效的能量门限（各模板峰值最小值 × TPL_GATE_R）

static int16_t  micBuf[MIC_BLOCK];
// ---- 按键调试：显示最近按下的键码（排查键位映射用）----
static uint8_t  dbgKey    = 0;
static uint32_t dbgKeyAt  = 0;
static char     micDiag[64] = "mic diag n/a";   // D 键显示的麦克风诊断信息

// ---- 新版 M5Unified Mic：record() 是异步队列，数据在后台任务填充后回调 ----
// 官方示例 Microphone.ino 的做法：setBufferReleaseCallback + 等回调再读数据。
static volatile bool micReady = false;
static void micReleaseCb(void*, void*, size_t) { micReady = true; }

// 投递一块录音请求并等待数据填充完成（最多等 timeoutMs）
// 第三参数必须是真实采样率 MIC_RATE —— 传 portMAX_DELAY 会被当成非法采样率直接失败。
static bool micRecordBlock(int16_t* buf, size_t len, uint32_t timeoutMs) {
  micReady = false;
  if (!M5Cardputer.Mic.record(buf, len, MIC_RATE)) return false;
  uint32_t t0 = millis();
  while (!micReady && (millis() - t0) < timeoutMs) delay(1);
  return micReady;
}
static M5Canvas canvas(&M5Cardputer.Display);

static void runCalibration(bool fromKey);   // 前置声明（handleKeys 中使用）

/* ======================== NVS 存取 ======================== */
static void saveSettings() {
  uint32_t now = millis();
  if (now - lastSaveMs < 500) return;          // 简单限流，保护 flash
  lastSaveMs = now;
  prefs.putFloat  ("sens",    sens);
  prefs.putInt    ("thr",     thr);
  prefs.putBool   ("thrMan",  thrManual);
  prefs.putUChar  ("thrScVer", THR_SCALE_VER);
  prefs.putUChar  ("yawAx",   yawAx);
  prefs.putUChar  ("pitAx",   pitAx);
  prefs.putBool   ("invYaw",  invYaw);
  prefs.putBool   ("invPitch",invPitch);
  prefs.putUChar  ("mode",    fireMode);
  prefs.putBool   ("debug",   debugInfo);
  prefs.putInt    ("centerX", centerX);
  prefs.putInt    ("centerY", centerY);
  prefs.putUChar  ("centerVer", CENTER_CFG_VER);
  prefs.putUChar  ("scrRot",  scrRot);
  prefs.putFloat  ("corrThr", corrThr);
}

static void loadSettings() {
  sens      = prefs.getFloat("sens",     8.0f);
  thr       = prefs.getInt  ("thr",      0);
  thrManual = prefs.getBool ("thrMan",   false);
  // ★ 量纲迁移：旧版 thr 是原始量纲（≤32000），旧版无 thrScVer（读到 0）。
  //   一次性 ×THR_SCALE 转到新量纲，保证用户已调好的触发门限完全不变；写完版本号以后不再迁移。
  if (prefs.getUChar("thrScVer", 0) < THR_SCALE_VER) {
    if (thr > 0) thr *= THR_SCALE;
    if (thr > THR_MAX) thr = THR_MAX;
    prefs.putInt   ("thr",     thr);            // 迁移后立刻落盘
    prefs.putUChar ("thrScVer", THR_SCALE_VER);
  }
  // ★ 增益下调后：麦克风信号量纲变了，旧阈值不再适用 → 作废，让开机校准按新增益重算。
  if (prefs.getUChar("micGainVer", 0) < MIC_GAIN_VER) {
    thr = 0; thrManual = false;
    prefs.putInt  ("thr",     thr);
    prefs.putBool ("thrMan",  thrManual);
    prefs.putUChar("micGainVer", MIC_GAIN_VER);
  }
  yawAx    = prefs.getUChar("yawAx",    2);
  pitAx    = prefs.getUChar("pitAx",    0);
  invYaw   = prefs.getBool ("invYaw",   true);
  invPitch = prefs.getBool ("invPitch", true);
  fireMode = prefs.getUChar("mode",     MODE_SINGLE);
  if (fireMode >= MODE_COUNT) fireMode = MODE_SINGLE;
  debugInfo= prefs.getBool ("debug",    false);
  uint8_t centerVer = prefs.getUChar("centerVer", 0);
  if (centerVer == CENTER_CFG_VER) {
    // 版本一致：沿用设备上微调保存的值
    centerX  = prefs.getInt  ("centerX",  CENTER_X);
    centerY  = prefs.getInt  ("centerY",  CENTER_Y);
  } else {
    // 版本不同（首次烧录 / 源码常量刚改过）：强制用源码常量覆盖旧 NVS
    centerX  = CENTER_X;
    centerY  = CENTER_Y;
    prefs.putInt   ("centerX", centerX);
    prefs.putInt   ("centerY", centerY);
    prefs.putUChar ("centerVer", CENTER_CFG_VER);
  }
  scrRot   = prefs.getUChar("scrRot",   1);
  if (scrRot != 1 && scrRot != 3) scrRot = 1;
  // ---- 枪声模板列表（NVS blob：ver + count + N 个模板）----
  {
    #pragma pack(push, 1)
    struct TplBlob {
      uint16_t ver;
      uint8_t  count;
      struct {
        uint16_t rms[TPL_FRAMES];
        uint16_t peak;
      } items[MAX_TPL];
    } blob;
    #pragma pack(pop)
    size_t need = sizeof(blob);
    if (prefs.getBytesLength("tpl") == (int)need
        && prefs.getBytes("tpl", &blob, need) == need
        && blob.ver == TPL_BLOB_VER && blob.count > 0 && blob.count <= MAX_TPL) {
      tplCount = blob.count;
      for (int t = 0; t < tplCount; t++) {
        tplPeak[t] = (float)blob.items[t].peak;
        for (int i = 0; i < TPL_FRAMES; i++) tplRms[t][i] = (float)blob.items[t].rms[i];
      }
      tplValid = true;
    } else {
      tplCount = 0;
      tplValid = false;
    }
  }
  // 匹配度阈值：版本不一致时用新默认值覆盖旧 NVS（避免 0.80 旧值继续误触发）
  if (prefs.getUChar("corrVer", 0) != CORR_CFG_VER) {
    corrThr = CORR_THR_DEFAULT;
    prefs.putFloat("corrThr", corrThr);
    prefs.putUChar("corrVer", CORR_CFG_VER);
  } else {
    corrThr = prefs.getFloat("corrThr", CORR_THR_DEFAULT);
  }
  // 兜底范围保护（微调键 2/3/4/5 把值限制在这个区间内）
  if (centerX < 1 || centerX > 2000) centerX = CENTER_X;
  if (centerY < 1 || centerY > 2000) centerY = CENTER_Y;
  if (sens < 0.5f)  sens = 0.5f;
  if (sens > 200.f) sens = 200.f;
}

/* ======================== 开火控制 ======================== */
static void releaseMouse() {
  if (mouseDown) {
    Mouse.release(MOUSE_LEFT);
    mouseDown = false;
    tRelease = millis();
  }
}

static void fire() {
  if (mouseDown) return;
  Mouse.press(MOUSE_LEFT);
  mouseDown = true;
  tPress    = millis();
  tLastLoud = tPress;
  shots++;
  fireFlashUntil = tPress + 120;
}

/* ======================== 鼠标归中 ======================== */
// Windows 不会把"绝对鼠标"HID 接口当作系统光标，所以只能用相对鼠标。
// 相对鼠标没有绝对坐标，归中只能两步：
//   1) 先往左上撞墙，把光标卡死在屏幕 (0,0)
//   2) 再"分轴 + 慢速"推到中心 (SCREEN_W/2, SCREEN_H/2)
// 分轴避免斜向合成速度触发加速曲线跑成椭圆；慢速(每步延时)让 Windows 按 1:1 处理。
// 分轴推送：标定与归中都用同一组参数（每步 16 count / 20ms），
// 这样 Windows 鼠标加速对两者的作用完全一致，标定的 count 归中时必然落到同一点。
static void moveAxis(int32_t delta, bool horizontal, uint8_t stepDelay, int8_t stepSize = 16) {
  int32_t rem  = delta < 0 ? -delta : delta;
  int8_t  sign = delta < 0 ? -1 : 1;
  while (rem > 0) {
    int8_t s = rem > stepSize ? stepSize : (int8_t)rem;
    if (horizontal) Mouse.move((int8_t)(sign * s), 0);
    else            Mouse.move(0, (int8_t)(sign * s));
    rem -= s;
    delay(stepDelay);
  }
}

static void recenterMouse() {
  // 第 1 步：撞墙归零（快速推，加速放大反而有利），先 X 后 Y 分轴
  moveAxis(-SCREEN_W * 2, true,  1, 40);
  moveAxis(-SCREEN_H * 2, false, 1, 40);
  delay(120);                       // 等系统把光标稳定夹在左上角
  // 第 2 步：用标定值慢速推到中心（未标定则用半屏兜底）
  int32_t tx = centerX > 0 ? centerX : SCREEN_W / 2;
  int32_t ty = centerY > 0 ? centerY : SCREEN_H / 2;
  moveAxis(tx, true,  PUSH_DELAY, PUSH_STEP);
  moveAxis(ty, false, PUSH_DELAY, PUSH_STEP);
  centerFlashUntil = millis() + 1200;  // 屏幕显示 "CENTERED" 1.2 秒作为反馈
}

/* ======================== 按键处理 ======================== */
static bool rPrevDown = false;      // R 键边沿锁存：防止按住时 repeat 反复触发归中
static void recordTemplate();       // 前置声明（G 键使用）
static void learnAxes();            // 前置声明（L 键使用：自动学习轴，阻塞 ~4.5s）

// ★ 调节枪声触发阈值：每次乘/除 THR_STEP，钳位在 [THR_MIN, THR_MAX]。
//   一经手动调整就置 thrManual，开机校准不再覆盖；按 C 重新校准可恢复自动值。
static void adjustThr(bool up) {
  // 从 0 起步时先给一个基准值，否则 0×任何倍率还是 0
  int32_t base = thr < THR_MIN ? THR_MIN : thr;
  float v = (float)base * (up ? THR_STEP : 1.0f / THR_STEP);
  int32_t nv = (int32_t)lroundf(v);
  if (nv < THR_MIN) nv = THR_MIN;
  if (nv > THR_MAX) nv = THR_MAX;
  thr = nv;
  thrManual = true;
  tThrRepeat = 0;   // 阈值变了，连发节奏重新开始
}

static void handleKeys() {
  auto ks = M5Cardputer.Keyboard.keysState();   // 每帧都读，用于边沿检测

  // R 键：只在"松开→按下"的上升沿触发一次归中（按住不动不重复）
  bool rDown = false;
  for (auto k : ks.hid_keys) if (k == KEY_R) { rDown = true; break; }
  if (rDown && !rPrevDown) recenterMouse();
  rPrevDown = rDown;

  if (!M5Cardputer.Keyboard.isChange()) return;
  bool changed = false;
  for (auto k : ks.hid_keys) {
    dbgKey = (uint8_t)k;               // 记录最近键码，屏幕显示 3 秒（调试用）
    dbgKeyAt = millis();
    switch (k) {
      case KEY_COMMA:     sens *= 0.8f; if (sens < 0.5f) sens = 0.5f; changed = true; break;
      case KEY_PERIOD:    sens *= 1.25f; if (sens > 200.f) sens = 200.f; changed = true; break;
      // ★ 阈值调节统一走 adjustThr()：- / = 万能；[ / ] 在"无模板"或"THR 模式"下也调阈值
      case KEY_MINUS:     adjustThr(false); changed = true; break;
      case KEY_EQUAL:     adjustThr(true);  changed = true; break;
      case KEY_LBRACKET:  if (tplValid && fireMode != MODE_THR) { corrThr -= 0.05f; if (corrThr < 0.50f) corrThr = 0.50f; }
                          else                                   { adjustThr(false); }
                          changed = true; break;
      case KEY_RBRACKET:  if (tplValid && fireMode != MODE_THR) { corrThr += 0.05f; if (corrThr > 0.95f) corrThr = 0.95f; }
                          else                                   { adjustThr(true); }
                          changed = true; break;
      case KEY_Y:         yawAx = (yawAx + 1) % 3; changed = true; break;
      case KEY_Z:         pitAx = (pitAx + 1) % 3; changed = true; break;  // Z 键：保留原有"俯仰轴切换"功能
      case KEY_L:         releaseMouse(); dxAcc = dyAcc = 0;               // 学习期间抑制鼠标乱动
                          learnAxes();                                     // 阻塞 ~4.5s，自动定水平/俯仰轴
                          learnActive = false; dxAcc = dyAcc = 0;
                          changed = false; break;                          // learnAxes 内部已 saveSettings
      // 归中 X/Y 微调：每次 ±10 count，实时生效；松开后空闲自动补存（避开 saveSettings 限流）
      case KEY_2:         centerX -= 10; if (centerX < 1)    centerX = 1;    centerDirty = true; lastCenterEdit = millis(); break;
      case KEY_3:         centerX += 10; if (centerX > 2000) centerX = 2000; centerDirty = true; lastCenterEdit = millis(); break;
      case KEY_4:         centerY -= 10; if (centerY < 1)    centerY = 1;    centerDirty = true; lastCenterEdit = millis(); break;
      case KEY_5:         centerY += 10; if (centerY > 2000) centerY = 2000; centerDirty = true; lastCenterEdit = millis(); break;
      case KEY_I:         invYaw   = !invYaw;   changed = true; break;
      case KEY_K:         invPitch = !invPitch; changed = true; break;
      case KEY_O:         if (calState==0) { Mouse.click(MOUSE_LEFT);  fireFlashUntil = millis() + 120; } break;  // 左键测试（校准时用于推X）
      case KEY_P:         if (calState==0) { Mouse.click(MOUSE_RIGHT); fireFlashUntil = millis() + 120; } break;  // 右键测试（校准时用于推Y）
      case KEY_R:         break;                          // 归中已在上面按边沿触发
      case KEY_T:                                        // 归中标定：三步确认
        if (calState == 0) {
          moveAxis(-SCREEN_W * 2, true,  1, 40);         // 撞墙归零 X
          moveAxis(-SCREEN_H * 2, false, 1, 40);         // 撞墙归零 Y
          delay(120);
          calCount = 0; calState = 1;                    // 进入"标 X"
        } else if (calState == 1) {
          centerX = calCount;                            // 记下水平中心 count
          moveAxis(-SCREEN_W * 2, true, 1, 40);          // 只把 X 撞回左边界
          calCount = 0; calState = 2;                    // 进入"标 Y"
        } else if (calState == 2) {
          centerY = calCount;                            // 记下垂直中心 count
          calState = 0;
          lastSaveMs = 0; saveSettings();                // 强制保存标定值
          centerFlashUntil = millis() + 1500;
        }
        break;
      case KEY_F:         fireMode = (fireMode + 1) % MODE_COUNT;   // SINGLE -> RAPID -> THR -> SINGLE
                          releaseMouse();
                          shotLatched = false; gateRms = 0; tThrRepeat = 0;
                          histLen = histHead = 0;
                          changed = true; break;
      case KEY_G:         recordTemplate(); changed = true; break;  // 追加枪声模板（阻塞 ~1s，最多 8 个）
      case KEY_H:         prefs.remove("tpl");                     // 清空模板列表，切回能量阈值模式
                          tplCount = 0; tplValid = false; lastCorr = 0; lastHitTpl = -1;
                          for (int t = 0; t < MAX_TPL; t++) tplPeak[t] = 0;
                          histLen = histHead = 0;
                          shotLatched = false; gateRms = 0;
                          changed = true; break;
      case KEY_D:         debugInfo = !debugInfo; changed = true; break;
      case KEY_1:         scrRot = (scrRot == 1) ? 3 : 1;                 // 数字1键：横竖屏切换（避开 Z 键冲突）
                          M5Cardputer.Display.setRotation(scrRot);
                          // 旋转后必须重设 pivot 到新屏幕中心，pushRotateZoom 才会居中
                          M5Cardputer.Display.setPivot(M5Cardputer.Display.width()  / 2.0f - 0.5f,
                                                       M5Cardputer.Display.height() / 2.0f - 0.5f);
                          changed = true; break;
      default: break;   // KEY_C 校准在别处单独处理（需阻塞流程）
    }
    if (k == KEY_C) {
      // 重新校准
      runCalibration(true);
      changed = true;
    }
  }
  if (changed) saveSettings();
}

// 校准期间按住 O 推 X / 按住 P 推 Y，节奏与归中完全一致（16count/20ms）
static void handleCalibPush() {
  if (calState == 0) return;
  auto ks = M5Cardputer.Keyboard.keysState();
  if (calState == 1) {
    for (auto k : ks.hid_keys) if (k == KEY_O) {
      Mouse.move(PUSH_STEP, 0); calCount += PUSH_STEP; delay(PUSH_DELAY); return;
    }
  } else if (calState == 2) {
    for (auto k : ks.hid_keys) if (k == KEY_P) {
      Mouse.move(0, PUSH_STEP); calCount += PUSH_STEP; delay(PUSH_DELAY); return;
    }
  }
}

/* ======================== 画布推送（支持旋转） ======================== */
// 画布永远是 240x135 横屏布局，所有绘制坐标都按横屏写。
// 横屏 rotation=1：直接 pushSprite(0,0) 铺满 240x135。
// 竖屏 rotation=3：屏幕逻辑尺寸变成 135x240，若直接 pushSprite 会宽度溢出环绕，
//   出现"上下两半重复+镜像"。正解是把 240x135 画布软件旋转 90°，
//   旋转后正好是 135x240，完美铺满竖屏，不会溢出。
static void pushDisplay() {
  if (scrRot == 1) {
    canvas.pushSprite(0, 0);
  } else {
    // 以各自中心为轴心旋转，把横屏画布转成竖屏铺满
    canvas.setPivot(canvas.width()  / 2.0f - 0.5f, canvas.height()  / 2.0f - 0.5f);
    M5Cardputer.Display.setPivot(M5Cardputer.Display.width()  / 2.0f - 0.5f,
                                 M5Cardputer.Display.height() / 2.0f - 0.5f);
    canvas.pushRotateZoom(&M5Cardputer.Display, 90, 1.0f, 1.0f);
  }
}

/* ========================== 自动学习轴（L 键）==========================
 * 不依赖"设备怎么绑/屏幕朝哪"，直接按用户的实际动作来定轴：
 *   阶段1 提示左右摆动 -> 采样三轴角速度 2 秒，取 max-min 跨度最大的轴 = 水平轴
 *   阶段2 提示上下摆动 -> 同样采样，在剩下的轴里取跨度最大的 = 俯仰轴
 * 用"跨度"而非"平均值"：对摆动方向不敏感，也能抵消恒定零偏。
 * 结果自动保存到 NVS。方向若反了，用户按 I / K 翻转即可。
 */
static void learnAxes() {
  const char* phaseName[2] = {"SWING LEFT-RIGHT", "TILT UP-DOWN"};
  uint8_t chosen[2] = {0, 0};

  for (int phase = 0; phase < 2; phase++) {
    float lo[3] = {1e9f, 1e9f, 1e9f};
    float hi[3] = {-1e9f, -1e9f, -1e9f};

    const uint32_t t0 = millis();
    while (millis() - t0 < 2200) {
      M5Cardputer.update();
      // 倒计时提示 + 实时三轴活动条
      int secLeft = 2 - (int)((millis() - t0) / 1000);
      if (secLeft < 0) secLeft = 0;
      canvas.fillScreen(TFT_BLACK);
      canvas.setTextDatum(middle_center);
      canvas.setTextFont(2);
      canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
      canvas.drawString("LEARN AXIS", 120, 20);
      canvas.setTextFont(1);
      canvas.setTextColor(TFT_WHITE, TFT_BLACK);
      canvas.drawString(phaseName[phase], 120, 40);
      canvas.drawString(String(secLeft) + "s  KEEP MOVING", 120, 54);
      // 三轴实时条（去除零偏后的绝对值），让用户看到哪根在动
      if (M5.Imu.update() & 2 /* gyro */) {
        auto d = M5.Imu.getImuData();
        float v[3] = { d.gyro.x - gyroZero[0], d.gyro.y - gyroZero[1], d.gyro.z - gyroZero[2] };
        for (int a = 0; a < 3; a++) {
          if (v[a] < lo[a]) lo[a] = v[a];
          if (v[a] > hi[a]) hi[a] = v[a];
          float span = hi[a] - lo[a];
          int bw = (int)(fabsf(v[a]) / 4.0f * 100.0f);
          if (bw > 100) bw = 100;
          if (bw < 0) bw = 0;
          canvas.setTextColor(TFT_WHITE, TFT_BLACK);
          canvas.drawString(String(a == 0 ? "X" : (a == 1 ? "Y" : "Z")), 50, 74 + a * 14);
          canvas.fillRect(66, 69 + a * 14, bw, 8,
                          a == chosen[0] && phase == 1 ? TFT_CYAN : TFT_GREEN);
          canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
          canvas.drawString(String((int)span), 175, 74 + a * 14);
        }
      }
      canvas.setTextDatum(top_left);
      pushDisplay();
      delay(8);
    }

    // 选跨度最大的轴；阶段2 排除阶段1 已选的水平轴
    float best = -1;
    int bestAx = phase == 0 ? 0 : (chosen[0] == 0 ? 1 : 0);
    for (int a = 0; a < 3; a++) {
      if (phase == 1 && a == chosen[0]) continue;
      float span = hi[a] - lo[a];
      if (span > best) { best = span; bestAx = a; }
    }
    chosen[phase] = (uint8_t)bestAx;
    learnRange[phase] = best;
  }

  yawAx = chosen[0];
  pitAx = chosen[1];
  // 若两轴撞车（极端情况），强制俯仰换到另一个轴
  if (pitAx == yawAx) pitAx = (yawAx + 1) % 3;

  char msg[64];
  const char* axName[3] = {"X", "Y", "Z"};
  snprintf(msg, sizeof(msg), "H=%s V=%s", axName[yawAx], axName[pitAx]);
  canvas.fillScreen(TFT_BLACK);
  canvas.setTextDatum(middle_center);
  canvas.setTextFont(2);
  canvas.setTextColor(TFT_GREEN, TFT_BLACK);
  canvas.drawString("LEARNED", 120, 40);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setTextFont(2);
  canvas.drawString(msg, 120, 66);
  canvas.setTextFont(1);
  canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
  canvas.drawString("I/K flip direction if reversed", 120, 92);
  canvas.setTextDatum(top_left);
  pushDisplay();
  saveSettings();
  delay(1800);
}

/* ========================== 校准 ========================== */
// 采样 1.2 秒：保持设备静止（测陀螺仪零偏）、保持安静（测环境噪声底）
static void runCalibration(bool fromKey) {
  // C 键主动校准：清掉手动阈值标记，让本次校准值重新生效（相当于"恢复自动阈值"）
  if (fromKey) thrManual = false;
  canvas.fillScreen(TFT_BLACK);
  canvas.setTextFont(2);
  canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
  canvas.setCursor(24, 40);
  canvas.print("CALIBRATING...");
  canvas.setCursor(24, 62);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.print("Hold still & quiet!");
  pushDisplay();

  float    gSum[3] = {0, 0, 0};
  uint32_t gCount  = 0;
  int32_t  maxPeak = 0;
  const uint32_t t0 = millis();

  while (millis() - t0 < 1200) {
    M5Cardputer.update();
    if (M5.Imu.update() & 2 /* sensor_mask_gyro */) {
      auto d = M5.Imu.getImuData();
      gSum[0] += d.gyro.x;  gSum[1] += d.gyro.y;  gSum[2] += d.gyro.z;
      gCount++;
    }
    if (micRecordBlock(micBuf, MIC_BLOCK, 50)) {
      for (size_t i = 0; i < MIC_BLOCK; i++) {
        int32_t v = micBuf[i];
        if (v < 0) v = -v;
        if (v > maxPeak) maxPeak = v;
      }
    }
    delay(1);
  }

  if (gCount > 0) {
    gyroZero[0] = gSum[0] / gCount;
    gyroZero[1] = gSum[1] / gCount;
    gyroZero[2] = gSum[2] / gCount;
  }
  noiseFloor = maxPeak;
  // ★ 只有用户没手动调过阈值时，才用校准值覆盖（否则会把用户调好的高阈值冲掉）。
  //   默认下限大幅拉高到 THR_AUTO_MIN，避免环境音误触。按 C 重新校准会重置 thrManual。
  if (!thrManual) {
    int32_t autoThr = (int32_t)maxPeak * THR_SCALE * 6;
    if (autoThr < THR_AUTO_MIN) autoThr = THR_AUTO_MIN;
    if (autoThr > THR_MAX)      autoThr = THR_MAX;
    thr = autoThr;
  }
  refract   = false;
  mouseDown = false;
  envelope  = 0;
  lastPeak  = 0;
  saveSettings();
}

/* ========================== 绘制 ========================== */
static const char* axisName(uint8_t a) { return a == 0 ? "X" : (a == 1 ? "Y" : "Z"); }

static void draw() {
  const uint32_t now = millis();
  canvas.fillScreen(TFT_BLACK);

  // ---- 顶部状态行 ----
  canvas.setTextFont(1);
  canvas.setTextColor(TFT_GREEN, TFT_BLACK);
  canvas.setCursor(4, 1);
  canvas.print("GUN-MOUSE");
  canvas.setTextColor(fireMode == MODE_SINGLE ? TFT_CYAN :
                      (fireMode == MODE_RAPID ? TFT_ORANGE : TFT_MAGENTA), TFT_BLACK);
  canvas.setCursor(96, 1);
  canvas.printf(fireMode == MODE_SINGLE ? "SINGLE" :
                (fireMode == MODE_RAPID ? "RAPID " : "THR   "));
  canvas.setTextColor(mouseDown ? TFT_RED : (refract ? TFT_DARKGREY : TFT_GREEN), TFT_BLACK);
  canvas.setCursor(170, 1);
  canvas.print(mouseDown ? "FIRE!" : (refract ? "-----" : "ARMED"));

  // ---- 参数行（有模板显示匹配度阈值，无模板显示能量阈值）----
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setCursor(4, 12);
  if (fireMode == MODE_THR) {
    // THR 模式：突出显示触发阈值 T 和当前实时音量 P，方便边开枪边用 - / = 调
    canvas.setTextColor(TFT_MAGENTA, TFT_BLACK);
    canvas.printf("S%.1f T%-7ld P%-7ld", sens,
                  (long)thr, (long)lastPeak * THR_SCALE);
  } else if (tplValid) {
    canvas.setTextColor(TFT_GREEN, TFT_BLACK);
    canvas.printf("S%.1f C%d%% T%d/%d ", sens, (int)(corrThr * 100 + 0.5f), tplCount, MAX_TPL);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.printf("%s%s/%s%s SH%u", invYaw ? "-" : "+", axisName(yawAx),
                  invPitch ? "-" : "+", axisName(pitAx), (unsigned)shots);
  } else {
    canvas.printf("S%.1f T%d %s%s/%s%s SH%u", sens, (int)thr,
                  invYaw   ? "-" : "+", axisName(yawAx),
                  invPitch ? "-" : "+", axisName(pitAx), (unsigned)shots);
  }

  // ---- 归中偏移值（实时显示，方便用 2/3/4/5 微调）+ 实时匹配度 ----
  canvas.setTextColor(TFT_CYAN, TFT_BLACK);
  canvas.setCursor(4, 21);
  canvas.printf("CTR X%-4d Y%-4d", (int)centerX, (int)centerY);
  if (tplValid) {
    canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
    canvas.printf(" m%02d%%#%d", (int)(fabsf(lastCorr) * 100 + 0.5f), lastHitTpl + 1);
  }

  // ---- 装饰准星（跟随开火闪烁）----
  const int cx = (int)aimX, cy = (int)aimY;
  const bool firing = (now < fireFlashUntil);
  const uint16_t crossColor = firing ? TFT_RED : TFT_WHITE;
  canvas.drawLine(cx - 12, cy, cx - 3, cy, crossColor);
  canvas.drawLine(cx + 3,  cy, cx + 12, cy, crossColor);
  canvas.drawLine(cx, cy - 12, cx, cy - 3, crossColor);
  canvas.drawLine(cx, cy + 3,  cx, cy + 12, crossColor);
  canvas.drawCircle(cx, cy, 16, firing ? TFT_RED : TFT_MAROON);
  if (firing) canvas.fillCircle(cx, cy, 3, TFT_RED);

  // ---- R 键归中反馈 / 归中标定引导 ----
  if (calState != 0) {
    canvas.setTextDatum(middle_center);
    canvas.setTextFont(2);
    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
    canvas.drawString(calState == 1 ? "CAL X - hold O" : "CAL Y - hold P", 120, 38);
    canvas.setTextFont(4);
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString(String(calCount), 120, 62);
    canvas.setTextFont(1);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.drawString("T=OK at center", 120, 84);
    canvas.setTextDatum(top_left);
  } else if (now < centerFlashUntil) {
    canvas.setTextDatum(middle_center);
    canvas.setTextFont(4);
    canvas.setTextColor(TFT_GREEN, TFT_BLACK);
    canvas.drawString("CENTERED", 120, 45);
    canvas.setTextDatum(top_left);
  }

  // ---- 麦克风包络条（音量统一放大到 thr 的 THR_SCALE 量纲）----
  const int barX = 4, barY = 96, barW = 200, barH = 14;
  float envScaled = envelope * THR_SCALE;         // 保持包络 → 放大量纲
  float peakScaled = (float)lastPeak * THR_SCALE; // 瞬时峰值 → 放大量纲
  float scale = (float)thr * 2.5f;
  if (scale < THR_MIN * 2.5f) scale = THR_MIN * 2.5f;
  canvas.drawRect(barX - 1, barY - 1, barW + 2, barH + 2, TFT_DARKGREY);
  float shown = fireMode == MODE_THR ? peakScaled : envScaled;
  int lv = (int)((shown / scale) * barW);
  if (lv > barW) lv = barW;
  if (lv > 0) {
    uint16_t c = shown > thr ? TFT_RED : (shown > thr * QUIET_RATIO ? TFT_ORANGE : TFT_GREEN);
    canvas.fillRect(barX, barY, lv, barH, c);
  }
  int tx = (int)(((float)thr / scale) * barW);
  if (tx >= 0 && tx <= barW) canvas.fillRect(barX + tx, barY - 3, 2, barH + 6, TFT_YELLOW);
  canvas.setTextFont(1);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setCursor(barX + barW + 8, barY + 4);
  // THR 模式显示瞬时峰值(与触发判定同源)，其它模式显示保持包络；都是放大后的量纲
  canvas.printf("%ld", fireMode == MODE_THR ? (long)peakScaled : (long)envScaled);

  // ---- 底部：调试 / 提示 ----
  canvas.setCursor(4, 118);
  if (debugInfo) {                      // D 键：麦克风/系统诊断（优先，便于排查）
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.print(micDiag);
    canvas.setCursor(4, 128);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.printf("pk%ld env%ld f%d %s", (long)lastPeak * THR_SCALE,
                  (long)envelope * THR_SCALE, (int)flatCount, micOk ? "OK" : "FLAT");
  } else if (now - dbgKeyAt < 3000) {   // 最近按过的键：显示原始码（排查键位）
    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
    canvas.printf("KEY 0x%02X", (unsigned)dbgKey);
  } else if (!micOk) {
    canvas.setTextColor(TFT_RED, TFT_BLACK);
    canvas.print("MIC FLAT! press D for diag");
  } else if (fireMode == MODE_THR) {
    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
    canvas.printf("-/= thr %s   F=mode", thrManual ? "MAN" : "AUTO");
  } else {
    canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
    canvas.print(tplValid ? "g=add h=clear [ ]=corr"
                           : "g=rec h=del 2/3=X 4/5=Y r=ctr");
  }

  pushDisplay();
}

/* ========================= 鼠标瞄准 ========================= */
static void updateAim(uint32_t t0) {
  static uint32_t tPrev = 0;
  if (tPrev == 0) { tPrev = t0; return; }
  float dt = (float)(t0 - tPrev) / 1000000.0f;
  tPrev = t0;
  if (dt <= 0 || dt > 0.05f) return;

  if (M5.Imu.update() & 2 /* sensor_mask_gyro */) {
    auto d = M5.Imu.getImuData();

    // ---- 运行时自适应零偏：消除温漂导致的缓慢漂移 ----
    // 三轴角速度都很小 = 设备基本静止，此时把零偏朝当前读数极慢收敛。
    // 只有持续静止 > 1.5 秒才开始，且每帧只修正 0.3%，不会吃掉正常转动。
    float ax = fabsf(d.gyro.x), ay = fabsf(d.gyro.y), az = fabsf(d.gyro.z);
    if (ax < 4.0f && ay < 4.0f && az < 4.0f) {
      stillMs += (uint32_t)(dt * 1000000.0f);
      if (stillMs > 1500) {
        gyroZero[0] += (d.gyro.x - gyroZero[0]) * 0.003f;
        gyroZero[1] += (d.gyro.y - gyroZero[1]) * 0.003f;
        gyroZero[2] += (d.gyro.z - gyroZero[2]) * 0.003f;
      }
    } else {
      stillMs = 0;
    }

    float rawYaw = d.gyro.value[yawAx] - gyroZero[yawAx];
    float rawPit = d.gyro.value[pitAx] - gyroZero[pitAx];
    // 轻度低通，压手抖
    rYaw = rYaw * 0.65f + rawYaw * 0.35f;
    rPit = rPit * 0.65f + rawPit * 0.35f;
  }

  // 死区：把小于 GYRO_DZ_DPS 的角速度视为 0
  float yaw = rYaw, pit = rPit;
  float ay = fabsf(yaw), ap = fabsf(pit);
  yaw = ay > GYRO_DZ_DPS ? (yaw > 0 ? ay - GYRO_DZ_DPS : GYRO_DZ_DPS - ay) : 0;
  pit = ap > GYRO_DZ_DPS ? (pit > 0 ? ap - GYRO_DZ_DPS : GYRO_DZ_DPS - ap) : 0;

  dxAcc += yaw * dt * sens * (invYaw ? -1.0f : 1.0f);
  dyAcc += pit * dt * sens * (invPitch ? -1.0f : 1.0f);

  int32_t dx = (int32_t)dxAcc;  dxAcc -= dx;
  int32_t dy = (int32_t)dyAcc;  dyAcc -= dy;
  if (dx < -127) { dx = -127; dxAcc = -0.5f; }
  if (dx >  127) { dx =  127; dxAcc =  0.5f; }
  if (dy < -127) { dy = -127; dyAcc = -0.5f; }
  if (dy >  127) { dy =  127; dyAcc =  0.5f; }
  // 开机后 2.5 秒内抑制鼠标输出：此时自适应零偏仍在收敛，避免前几秒漂移
  if (millis() - bootAt < 2500) return;
  if (dx != 0 || dy != 0) Mouse.move((int8_t)dx, (int8_t)dy);

  // 屏幕上的装饰准星随手势移动
  aimX += dx * 0.35f;  aimY += dy * 0.35f;
  if (aimX < 30)  aimX = 30;
  if (aimX > 210) aimX = 210;
  if (aimY < 45)  aimY = 45;
  if (aimY > 92)  aimY = 92;
}

/* ========================= 枪声检测 ========================= */
// Pearson 相关系数：x/y 为旧->新排列的等长序列，返回 -1..1
static float pearson(const float* x, const float* y, int n) {
  float sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
  for (int i = 0; i < n; i++) {
    sx  += x[i];
    sy  += y[i];
    sxx += x[i] * x[i];
    syy += y[i] * y[i];
    sxy += x[i] * y[i];
  }
  float cov = sxy - sx * sy / n;
  float vx  = sxx - sx * sx / n;
  float vy  = syy - sy * sy / n;
  if (vx <= 0 || vy <= 0) return 0;
  return cov / sqrtf(vx * vy);
}

// 峰值锚定相关：把模板的峰值帧对齐到窗口的第 peakAt 帧后再算 Pearson。
// 枪声是瞬态冲击，模板峰值锚在 tpl[TPL_PEAK_IDX]=12（居中），
// 但实时窗口里峰值出现的列随采样相位漂移（可能在 win[0..24] 任意位置）。
// 直接逐列比会因"峰值没对齐"得低分——把模板平移到峰值对齐处再比，命中率大幅提升。
// 居中锚定的好处：峰值刚出现在窗口末尾(pk≈24)时 off≈12，仍有 13 帧重叠可算，
// 不必等峰值滑到窗口前部 → 检测延迟从 ~40ms 降到接近 0，连发能跟上。
// off > 0：模板相对窗口右移 off 帧；off < 0：左移。
static float pearsonAt(const float* win, const float* tpl, int n, int off) {
  float sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
  int cnt = 0;
  for (int i = 0; i < n; i++) {
    int j = i + off;                 // 窗口第 j 帧对模板第 i 帧
    if (j < 0 || j >= n) continue;   // 移出窗口的部分不参与
    float x = win[j], y = tpl[i];
    sx += x; sy += y; sxx += x*x; syy += y*y; sxy += x*y;
    cnt++;
  }
  if (cnt < 12) return 0;            // 重叠太少不可信
  float cov = sxy - sx * sy / cnt;
  float vx  = sxx - sx * sx / cnt;
  float vy  = syy - sy * sy / cnt;
  if (vx <= 0 || vy <= 0) return 0;
  return cov / sqrtf(vx * vy);
}

static void updateMic(const uint32_t now) {
  if (!micRecordBlock(micBuf, MIC_BLOCK, 50)) return;

  // 块峰值（显示用）+ 块 RMS（匹配用）
  int16_t peak = 0;
  double sum2  = 0;
  for (size_t i = 0; i < MIC_BLOCK; i++) {
    int16_t v = micBuf[i];
    if (v < 0) v = -v;
    if (v > peak) peak = v;
    sum2 += (double)v * v;
  }
  float rms = sqrtf((float)(sum2 / MIC_BLOCK));
  lastPeak = peak;
  envelope = peak > envelope ? (float)peak : envelope * 0.6f;

  // 滑动窗口推进（旧 -> 新，histHead 指向最旧帧位置）
  if (histLen < TPL_FRAMES) histLen++;
  histRms[histHead] = rms;
  histHead = (histHead + 1) % TPL_FRAMES;

  // 检测"假静音"：连续多块数据完全相同
  if (peak == flatSample) {
    if (++flatCount > 3000) micOk = false;
  } else {
    flatSample = peak;
    flatCount = 0;
    micOk = true;
  }

  if (fireMode == MODE_THR) {
    // ============================================================
    // ★ THR 粗暴阈值模式（用户要的"简单粗暴"方案）
    //   实时音量(envelope 的瞬时峰值 peak) > thr  ->  开火
    //   只要一直超过阈值 -> 每隔 THR_REPEAT_MS 再开一枪（模拟扣住扳机连发）
    //   完全不看模板、不算相关系数：阈值调高后，环境音够不到就不会误触。
    //   ★ peak 是原始 16 位量纲(≤32767)，thr 是放大 THR_SCALE 倍的量纲，
    //     比较前先把 peak 放大到同一量纲，保证阈值能设到百万级。
    // ============================================================
    if ((int32_t)peak * THR_SCALE > thr) {
      if (!mouseDown) {
        // 首次越过阈值：立即开火（按下左键）
        fire();
        tThrRepeat = now;
      } else if (now - tPress >= PRESS_MS) {
        // 已经按住：松开后再判断，若仍超阈值则下一轮重新按下 -> 形成连发
        releaseMouse();
        tThrRepeat = now;
      }
      tLastLoud = now;
    } else {
      // 音量掉回阈值以下：松开左键，结束本次连发
      if (mouseDown) releaseMouse();
    }
    return;   // 不走模板/相关系数
  }

  if (tplValid) {
    // ---- 多模板匹配模式 ----
    // 能量门限取"所有模板峰值的最小值 × 系数"：任何一支枪的样本都能被触发。
    if (gateRms <= 0) {
      float minPeak = tplPeak[0];
      for (int t = 1; t < tplCount; t++) if (tplPeak[t] < minPeak) minPeak = tplPeak[t];
      gateRms = minPeak * TPL_GATE_R;
    }

    const bool single = (fireMode == MODE_SINGLE);

    // ★ 谷底复位：必须用 rms（与 gateRms 同量纲！旧代码误用 peak 采样值比较，
    //   导致能量还很响时就过早解锁 → 尾部余波被当成新一枪 → 停下来后又触发一次）。
    //   SINGLE 还额外要求过了最短锁定时间，杜绝一枪多点/补触发；
    //   RAPID 只等谷底，尽快解锁以跟上连发射速。
    if (shotLatched) {
      const bool quiet = (rms < gateRms * REARM_RATIO);
      const bool lockOk = !single || (now - tShotStart >= SINGLE_LOCK_MS);
      if ((quiet && lockOk) || now - tShotStart > REARM_TIMEOUT_MS) {
        shotLatched = false;
        if (single) { histLen = histHead = 0; }  // 单发解锁清窗口，旧余波彻底出局
      }
    }

    const uint32_t cd = single ? SINGLE_LOCK_MS : COOLDOWN_MS;
    if (histLen >= TPL_FRAMES && !shotLatched && now - tLastTrig >= cd) {
      float win[TPL_FRAMES];
      for (int i = 0; i < TPL_FRAMES; i++) win[i] = histRms[(histHead + i) % TPL_FRAMES];

      int   pk = 0;
      float winPeak = 0;
      for (int i = 0; i < TPL_FRAMES; i++) {
        if (win[i] > winPeak) { winPeak = win[i]; pk = i; }
      }
      lastCorr = 0;

      if (winPeak >= gateRms) {
        float bestC = 0;
        int   bestIdx = -1;
        for (int t = 0; t < tplCount; t++) {
          int base = pk - TPL_PEAK_IDX;
          float c = 0;
          for (int d = -TPL_PK_SEARCH; d <= TPL_PK_SEARCH; d++) {
            float cd2 = pearsonAt(win, tplRms[t], TPL_FRAMES, base + d);
            if (cd2 > c) c = cd2;
          }
          if (c > bestC) { bestC = c; bestIdx = t; }
        }
        lastCorr = bestC;
        if (bestC >= corrThr && bestIdx >= 0) {
          lastHitTpl = bestIdx;
          shotLatched = true;
          tShotStart = now;
          tLastTrig = now;
          if (single) { histLen = histHead = 0; }  // ★ 单发命中即清窗口，余波无法二次匹配
          fire();
        }
      }
    }
    if (mouseDown && now - tPress >= PRESS_MS) releaseMouse();
  } else {
    // ---- 能量阈值模式（无模板时的兜底，原有逻辑）----
    if ((int32_t)peak * THR_SCALE > thr) {
      tLastLoud = now;
      if (!mouseDown) {
        fire();
        if (fireMode == MODE_SINGLE) refract = true;
      }
    }

    if (mouseDown) {
      if (fireMode == MODE_SINGLE) {
        if (now - tPress >= PRESS_MS) releaseMouse();
      } else {
        if (now - tLastLoud >= HOLD_OFF_MS) releaseMouse();
      }
    } else if (refract) {
      // 恢复期：安静下来 或 超时 后才能再次击发
      if ((int32_t)envelope * THR_SCALE < thr * QUIET_RATIO || now - tRelease > REARM_MS) refract = false;
    }
  }
}

/* ======================= 枪声模板录音 ======================= */
// G 键：录 REC_MS 毫秒，自动定位能量峰值帧，截取 100ms 包络做模板存 NVS。
// 录音是阻塞流程：期间不瞄准、不开火，画实时能量柱帮助确认麦克风活着。
static void recordTemplate() {
  releaseMouse();
  if (tplCount >= MAX_TPL) {                // 列表满：提示清空后再录
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextDatum(middle_center);
    canvas.setTextFont(2);
    canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
    canvas.drawString("LIST FULL (8)", 120, 50);
    canvas.setTextFont(1);
    canvas.drawString("press H to clear all", 120, 74);
    canvas.setTextDatum(top_left);
    pushDisplay();
    delay(1500);
    return;
  }
  canvas.fillScreen(TFT_BLACK);
  canvas.setTextFont(2);
  canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
  canvas.setTextDatum(middle_center);
  canvas.drawString("REC TEMPLATE", 120, 14);
  canvas.setTextDatum(middle_center);
  canvas.setTextFont(1);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString("FIRE ONE SHOT NOW!", 120, 36);
  canvas.setTextDatum(top_left);

  const size_t totalBlocks = MIC_RATE * REC_MS / 1000 / MIC_BLOCK;   // 250 块
  const int barCols = 60;                                            // 能量柱数
  const int barY = 52, barH = 40;
  canvas.drawRect(3, barY - 2, 234, barH + 4, TFT_DARKGREY);
  float colPeak[60] = {0};
  int   curCol = -1;
  pushDisplay();                       // ★ 先把界面推上屏：确认 G 键流程真的进来了

  auto drawBar = [&](int c) {
    int h = (int)(colPeak[c] * barH / 32767.0f);
    if (h > 0) canvas.fillRect(4 + c * 4, barY + barH - h, 3, h,
                                colPeak[c] > 8000 ? TFT_RED : TFT_GREEN);
  };

  const uint32_t t0 = millis();
  int micFail = 0;
  for (size_t b = 0; b < totalBlocks; b++) {
    if (!micRecordBlock(recBuf + b * MIC_BLOCK, MIC_BLOCK, 200)) {
      if (++micFail > 50) {           // ★ 麦克风出不来数据：明示报错，不再静默空转
        canvas.fillScreen(TFT_BLACK);
        canvas.setTextDatum(middle_center);
        canvas.setTextFont(2);
        canvas.setTextColor(TFT_RED, TFT_BLACK);
        canvas.drawString("MIC FAIL!", 120, 60);
        canvas.setTextDatum(top_left);
        pushDisplay();
        delay(1500);
        return;
      }
      b--;                            // record 失败重试本块
      if (millis() - t0 > REC_MS * 3) return;       // 超时保护
      continue;
    }
    micFail = 0;
    int16_t pk = 0;
    for (size_t i = 0; i < MIC_BLOCK; i++) {
      int16_t v = recBuf[b * MIC_BLOCK + i];
      if (v < 0) v = -v;
      if (v > pk) pk = v;
    }
    int c = (int)(b * barCols / totalBlocks);       // 当前块所属柱
    if (pk > colPeak[c]) colPeak[c] = pk;
    if (c != curCol) {                              // 进入新柱：固化上一柱
      if (curCol >= 0) drawBar(curCol);
      curCol = c;
    }
    // 进度条
    int px = (int)((b + 1) * 220 / totalBlocks);
    canvas.fillRect(8, 108, px, 6, TFT_CYAN);
    pushDisplay();
  }
  drawBar(curCol);                                  // 画最后一柱

  // ---- 定位峰值块，向前对齐截 25 帧包络 ----
  float frameRms[totalBlocks];
  int   best = 0;
  float bestRms = 0;
  for (size_t f = 0; f < totalBlocks; f++) {
    double s2 = 0;
    for (size_t i = 0; i < MIC_BLOCK; i++) {
      double v = recBuf[f * MIC_BLOCK + i];
      s2 += v * v;
    }
    frameRms[f] = sqrtf((float)(s2 / MIC_BLOCK));
    if (frameRms[f] > bestRms) { bestRms = frameRms[f]; best = (int)f; }
  }

  // ★ 噪声底估计：取全部帧 RMS 的 25% 分位数（排序后取该位）。
  //   这样按键声/环境微弱波动都被归入噪声底，不会当成枪声起点。
  float sortedRms[totalBlocks];
  memcpy(sortedRms, frameRms, sizeof(frameRms));
  // 简单插入排序（250 个 float，一次性，代价可忽略）
  for (size_t i = 1; i < totalBlocks; i++) {
    float key = sortedRms[i];
    size_t j = i;
    while (j > 0 && sortedRms[j - 1] > key) { sortedRms[j] = sortedRms[j - 1]; j--; }
    sortedRms[j] = key;
  }
  float noise = sortedRms[totalBlocks / 4];
  if (noise < 20) noise = 20;   // 防止全静音时噪声底为 0

  // ★ 找枪声"真正起振点"：从峰值往前回溯，直到 RMS 落回 (噪声 + 峰值差 × 15%) 以下。
  //   起振点之前的微弱波动（按键声/环境声）在构造模板时会被清零，不混进枪声特征。
  float onsetLevel = noise + (bestRms - noise) * 0.15f;
  int onset = best;
  while (onset > 0 && frameRms[onset - 1] >= onsetLevel) onset--;
  // 模板起点固定为 best - TPL_PEAK_IDX：保证峰值一定锚在 tpl[TPL_PEAK_IDX]，
  // 匹配端才能零延迟对齐。边界处 clamp（极早/极晚开枪时峰值位置会略偏，可接受）。
  int start = best - TPL_PEAK_IDX;
  if (start < 0) start = 0;
  if (start + TPL_FRAMES > (int)totalBlocks) start = (int)totalBlocks - TPL_FRAMES;

  canvas.fillScreen(TFT_BLACK);
  canvas.setTextDatum(middle_center);
  canvas.setTextFont(2);

  if (bestRms < (float)TPL_MIN_PEAK) {
    // 峰值太小 = 基本没录到声
    canvas.setTextColor(TFT_RED, TFT_BLACK);
    canvas.drawString("TOO QUIET - RETRY (g)", 120, 60);
    pushDisplay();
    delay(1200);
    return;
  }

  // 模板构造：tpl[i] = 录音第 (start+i) 帧，峰值锚在 tpl[TPL_PEAK_IDX]。
  // ★ 起振点 onset 之前的微弱波动一律清零 —— 按键声/环境声不混进模板，
  //   这就是用户要的"不能从第一次有波动的地方裁切"。
  int slot = tplCount;
  float peakR = 0;
  for (int i = 0; i < TPL_FRAMES; i++) {
    int src = start + i;
    float v = (src >= 0 && src < (int)totalBlocks) ? frameRms[src] : 0;
    if (start + i < onset) v = 0;                  // 前导微弱声剔除
    tplRms[slot][i] = v;
    if (v > peakR) peakR = v;
  }
  tplPeak[slot] = peakR;
  tplCount++;
  tplValid = true;
  lastCorr = 0;
  lastHitTpl = -1;
  gateRms = 0;                                // 门限需按新模板重算
  shotLatched = false;
  histLen = histHead = 0;                     // 清滑动窗口，避免旧数据污染匹配

  // ---- NVS 持久化整个模板列表 ----
  #pragma pack(push, 1)
  struct TplBlob {
    uint16_t ver;
    uint8_t  count;
    struct {
      uint16_t rms[TPL_FRAMES];
      uint16_t peak;
    } items[MAX_TPL];
  } blob;
  #pragma pack(pop)
  blob.ver   = TPL_BLOB_VER;
  blob.count = (uint8_t)tplCount;
  for (int t = 0; t < MAX_TPL; t++) {
    blob.items[t].peak = (uint16_t)lroundf(tplPeak[t]);
    for (int i = 0; i < TPL_FRAMES; i++) blob.items[t].rms[i] = (uint16_t)lroundf(tplRms[t][i]);
  }
  prefs.putBytes("tpl", &blob, sizeof(blob));

  canvas.setTextColor(TFT_GREEN, TFT_BLACK);
  canvas.drawString("TEMPLATE SAVED", 120, 46);
  canvas.setTextFont(1);
  canvas.setTextColor(TFT_CYAN, TFT_BLACK);
  canvas.drawString("slot " + String(tplCount) + "/" + String(MAX_TPL) +
                    "   peak " + String((int)peakR), 120, 70);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString("g=add more  h=clear", 120, 84);
  pushDisplay();
  delay(1300);
  canvas.setTextDatum(top_left);
}

/* ============================ setup ============================ */
void setup() {
  prefs.begin("gunmouse", false);
  loadSettings();

  // ★ 与 CarHUD 工程完全一致的初始化姿势（该工程在同一套 core/库下实测麦克风可用）：
  //   不动 internal_spk 等默认开关、不直写任何 ES8311 寄存器、只设采样率。
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);

  M5Cardputer.Display.setRotation(scrRot);
  canvas.setColorDepth(16);
  canvas.createSprite(240, 135);
  canvas.setTextDatum(top_left);

  // USB HID 鼠标 —— 两句都必须有！
  // Mouse.begin() 只注册 HID 接口描述符，USB.begin() 才真正启动 TinyUSB 设备栈。
  // 漏掉 USB.begin() 会导致设备停留在硬件 JTAG/serial 模式、电脑认不出鼠标。
  Mouse.begin();
  USB.begin();

  // ★ 不要调用 Speaker.end()：
  //   Cardputer ADV 的 speaker 用 I2S_NUM_1、mic 用 I2S_NUM_0，互不冲突；
  //   而 Speaker.end() 会触发 ES8311 的 power-down 序列(0x0D=0xFC,0x00=0x00)，
  //   这正是 CarHUD（能用）与我们（全 0）的关键差异。CarHUD 从不断开 speaker。
  if (M5.Mic.isEnabled()) {
    auto mcfg = M5.Mic.config();
    mcfg.sample_rate = MIC_RATE;
    mcfg.stereo = 1;             // 立体声：覆盖麦克风接左/右声道的不确定性
    M5.Mic.config(mcfg);
    M5.Mic.setBufferReleaseCallback(nullptr, micReleaseCb);
    M5.Mic.begin();

    // ★ ES8311 完整初始化（关键修复）
    // M5Unified 只在板型识别为 CardputerADV 时才通过 mic_enable_cb 执行这段序列。
    // 若运行时板型识别成普通 Cardputer，回调不注册 → ES8311 一直 power down → I2S 全 0。
    // 这里照抄库源码 _microphone_enabled_cb_cardputer_adv 的 enabled_bulk_data，确保无论板型如何都初始化到位。
    //
    // ★★ 增益下调（本次修复"太容易触发/削波"）★★
    // 旧版误用 stopwatch 的超高增益 0x14=0x17(PGA+21dB)+0x17=0xCB(ADC+6dB)，
    // 相当于把麦克风信号放大约 22 倍 → 贴脸枪声直接削波顶到 32767，环境音也跟着爆表，
    // 阈值怎么调都区分不开。现改回 CardputerADV 官方低增益：
    //   0x14=0x10 (PGA 最小) / 0x17=0xBF (ADC 0dB) —— 与库默认完全一致。
    // 恢复真实动态范围后，用 - / = 把阈值调到"枪声能过、环境音过不了"即可。
    // MIC_PGA_GAIN 可按需微调（0x10=最小, 每 +1 码约 +3dB, 0x17=+21dB）。
    static constexpr uint8_t es8311Init[][2] = {
      {0x00, 0x80},  // RESET/CSM POWER ON
      {0x01, 0xBA},  // CLOCK_MANAGER: MCLK=BCLK
      {0x02, 0x18},  // CLOCK_MANAGER: MULT_PRE=3
      {0x0D, 0x01},  // SYSTEM: Power up analog circuitry
      {0x0E, 0x02},  // SYSTEM: Enable analog PGA + ADC modulator
      {0x14, MIC_PGA_GAIN},  // ADC_REG14: Mic1p-Mic1n / PGA 增益（0x10 最小=官方默认）
      {0x17, MIC_ADC_VOL },  // ADC_REG17: ADC_VOLUME（0xBF=0dB 官方默认）
      {0x1C, 0x6A},  // ADC_REG1C: EQ bypass, cancel DC offset
    };
    for (auto &r : es8311Init) {
      M5.In_I2C.writeRegister8(0x18, r[0], r[1], 100000);
    }
    delay(50);   // 等 codec 模拟电路稳定

    // 读回寄存器验证写入，并采样诊断信息
    uint8_t r14 = M5.In_I2C.readRegister8(0x18, 0x14, 100000);
    uint8_t r0d = M5.In_I2C.readRegister8(0x18, 0x0D, 100000);
    auto mc = M5.Mic.config();
    snprintf(micDiag, sizeof(micDiag), "B%u P%u/%u R%u/%u E%u R14=%02X D%02X",
             (unsigned)M5.getBoard(), (unsigned)mc.pin_data_in, (unsigned)mc.pin_bck,
             (unsigned)M5.Mic.isRunning(), (unsigned)M5.Mic.isEnabled(),
             (unsigned)mc.input_channel, r14, r0d);
  } else {
    snprintf(micDiag, sizeof(micDiag), "MIC DISABLED B%u", (unsigned)M5.getBoard());
  }

  // 麦克风上电后约 1 秒预热期数据全零 —— 校准前先等一下
  delay(1100);

  runCalibration(false);
  bootAt = millis();   // 记录开机完成时刻，前 2.5 秒抑制鼠标输出消除漂移
}

/* ============================= loop ============================= */
void loop() {
  const uint32_t now = millis();
  const uint32_t t0 = micros();

  M5Cardputer.update();          // 按键 + G0
  handleKeys();
  handleCalibPush();             // 归中标定：按住 O/P 实时推鼠标
  if (M5Cardputer.BtnA.wasPressed() && !mouseDown && !refract) fire();

  updateAim(t0);
  updateMic(now);

  // 归中值微调后，空闲 600ms 自动补存（绕过 saveSettings 的 500ms 限流）
  if (centerDirty && (uint32_t)(millis() - lastCenterEdit) > 600) {
    centerDirty = false;
    lastSaveMs = 0;              // 强制保存
    saveSettings();
  }

  static uint32_t lastDraw = 0;
  if (now - lastDraw >= 33) {    // ~30fps 刷屏
    lastDraw = now;
    draw();
  }
}
