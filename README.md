# RoboCasa 仿真控制插件

在 DeepSeek Harness 里用对话操控 RoboCasa 厨房仿真：开场景、看相机、移动夹爪、检查任务是否成功。

## 结构

```
robocasa-control/
├── index.js          # Host 插件：注册 robocasa_* 工具，管理 Python 子进程
├── worker.py         # 常驻仿真进程，JSON-RPC over stdio
├── cordis.patch.yml  # 注入到 profile 的 Loader 配置
├── locale/{en,zh}.json
└── icon.svg
```

```
DSH Agent
   │ tool calls
   ▼
index.js（Node）── stdin/stdout 上的 JSON 行协议 ──▶ worker.py（RoboCasa / MuJoCo）
```

仿真状态常驻在 worker 进程里，因此多次工具调用之间物理状态是连续的。

## 工具

| 工具 | 作用 |
|---|---|
| `robocasa_doctor` | 环境体检：解释器、依赖版本、资产、磁盘、渲染后端；`deep=true` 会真建一个场景并渲染一帧 |
| `robocasa_open` | 打开场景（task / seed / layout / style / camera / 分辨率），并创建本次运行的目录与 manifest |
| `robocasa_look` | 截当前相机画面，返回图片 + 末端位姿 |
| `robocasa_locate` | 按名字查物体或夹具：坐标、尺寸、长轴朝向、建议抓取 yaw、夹具内部区域中心 |
| `robocasa_move` | 闭环移动到世界坐标 (x, y, z)，可同时指定绝对朝向（`yaw_deg` 或四元数 `quat`） |
| `robocasa_rotate` | 闭环相对旋转（度数），用于抓取前对齐夹爪 |
| `robocasa_act` | 原始控制器量：末端平移、底盘 `forward/side/yaw`、躯干 `torso` 升降 |
| `robocasa_grasp` | 多假设抓取：按 yaw 候选依次尝试，用「物体是否跟随」验证并返回结果 |
| `robocasa_status` | 步数、末端位姿、任务成功、物体世界坐标与包围盒尺寸 |
| `robocasa_reset` | 新开一局（会重新随机化场景），并把 restart 锚点移到新一局开头 |
| `robocasa_restart` | 回到当前这一局的开头（open 或最近一次 reset 时的快照），逐位还原 |
| `robocasa_close` | 关闭仿真，释放进程 |

## 安装

见 [INSTALL.md](INSTALL.md)：前置条件、仿真环境安装、插件安装、体检、两种安装模式（用户自主 / AI 协助）与常见问题。

## 运行目录与 manifest

每次 `open` 都会创建一个运行目录（默认 `<storageRoot>/<task>-seed<N>-<本地时间>`，可用 `dir` 参数指定）：

```
<run dir>/
├── manifest.json     # 场景、版本、统计、事件
└── frames/           # 截图
```

`manifest.json` 记录：

- `scene`：task / seed / layout / style / camera / 分辨率
- `environment`：python / numpy / mujoco / robosuite / robocasa 版本（来自真实运行进程）
- `stats`：步数、截图数、move/act/rotate/restart/reset 次数
- `events`：时间线（open → screenshot → move → … → close），上限 200 条

默认存储根由 `storageRoot` 配置项决定，运行时可用 `robocasa_open(dir=...)` 覆盖。

## 环境体检

`robocasa_doctor` 会逐项输出 `✅ / ⚠️ / ❌` 与修复提示：

| 检查 | 失败时的提示 |
|---|---|
| Python >= 3.10 | 升级解释器 |
| numpy / mujoco / robosuite / robocasa import | 对应 pip 安装命令 |
| 版本是否匹配锁定值 | 精确版本命令 |
| `robocasa/models/assets` 及关键子目录 | `download_kitchen_assets` |
| 剩余磁盘 | 提示资产可能占用数十 GB |
| EGL / `MUJOCO_GL` | 安装 `libegl1` 或改用 osmesa |

`deep=true` 会额外建一个 64×64 的场景并渲染一帧，约 6 秒，用于验证整条链路而不只是 import。

## 坐标系约定

- `robocasa_move` / `robocasa_status` 使用**世界坐标**（米）。
- OSC_POSE 控制器的输入参考系是 **base**，`worker.py` 内部用 `origin_ori` 把世界增量旋到 base 系，因此对话里只需要关心世界坐标。
- `robocasa_act` 的 `dx/dy/dz` 是**归一化控制器量**（1.0 ≈ 5 cm/步），方向已从世界系旋到 base 系。

## 旋转

- `robocasa_rotate`：相对旋转，单位度，闭环，默认容差 0.03 rad（约 1.7°）。
- `robocasa_move(yaw_deg=...)`：绝对偏航角，保持当前 pitch/roll。
- `robocasa_move(quat=[x,y,z,w])`：绝对姿态，优先于 `yaw_deg`。

注意：OSC 姿态控制有跟踪滞后，**开环按步喂增量不可靠**（实测 3 步要求 30° 只到 5°），
所以旋转一律走闭环。

## 配置（cordis.patch.yml）

```yaml
- insert:
    - id: robocasa-control
      name: '@local/robocasa-control'
      config:
        pythonPath: /home/zhu/miniconda3/envs/robocasa/bin/python
        shotsDir: /home/zhu/桌面/harness-skills-demo/robocasa-shots
```

可选：`workerScript`、`cwd`。

## 感知层（locate）

`robocasa_locate` 是感知层的第一个接口，返回结构化几何而不是像素：

- 物体：位置、包围盒尺寸、`top_z`、`pre_grasp`、`grasp`、`long-axis yaw`
- 夹具：位置、尺寸、`interior` 各层区域中心（柜内、水槽盆）
- 每条结果都带 `source`：`ground_truth`（默认，读仿真状态）或 `vision`（开放词表检测器）

### vision 后端

`robocasa_locate(query="bread", backend="vision")` 会在当前相机画面上跑
`IDEA-Research/grounding-dino-tiny`，返回**多个候选**：

```
"bread" score=0.663 box=[109.6, 190.7, 137.2, 217.1]
  world: x=2.2341 y=-0.5586 z=0.9769 (depth 1.15 m)
```

每个候选带 2D 框、分数、像素、深度和反投影出的世界坐标。设计上**不自动替调用方选实例**——
同类别可能有干扰物，选错比选不到更危险。

依赖：`pip install transformers`（约 50 MB）+ 模型权重（约 700 MB，缓存在 `~/.cache/huggingface`）。
worker 会清理宿主进程里的 socks 代理并使用本地缓存（`HF_HUB_OFFLINE=1`）。

已知限制：本机 torch 2.7.1+cu126 与 RTX 5060（sm_120）不匹配，推理走 CPU，单帧约 4 秒；
候选点的 z 是**可见表面**而非物体中心，直接用它做抓取会偏低约 3 cm。

**抓取 yaw 的由来**：实测夹爪闭合轴 ≈ 末端 yaw + 90.5°。要让夹爪横跨物体短轴，
末端 yaw 应对齐物体长轴朝向。`locate` 直接给出这个角度。

平行夹爪在 yaw 与 yaw+180° 下等价，`robocasa_move` 会自动选转动量更小的那个。

## 多假设抓取

单目朝向估计有约 30° 的系统误差，所以 `robocasa_grasp` 不赌一个角度，而是依次尝试
`yaw_deg` 和 `offsets`（默认 `[0, -30, 30]`）：每次下探 → 合爪 → 抬升 10 cm →
检查 `verify` 物体是否跟随，跟上了就返回成功。

实测：视觉估计 -34.2°（真值 -66.5°，差 32°），**第一次假设就抓起来了**，物体抬升 6.7 cm，
全程 12.4 秒。验证逻辑在抬升失败时会让位给下一个候选。

**已知偏差**：视觉长轴估计在当前机位下有约 **-30° 的系统偏差**。所以 `yaw_deg=视觉值, offsets=[-30]`
是最稳的一次命中；默认的三次自动搜索在两次失败后手臂姿态会卡住，尚未解决。

## 性能与 GPU

在 RTX 5060 (sm_120) 上实测（`bench_sim.py` / `time_vision`）：

| 环节 | 耗时 |
|---|---:|
| `create_env` + reset | 5.3 s（一次性） |
| 物理单步 | 16.3 ms |
| 离屏渲染 320×320 | 0.3 ms |
| PNG 编码 320×320 | 8.8 ms |
| `look()` 全流程 | 9.2 ms |
| 视觉检测（CPU） | 2261 ms |
| 视觉检测（GPU fp32） | 256 ms |
| **视觉检测（GPU fp16，热）** | **93 ms** |
| 视觉检测首次调用（含加载） | 2.4 s |

**RTX 50 系（sm_120）必须用 CUDA 12.8 的 PyTorch 构建**，`cu126` 的 wheel 只编到 sm_90：

```bash
pip install --index-url https://download.pytorch.org/whl/cu128 torch==2.7.1+cu128 torchvision==0.22.1+cu128
```

worker 会自动选择 `cuda` 并切到 fp16；没有 GPU 时退回 CPU。

## 自由相机

`robocasa_look` 的 `camera` 参数除了命名相机，还接受自由视角：

```
free                                   # 默认第三人称（距离 2.8、方位 200°、俯角 -32°）
free:az=45,el=-40,dist=2.8             # 任意角度
free:lx=0.3,ly=-1.2,lz=1.0,dist=1.5    # 指定观察点 + 距离
```

参数含义：`az` 绕竖直轴方位角、`el` 俯仰角（负值向下）、`dist` 距离、`lx/ly/lz` 观察点世界坐标
（省略时看向末端执行器）。

实现细节：robosuite 的 `sim.render(camera_name=None)` **不会**自己切成自由相机——它只在传入
`camera_id` 时改 `cam.type`。所以 worker 显式设置 `mjCAMERA_FREE` 再渲染。

**限制**：自由相机没有命名相机的内参矩阵，视觉检测（`backend="vision"`）目前只支持命名相机。

## 预览窗口的两种模式

| 模式 | 开关 | 效果 |
|---|---|---|
| 肩载视角（默认） | 删除 `.preview_free` | 跟着机器人左肩 |
| **第三人称全景** | `touch .preview_free` | 看整个厨房 + 机器人 |

窗口建议用 systemd 用户服务启动，避免被宿主进程组回收：

```bash
systemd-run --user --collect --unit=robocasa-viewer --setenv=DISPLAY=:1 \
  /path/to/python /path/to/viewer.py
```

## 物体尺寸

`open` 与 `status` 会同时给出物体的**世界坐标**与**包围盒尺寸**（米）。尺寸用于决定抓取下探深度与夹爪朝向，
不要假设所有物体都像球一样以中心为原点。

## 快照语义

- `robocasa_restart` 在内存里保存一份 MuJoCo 状态快照，只在本进程内有效；进程重启即失效。
- 快照在 `open` 和每次 `reset` 时自动捕获，因此 `reset` 之后 `restart` 回到的是新一局的开头。
- 实测：移动 13 步后 restart，位置 / 物体坐标 / 四元数与开局差值为 0。

## 已知限制

- 抓取依赖夹爪闭合时刚好包住物体；`move` 到达残差 > 3 cm 时说明目标超出臂展，需要先用 `robocasa_act` 的 `forward/side` 移动底盘。
- 任务成功判定调用环境的 `_check_success()`，不同任务语义不同。
- 尚未接入 ROS 2；接真机时把 worker 的 transport 换成 rclpy，工具接口不变。
