# 安装手册 / Installation

本插件依赖一个能跑 RoboCasa 的 Python 环境。**插件的代码很少，难的是环境**——所以安装分两步：先装环境，再装插件。

---

## 0. 前置条件

| 项 | 要求 | 怎么查 |
|---|---|---|
| 操作系统 | Linux（需要 headless 渲染） | `uname -s` |
| Python | >= 3.10（推荐 3.11） | `python3 -V` |
| 磁盘 | 至少 **40 GB** 可用（资产约 24 GB） | `df -h ~` |
| 图形库 | `libEGL`（无显示器时用） | `ldconfig -p \| grep libEGL` |
| 网络 | 能访问 GitHub 与 PyPI | — |

已知的版本锁定（`robocasa/setup.py`）：

```
numpy==2.2.5  mujoco==3.3.1  robosuite==1.5.2
```

---

## 1. 安装仿真环境

```bash
# 1.1 建环境
conda create -n robocasa python=3.11 -y
conda activate robocasa

# 1.2 robosuite 与 robocasa（建议都用源码 + editable）
git clone https://github.com/ARISE-Initiative/robosuite.git
cd robosuite && pip install -e . && cd ..

git clone https://github.com/robocasa/robocasa.git
cd robocasa && pip install -e . && cd ..

# 1.3 厨房资产（体积大，见下方提示）
python -m robocasa.scripts.download_kitchen_assets
```

> **关于下载**：资产包解压后约 24 GB，下载耗时取决于网络。中途失败不用重来，脚本支持续传；解压后确认 `robocasa/models/assets/{objects,fixtures,textures,scenes}` 四个目录都在。

---

## 2. 安装插件

插件是一个 bundle，三种安装方式任选：

**从 GitHub（推荐）**

```bash
pnpm add github:only-one-over/robocasa-control
```

**从 Release 压缩包**

下载 <https://github.com/only-one-over/robocasa-control/releases/latest> 里的
`robocasa-control-1.0.0.tgz`，在 DSH 里安装该文件。

**从本地目录（开发时）**

```bash
dsh plugin add /path/to/robocasa-control
```

也可以由智能体在对话里执行（需要 `plugin_manager` 权限）：

```
plugin_manager install_bundle github:only-one-over/robocasa-control
```

装好后工具列表里应出现：

```
robocasa_doctor  open   locate  look    move   rotate  act
grasp            drop   plan    status  reset  restart close  preview
```

> npm 上的同名包已占位但尚未批准发布（等待账号 2FA），在那之前请用上面的方式。

---

## 3. 安装后必做：体检

**在打开任何场景之前**，先跑：

```
robocasa_doctor           # 快速检查，约 0.5 秒
robocasa_doctor deep=true # 额外真建一个场景并渲染一帧，约 6 秒
```

体检会逐项输出 `✅ / ⚠️ / ❌` 并给出修复命令。**只有 `❌` 会让体检失败**，`⚠️` 表示能用但偏离推荐配置。

| 检查项 | 失败通常意味着 |
|---|---|
| `python` | 解释器版本太旧 |
| `import:numpy/mujoco/robosuite/robocasa` | 包没装、或装到了另一个解释器里 |
| `version:*` | 版本漂移，行为可能和别人不一致 |
| `assets` | 资产没下全 |
| `disk` | 空间不足，跑到一半会失败 |
| `render` | 无 EGL，出黑图 |
| `deep:render` | 前置都通过但实际渲染不了（最难查的一类） |

---

## 4. 两种安装方式，由用户选择

| | 用户自主安装 | AI 协助安装 |
|---|---|---|
| 谁执行命令 | 用户 | 智能体 |
| 谁判断成功 | 用户跑 doctor | 智能体每步后跑 doctor |
| 适合 | 网络受限、需要审计、多人共用机器 | 单机自用、想省事 |
| 风险 | 无 | AI 可能装错解释器或漏装资产 |

**规则约束（给智能体）**：

1. 动手前先跑一次 `robocasa_doctor`，把当前状态告诉用户。
2. 告知预计下载量（约 24 GB 资产）与耗时，**取得用户同意后再执行**。
3. 每完成一步（建环境 / 装包 / 下资产）立刻重跑 doctor，不要连做多步再验证。
4. 体检出现 `❌` 时停止后续步骤，先修当前项。
5. 不要把 `⚠️` 当成失败去反复重装——那只会把环境搞坏。

---

## 5. 配置

`cordis.patch.yml` 里可改：

```yaml
- insert:
    - id: robocasa-control
      name: '@local/robocasa-control'
      config:
        pythonPath: /home/zhu/miniconda3/envs/robocasa/bin/python
        storageRoot: /home/zhu/桌面/harness-skills-demo/robocasa-runs
```

- `pythonPath`：**必须是 robocasa 环境的解释器**，不是系统 `python3`
- `storageRoot`：运行目录的根；每次 `open` 在其下建 `<task>-seed<N>-<时间>`，内含 `manifest.json` 与 `frames/`。
  单次运行可用 `robocasa_open(dir=...)` 覆盖
- 可选：`workerScript`、`cwd`

改完配置需要重新加载插件（禁用再启用）才生效。

---

## 6. 常见问题

**`no simulation open`**
没有先 `robocasa_open`，或 worker 崩溃过（错误信息里会写）。重新 `open` 即可。

**`The simulation worker process died ...`**
Python 侧崩溃了，错误信息附带最后几行 stderr。对照它修环境，而不是反复重开。

**渲染出来是黑图**
`MUJOCO_GL` 没设或 EGL 不可用。`export MUJOCO_GL=egl`，或装 `libegl1`。

**CUDA 报 `no kernel image is available for execution on the device`**
PyTorch wheel 没编进你这张显卡的架构。RTX 50 系（sm_120）需要 CUDA 12.8 构建：

```bash
pip install --index-url https://download.pytorch.org/whl/cu128 torch==2.7.1+cu128 torchvision==0.22.1+cu128
```

换完用 `python -c "import torch; print(torch.cuda.get_arch_list())"` 确认列表里有 `sm_120`。
（torch 在 robocasa 环境里是叶子依赖，换它不影响 robosuite/mujoco。）

**体检说 import 失败，但终端里 `python -c "import robocasa"` 成功**
解释器不是同一个。用 `which python` 和体检输出里的 `Interpreter:` 对照，改 `pythonPath`。
