# RoboLab with RoboAPI

[English](README.md) | **简体中文**

[![RoboAPI 离线检查状态](https://github.com/huashanjian/roboapi-playground/actions/workflows/roboapi.yml/badge.svg?branch=feat%2Froboapi-eval)](https://github.com/huashanjian/roboapi-playground/actions/workflows/roboapi.yml?query=branch%3Afeat%2Froboapi-eval)
[![机器人资产包版本](https://img.shields.io/github/v/release/huashanjian/roboapi-playground?label=rig%20assets)](https://github.com/huashanjian/roboapi-playground/releases/latest)
[![代码许可证](https://img.shields.io/github/license/huashanjian/roboapi-playground)](LICENSE)

**在 NVIDIA RoboLab 中运行和评测 RoboAPI 机器人策略。**

将 [PokeBot RoboAPI](https://developers.pokebot.com/docs/roboapi/latest) 接入 [NVIDIA RoboLab](https://github.com/NVLabs/RoboLab) 仿真基准，使用 Franka 机械臂、经过标定的 Robotiq 夹爪组件和腕部鱼眼相机执行任务。

**[运行第一个任务 →](#快速开始)**

[集成特点](#集成特点) · [文档导航](#文档导航) · [常见问题](#常见问题) · [测试](#测试)

## 集成特点

- **根据腕部观测评测策略。** 模型接收任务指令、单张腕部 RGB 图像、测得的法兰位姿和夹爪开口宽度，不接收物体状态或任务特权观测。
- **使用经过标定的机器人组件。** 集成 Franka 机械臂、Robotiq 连杆机构、原生鱼眼相机模型和经过标定的夹爪响应模型。
- **运行 RoboLab 的 120 个任务。** 沿用官方任务定义、回合执行流程和成功判定条件。当前配置将每回合仿真时长统一设为 120 秒，最多执行 90 个动作块，详见[评测条件](docs/roboapi/evaluation.md#evaluation-conditions)。

## 快速开始

### 1. 准备环境与资产

克隆本仓库，后续命令均在仓库根目录中使用 Bash 执行：

```bash
git clone --branch feat/roboapi-eval https://github.com/huashanjian/roboapi-playground.git
cd RoboLab
```

| 所需条件 | 准备方式 |
| --- | --- |
| Python 与仿真器 | 使用 Python 3.11、Isaac Sim 5.1 和 Isaac Lab 2.3.2.post1。按照[上游安装指南](https://github.com/NVLabs/RoboLab/tree/b6cfcf6ccd2113c4cd1216d35b33024a7d5691da#installation)为当前仓库安装环境，选择 `isaac51` 可选依赖组。 |
| 相机支持 | 仿真器须提供原生 `OmniLensDistortionFthetaAPI` 支持。 |
| 工具组件与标定文件 | 按照[资产下载说明](docs/roboapi/setup.md#download-the-rig-assets)，安装 [v1.0.0 机器人资产包](https://github.com/huashanjian/roboapi-playground/releases/tag/roboapi-rig-v1.0.0)。 |
| Panda 模型 | 准备本地 `panda_instanceable.usd` 及其引用的 `Materials/`、`Props/` 文件，保持相对目录结构。 |

### 2. 配置并构建机器人

创建本地配置文件：

```bash
cp -n .env.example .env.local
```

编辑 `.env.local`，将 `ROBOAPI_PANDA_USD` 设为本地 Panda 模型路径，将 `ROBOAPI_PYTHON` 设为仿真环境的 Python 可执行文件路径。构建命令需要该环境中已安装 USD（`pxr`）和 NumPy。机器人资产路径应与第 1 步安装的文件位置一致。

默认模型为 `poke-aurora-preview`，API 地址为 `https://api.pokebot.com/v1`，可通过 `ROBOAPI_MODEL` 和 `ROBOAPI_BASE_URL` 修改。在运行模型评测前，将 API 密钥保存到 `ROBOAPI_TOKEN_FILE` 指定的文件中，文件权限设为 `0600`，父目录权限设为 `0700`。

加载配置并构建组装后的机器人：

```bash
set -a
source .env.local
set +a
"$ROBOAPI_PYTHON" -m policies.roboapi.build_asset
```

### 3. 检查仿真环境

检查机械结构、控制与渲染，此步骤无需 API 密钥，也不调用模型：

```bash
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --preflight-only --output-folder-name roboapi_preflight
```

检查报告写入 `output/roboapi_preflight/preflight.json`，腕部相机录像保存在同一目录。若报告中有未通过的检查，先解决对应问题，再运行模型评测。

### 4. 运行一个评测回合

保存好 API 密钥后，执行：

```bash
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --output-folder-name roboapi_single
```

运行产物保存在 `output/roboapi_single/` 下。本地配置、已安装的资产、凭据和评测产物均已排除在版本控制和 Docker 构建之外。

## 文档导航

| 要完成的工作 | 对应文档 |
| --- | --- |
| 配置环境、资产或相机 | [安装与标定](docs/roboapi/setup.md) |
| 理解坐标系、机械臂控制与夹爪响应 | [控制模型](docs/roboapi/control.md) |
| 理解观测、动作和评测条件 | [协议与评测](docs/roboapi/evaluation.md) |
| 评测全部 120 个任务 | [完整评测工具](docs/roboapi/evaluation.md#complete-evaluation-tools) |
| 修改模型、API 地址或本地路径 | [配置变量](.env.example) |

## 常见问题

### 需要实体机器人吗？

评测在 Isaac Sim 中运行，使用安装说明中的本地 Panda 模型、经过标定的工具组件和 RoboLab 任务资产。

### 机器人资产包包含哪些文件？

资产包提供工具组件、相机标定文件和执行器标定文件。Panda 几何文件、基准场景和仿真器需要另行准备，详见[所需资产](docs/roboapi/setup.md#required-assets)。

### 调用模型前可以先检查什么？

[离线测试](#测试)无需 Isaac Sim，也不调用模型。上方的仿真环境检查会检查组装后的机器人和相机渲染，无需 API 密钥或模型调用，但需要已安装的仿真器与资产。

### 可以更换模型或 API 地址吗？

可以。在 `.env.local` 中设置 `ROBOAPI_MODEL` 和 `ROBOAPI_BASE_URL`，启动前重新加载配置。API 服务须实现 [RoboAPI 模型接口协议](https://developers.pokebot.com/docs/roboapi/latest/model-contract)。

### 如何运行完整评测？

按照[完整评测工具说明](docs/roboapi/evaluation.md#complete-evaluation-tools)准备任务目录、启动工作进程并汇总结果。默认协议对 120 个任务分别使用随机种子 0 至 9，共运行 1,200 个任务与种子组合。

### 报告结果时需要注明哪些条件？

注明 Franka/Robotiq 组件、腕部相机配置、统一的 120 秒仿真时长，以及每回合最多 90 个动作块。这些条件与上游 RoboLab 的标准 Panda 夹爪、相机和任务时限不同。仓库尚未附带评测成绩或对比结果。

## 测试

在独立的 Python 环境中运行离线检查：

```bash
python -m pip install -r requirements-roboapi-tests.txt
scripts/test_roboapi.sh
```

这些测试无需 Isaac Sim 即可运行。安装 PyTorch 后，还会在 CPU 上测试夹爪动作缓冲区。新安装的环境或新标定的资产仍需在仿真器中完成运行前检查。

## 许可证

代码采用 [Apache 2.0 许可证](LICENSE)。本仓库保留 RoboLab 的上游历史和 NVIDIA 版权声明。随附的机器人、工具和基准资产各自遵循其原有许可证，详见[第三方声明](THIRD_PARTY_NOTICES.md)。

## Star 趋势

<a href="https://www.star-history.com/?repos=huashanjian%2Froboapi-playground&amp;type=date&amp;legend=top-left">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;theme=dark&amp;legend=top-left" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;legend=top-left" />
    <img alt="huashanjian/roboapi-playground 的 GitHub Star 数量随时间变化曲线" src="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;legend=top-left" width="800" />
  </picture>
</a>
