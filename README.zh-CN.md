# RoboLab with RoboAPI

[English](README.md) | **简体中文**

通过 [PokeBot RoboAPI](https://developers.pokebot.com/docs/roboapi/latest)，在 [NVIDIA RoboLab](https://github.com/NVLabs/RoboLab) 基准中运行基于视觉的机器人策略。
本集成使用 Franka 机械臂、经过标定的 Robotiq 连杆机构和单个腕部鱼眼相机，沿用 RoboLab 的任务定义、回合执行流程和成功判定条件。

策略接收任务指令、腕部 RGB 图像、测得的法兰位姿和夹爪开口宽度，不接收物体状态或任务特权观测。
默认模型为 `poke-aurora-preview`，API 地址为 `https://api.pokebot.com/v1`，两者均可配置。

## 安装与运行

按照[上游安装指南](https://github.com/NVLabs/RoboLab/tree/b6cfcf6ccd2113c4cd1216d35b33024a7d5691da#installation)，安装 RoboLab 所需的 Isaac Sim 5.1 和 Isaac Lab 2.3.2 环境。
标定后的相机需要原生 `OmniLensDistortionFthetaAPI` 支持。

下载指定版本的 [RoboAPI 机器人配置资产包](https://github.com/huashanjian/roboapi-playground/releases/tag/roboapi-rig-v1.0.0)，并按照[下载与验证步骤](docs/roboapi/setup.md#download-the-rig-assets)，安装工具组件、相机标定文件和执行器标定文件。
将 [.env.example](.env.example) 复制为 `.env.local`，填写本地 Panda USD 文件的路径，并确保其引用的几何文件齐全。
凭据保存在配置指定的私有令牌文件中。已安装的资产和凭据均已排除在版本控制和 Docker 构建之外。

```bash
set -a
source .env.local
set +a
# 使用已安装 USD（pxr）和 NumPy 的 Python 环境。
python -m policies.roboapi.build_asset
# 不调用模型，先检查机械结构、控制与渲染。
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --preflight-only --output-folder-name roboapi_preflight
# 运行一个评测回合。
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --output-folder-name roboapi_single
```

详细说明见[配置与机器人要求](docs/roboapi/setup.md)、[协议与评测](docs/roboapi/evaluation.md)和[控制模型](docs/roboapi/control.md)。

## 测试

```bash
python -m pip install -r requirements-roboapi-tests.txt
scripts/test_roboapi.sh
```

这些测试无需 Isaac Sim 即可运行。安装 PyTorch 后，还会在 CPU 上测试夹爪动作缓冲区。
新安装的环境或新标定的资产仍需在仿真器中完成运行前检查。

## 许可证

本仓库保留 RoboLab 的上游历史和 NVIDIA 版权声明，详见 [LICENSE](LICENSE) 和[第三方声明](THIRD_PARTY_NOTICES.md)。
随附的机器人、工具和基准资产各自遵循其原有许可证。

## Star 趋势

<a href="https://www.star-history.com/?repos=huashanjian%2Froboapi-playground&amp;type=date&amp;legend=top-left">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;theme=dark&amp;legend=top-left" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;legend=top-left" />
    <img alt="huashanjian/roboapi-playground 的 GitHub Star 数量随时间变化曲线" src="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;legend=top-left" width="800" />
  </picture>
</a>
