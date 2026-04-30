---
sidebar_position: 1
slug: /k3/ai/speech
---

# 语音能力总览

## 1. 适用范围

本章介绍 SpacemiT Robot SDK 中与语音交互相关的基础能力，包括语音活动检测、声纹识别、语音识别和语音合成。它们可以单独集成，也可以组合成端到端语音链路：

```text
麦克风采集 -> VAD 断句 -> 声纹验证（可选） -> ASR 转写 -> LLM 推理 -> TTS 合成 -> 扬声器播放
```

语音模块主要面向 K3 机器人语音助手、离线语音识别、说话人验证、语音播报和 `omni_agent` 端到端对话应用。音频采集、播放、重采样和声源定位属于多媒体基础能力，见 [多媒体总览](../../06-系统与平台/6.3-多媒体/index.md)。

## 2. 文档关系

- 前置阅读：先了解 [多媒体 · audio](../../06-系统与平台/6.3-多媒体/6.3.3-audio.md)，确认录音、播放、设备索引和采样率配置。
- 组合应用：端到端语音对话见 [Agent](../4.5-Agent.md)，其中串联了 VAD、ASR、LLM、TTS、声纹和 MCP 工具调用。
- 仓库路径：
  - ASR：`components/model_zoo/asr`
  - TTS：`components/model_zoo/tts`
  - VAD：`components/model_zoo/vad`
  - 声纹：`components/model_zoo/voiceprint`
  - 语音对话应用：`application/native/omni_agent`

## 3. 阅读索引

建议按下表顺序阅读。只做语音播报时可以直接阅读 TTS；只做说话人验证时可以直接阅读声纹。

| 序号 | 文档 | 摘要 |
| --- | --- | --- |
| 1 | [VAD](4.1.3-VAD.md) | 使用 Silero VAD 检测语音开始和结束，为 ASR 断句、barge-in 和低功耗监听提供前置能力。 |
| 2 | [声纹](4.1.4-声纹.md) | 使用 CamP+ 提取 192 维 embedding，支持注册、识别和 1:1 验证。 |
| 3 | [ASR](4.1.1-ASR.md) | 使用 SenseVoice、Zipformer 或 Qwen3-ASR 将语音转成文本，支持文件识别和流式识别。 |
| 4 | [TTS](4.1.2-TTS.md) | 使用 Matcha-TTS 或 Kokoro 将文本合成为 WAV/PCM，支持文件合成和流式播放。 |
| 5 | [Agent](../4.5-Agent.md) | 组合音频、VAD、声纹、ASR、LLM、TTS 和 MCP，运行完整语音助手。 |
