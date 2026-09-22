---
sidebar_position: 1
slug: /k3/ai/speech
---

# 4.1 Speech

## 1. Scope

This chapter covers the voice-interaction capabilities in the SpacemiT Robot SDK: voice activity detection, voiceprint recognition, speech recognition, and speech synthesis. You can integrate each module on its own or combine them into an end-to-end speech pipeline:

```text
Microphone capture -> VAD segmentation -> Voiceprint verification (optional) -> ASR transcription -> LLM inference -> TTS synthesis -> Speaker playback
```

The speech modules are primarily used for K3 robot voice assistants, offline speech recognition, speaker verification, voice announcements, and the `omni_agent` end-to-end dialogue application. Audio capture, playback, resampling, and sound-source localization are multimedia foundation capabilities. See [Multimedia Overview](../../06-系统与平台/6.3-多媒体/index.md).

## 2. Related Documents

- Prerequisite reading: Start with [Multimedia audio](../../06-系统与平台/6.3-多媒体/6.3.3-audio.md) to confirm the recording, playback, device-index, and sample-rate configuration.
- Combined application: End-to-end voice dialogue is covered in [4.5 Agent](../4.5-Agent.md), which chains together VAD, ASR, LLM, TTS, voiceprint, and MCP tool calling.
- Repository paths:
  - ASR: `components/model_zoo/asr`
  - TTS: `components/model_zoo/tts`
  - VAD: `components/model_zoo/vad`
  - Voiceprint: `components/model_zoo/voiceprint`
  - Voice dialogue application: `application/native/omni_agent`

## 3. Reading Guide

Read the documents in the order shown in the table below. If you only need voice announcements, go straight to [4.1.2 TTS](4.1.2-TTS.md). If you only need speaker verification, go straight to [4.1.4 Voiceprint](4.1.4-声纹.md).

| No. | Document | Summary |
| --- | --- | --- |
| 1 | [4.1.3 VAD](4.1.3-VAD.md) | Use Silero VAD to detect the start and end of speech, providing the front-end capability for ASR segmentation, barge-in, and low-power listening. |
| 2 | [4.1.4 Voiceprint](4.1.4-声纹.md) | Use CamP+ to extract a 192-dimensional embedding, with support for enrollment, identification, and 1:1 verification. |
| 3 | [4.1.1 ASR](4.1.1-ASR.md) | Transcribe speech with SenseVoice, Zipformer, Qwen3-ASR, Fun-ASR Nano, or Gemma4 ASR. Gemma4 ASR can also transcribe foreign-language speech into English. |
| 4 | [4.1.2 TTS](4.1.2-TTS.md) | Synthesize text into WAV/PCM with Matcha-TTS or Kokoro, with support for file-based synthesis and streaming playback. |
| 5 | [4.5 Agent](../4.5-Agent.md) | Combine audio, VAD, voiceprint, ASR, LLM, TTS, and MCP to run a complete voice assistant. |
