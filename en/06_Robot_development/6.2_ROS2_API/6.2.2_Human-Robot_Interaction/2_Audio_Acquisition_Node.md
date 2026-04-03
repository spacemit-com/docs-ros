sidebar_position: 2

# Audio Capture Node

## Supported Hardware and Protocols

- **Hardware Interfaces**：
  - USB Microphone
  - USB Sound Card
  - Onboard I²S Microphone (not present on all boards)
- **Protocols**：
  - ALSA (Advanced Linux Sound Architecture)
  - PulseAudio(Optional; primarily used for development/debugging)

## Software and Hardware Environment

- **Operating System**：ROS2_LXQT System
- **Recommended Hardware**：SpacemiT **MUSE Pi Pro** Development Board
- **Dependency Tools（Commonly used during development/debugging）**：
  - `arecord` （ALSA command-line recording tool）
  - `aplay` （ALSA command-line playback tool）
  - `alsamixer` （Command-line volume control）

## System Dependencies

```
sudo apt install python3-pyaudio python3-scipy libfftw3-dev
```

## Microphone Connection Diagram

- Plug the USB microphone into a USB port on the **MUSE Pi Pro**.

![](images/mic_connect.png)

## Adding to the Audio Group

Ordinary users may not have access to audio devices by default and need to join the * * audio * * user group：

```bash
sudo usermod -aG audio $USER
```

After execution, it is necessary to log in to the terminal again to take effect.

## Identify Audio Device ID

Use `arecord` to query the audio devices available on the system:

```bash
arecord -l
```

Example Output:

```
bianbu@bianbu:~$ arecord -l
**** CAPTURE Hardware Device List ****
card 1: sndes8326 [snd-es8326], device 0: i2s0-dai-ES8326 HiFi ES8326 HiFi-0 []
  Sub device: 1/1
  Sub device #0: subdevice #0
card 2: Device [USB PnP Sound Device], device 0: USB Audio [USB Audio]
  Sub device: 1/1
  Sub device #0: subdevice #0
```

Explanation:

- `card 2` indicates sound card number 2.
- `device 0` indicates device number 0.
- The full device identifier is **hw:2,0**

## Verifying Supported Frequencies

```
arecord -D hw:2,0 --dump-hw-params
```

**Example output**

![](./images/arecord_out1.png)

Interpretation of Parameters

```
--------------------
ACCESS:  MMAP_INTERLEAVED RW_INTERLEAVED #Data access method，RW_INTERLEAVED：The common method, where data from multiple channels is stored in an interleaved fashion (e.g., L, R, L, R...);MMAP_INTERLEAVED：Memory mapping method, which has better performance but is more complex to use.
FORMAT:  S16_LE             # S16_LE：16-bit signed integer, little-endian byte order (commonly used).
SUBFORMAT:  STD            # Standard PCM (Pulse Code Modulation), a conventional audio format.
SAMPLE_BITS: 16             # The number of bits occupied by each sample point (here, 16 bits).
FRAME_BITS: 16              # The number of bits occupied by each frame (since this is a mono channel, this value is the same as SAMPLE_BITS—16 bits).
CHANNELS: 1                 # Number of audio channels.
RATE: [44100 48000]         # Range of supported sample rates; this indicates support for 44.1 kHz and 48 kHz.
PERIOD_TIME: [1000 1000000] # Duration of each period (Unit: microseconds [us]).
PERIOD_SIZE: [45 48000]     # Number of sample points per period (minimum: 45 points; maximum: 48,000 points).
PERIOD_BYTES: [90 96000]    # Number of bytes corresponding to each period; since this is 16-bit mono audio, this value equals PERIOD_SIZE × 2.
PERIODS: [2 1024]           # The number of periods comprising the ring buffer; minimum 2, maximum 1024.
BUFFER_TIME: [1875 2000000] # Total buffer duration (in microseconds); here, this indicates a minimum buffer time of 1.875 ms and a maximum of 2000 ms.
BUFFER_SIZE: [90 96000]     # The number of sample points the buffer can accommodate.
BUFFER_BYTES: [180 192000]  # The buffer size in bytes; since the configuration is for mono, 16-bit audio, this value equals BUFFER_SIZE × 2.
TICK_TIME: ALL              # Timing granularity; ALL indicates no restrictions (typically determined by the driver).
--------------------
```

**The sampling rate selected here is 48000**，44100 is also acceptable.

Test Recording:

```bash
arecord -D hw:2,0 -f S16_LE -r 48000 -c 1 test.wav
```

## Launch Command

```
ros2 launch rdk_hri recorder.launch.py device_index:=2 sample_rate:=48000
```

**Terminal Output：**

![](./images/ros2out1.png)

This will release audio data streams to the topic of '/audio/law'

**Check Topic Publish Rate：**

`ros2 topic hz /audio/raw`

![](./images/ros2out2.png)

Theoretical Publish Rate = sample_rate / frame_size = 48000 / 512 = 93.75

If the actual publish rate is close to the theoretical rate, it indicates that audio acquisition is functioning correctly.

## Parameter Details

|    Parameter Name    |  Type  |   Default Value   |            Candidate Values/Range             |     Description     |
| :----------: | :----: | :--------: | :---------------------------------: | :----------: |
| device_index |  int   |     0      |               Positive integers                | Audio device index |
| sample_rate  |  int   |   48000    |            Valid sample rates             |  Audio sample rate  |
|   channels   |  int   |     1      |                1、2                 |    Number of channels    |
|  frame_size  |  int   |    512     | 256、512、1024、2^n ...(Not recommended to change) | Samples per frame |
|    format    | string |    pcm     |               float32               | Sample data format |
|  pub_topic   | string | /audio/raw |               custom                | Topic name for publishing |
