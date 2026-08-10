# Scripts

This folder contains the canonical scripts used to build and run the project.

## Frontend (React)

Build production bundle:

```bash
bash scripts/build_frontend.sh
```

Sync build into the ROS 2 package static app folder:

```bash
bash scripts/sync_frontend_to_ros.sh
```

Recommended combined flow:

```bash
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
```

## ROS 2 Workspace

Build ROS2 workspace (requires ROS2 + colcon):

```bash
bash scripts/build_ros.sh
```

Run UI backend (requires ROS2):

```bash
bash scripts/run_ui_backend.sh
```

For the Blockly `Voice Command` feature to work, set an LLM API key first —
see `ros2/src/openamr_ui_package/.env.example` and
[../ros2/src/openamr_ui_package/launch/README.md](../ros2/src/openamr_ui_package/launch/README.md).

## Docker Container

`container_entrypoint.sh` is used by the root `Dockerfile`. It sources ROS 2,
optionally rebuilds the frontend and ROS workspace when
`OPENAMR_REBUILD_ON_START=1` is set, sources the built workspace, and launches:

```bash
ros2 launch openamr_ui_bringup ui.launch.py
```

Normal users should start the container from the repository root:

```bash
docker compose up --build
```

## Feature-tour video

`create_asset_video.sh` creates a captioned 1920×1080 feature tour from the
current screenshots under `docs/assets/`. It requires FFmpeg and the DejaVu
Sans fonts. The generated MP4 includes a silent AAC track for broad playback
compatibility.

```bash
bash scripts/create_asset_video.sh
```

Default output:

```text
docs/assets/openamrobot_ui_feature_tour.mp4
```

Pass a different output path as the first argument when needed.

### Add narration

`create_narrated_video.sh` creates a synchronized English narration track and
adds it to the feature tour. It requires FFmpeg and eSpeak NG:

```bash
sudo apt install ffmpeg espeak-ng
bash scripts/create_asset_video.sh
bash scripts/create_narrated_video.sh
```

Default outputs:

```text
docs/assets/openamrobot_ui_feature_tour_narration.m4a
docs/assets/openamrobot_ui_feature_tour_with_audio.mp4
```

The original feature-tour MP4 is kept unchanged. You can optionally pass the
source video, narrated-video output, and audio-only output as three arguments.
The matching transcript is maintained in
`docs/assets/openamrobot_ui_feature_tour_transcript.md`.

### Add a recorded human voice-over

`add_feature_tour_voiceover.sh` replaces the generated narration with the
recorded 83.8-second English voice-over and adjusts each slide to follow its
natural sentence pauses:

```bash
bash scripts/add_feature_tour_voiceover.sh \
  docs/assets/openamrobot_ui_feature_tour.mp4 \
  /path/to/voiceover.mp3
```

It updates the same narrated-video and audio-only outputs listed above while
leaving the silent feature tour unchanged.
