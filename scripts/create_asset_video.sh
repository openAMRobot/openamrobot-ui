#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET_DIR="${REPO_ROOT}/docs/assets"
OUTPUT_PATH="${1:-${ASSET_DIR}/openamrobot_ui_feature_tour.mp4}"
FONT_REGULAR="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FPS=30
SLIDE_SECONDS=3.5
TITLE_SECONDS=4

for command_name in ffmpeg ffprobe; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: ${command_name} is required."
    exit 1
  fi
done

for font_path in "${FONT_REGULAR}" "${FONT_BOLD}"; do
  if [ ! -f "${font_path}" ]; then
    echo "ERROR: required font not found: ${font_path}"
    exit 1
  fi
done

VIDEO_TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${VIDEO_TMP_DIR}"' EXIT
CONCAT_LIST="${VIDEO_TMP_DIR}/segments.txt"
: >"${CONCAT_LIST}"
SEGMENT_INDEX=0

append_segment() {
  local segment_path="$1"
  printf "file '%s'\n" "${segment_path}" >>"${CONCAT_LIST}"
}

make_card() {
  local heading="$1"
  local subheading="$2"
  local output_file
  output_file="$(printf "%s/segment-%02d.mp4" "${VIDEO_TMP_DIR}" "${SEGMENT_INDEX}")"

  ffmpeg -hide_banner -loglevel error \
    -f lavfi -i "color=c=0x07111f:s=1920x1080:r=${FPS}:d=${TITLE_SECONDS}" \
    -vf "drawbox=x=0:y=0:w=1920:h=14:color=0x39bdf8:t=fill,
         drawtext=fontfile=${FONT_BOLD}:text='${heading}':fontcolor=white:fontsize=72:x=(w-text_w)/2:y=400,
         drawtext=fontfile=${FONT_REGULAR}:text='${subheading}':fontcolor=0xb9c8dc:fontsize=32:x=(w-text_w)/2:y=510,
         drawtext=fontfile=${FONT_REGULAR}:text='openAMRobot':fontcolor=0x39bdf8:fontsize=25:x=(w-text_w)/2:y=940,
         fade=t=in:st=0:d=0.6,fade=t=out:st=3.4:d=0.6" \
    -an -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -r "${FPS}" \
    -movflags +faststart "${output_file}"

  append_segment "${output_file}"
  SEGMENT_INDEX=$((SEGMENT_INDEX + 1))
}

make_slide() {
  local relative_image="$1"
  local heading="$2"
  local caption="$3"
  local image_path="${ASSET_DIR}/${relative_image}"
  local output_file
  output_file="$(printf "%s/segment-%02d.mp4" "${VIDEO_TMP_DIR}" "${SEGMENT_INDEX}")"

  if [ ! -f "${image_path}" ]; then
    echo "ERROR: screenshot not found: ${image_path}"
    exit 1
  fi

  ffmpeg -hide_banner -loglevel error \
    -loop 1 -framerate "${FPS}" -t "${SLIDE_SECONDS}" -i "${image_path}" \
    -vf "scale=1920:1080:force_original_aspect_ratio=decrease,
         pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=0x07111f,
         zoompan=z='min(zoom+0.00012,1.018)':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':d=1:s=1920x1080:fps=${FPS},
         drawbox=x=0:y=0:w=1920:h=14:color=0x39bdf8:t=fill,
         drawbox=x=0:y=885:w=1920:h=195:color=0x07111f@0.92:t=fill,
         drawtext=fontfile=${FONT_BOLD}:text='${heading}':fontcolor=white:fontsize=42:x=70:y=915,
         drawtext=fontfile=${FONT_REGULAR}:text='${caption}':fontcolor=0xc8d6e8:fontsize=27:x=70:y=980,
         fade=t=in:st=0:d=0.45,fade=t=out:st=3.05:d=0.45" \
    -an -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -r "${FPS}" \
    -movflags +faststart "${output_file}"

  append_segment "${output_file}"
  SEGMENT_INDEX=$((SEGMENT_INDEX + 1))
}

make_card \
  "OpenAMRobot UI" \
  "One browser dashboard for operating, automating, and understanding your robot"

make_slide "map/map.png" \
  "Map and Manual Control" \
  "See the robot, send goals, drive manually, manage waypoints, and dock"
make_slide "Routes/routes.png" \
  "Reusable Routes" \
  "Create and edit waypoint sequences for the active map"
make_slide "Maps/Maps.png" \
  "Map Management" \
  "Build, save, switch, rename, and organize maps"
make_slide "Programms/Blockly.png" \
  "Visual Robot Programs" \
  "Build validated navigation and motion routines with Blockly"
make_slide "Scheduler/Scheduler.png" \
  "Scheduling" \
  "Trigger routine browser-side robot actions at selected times"
make_slide "missions/missions.png" \
  "Multi-step Missions" \
  "Combine waypoints, waits, docking, and reusable actions"
make_slide "Status/status.png" \
  "Live Status" \
  "Monitor camera, battery, position, velocity, and system health"
make_slide "robot description/image.png" \
  "Robot Description" \
  "Explore the URDF model, kinematic tree, joints, and live overlays"
make_slide "devices/devices.png" \
  "External Devices" \
  "Register USB, CAN, network, and Raspberry Pi hardware"
make_slide "health/health.png" \
  "System Health Centre" \
  "Understand readiness, lifecycle state, devices, battery, and faults"
make_slide "metrics/metrics.png" \
  "Metrics" \
  "Track distance, uptime, navigation outcomes, and docking performance"
make_slide "recordings/recordings.png" \
  "Record and Replay" \
  "Capture rosbag sessions for debugging, demonstrations, and lessons"
make_slide "events/evnets.png" \
  "Event Timeline" \
  "Review and export navigation, docking, battery, and safety events"
make_slide "console/console.png" \
  "ROS Console" \
  "Inspect rosout messages and echo live topics from the browser"
make_slide "parameters/parameters.png" \
  "Runtime Parameters" \
  "Read and update selected Nav2 parameters on running nodes"
make_slide "fleet/fleet.png" \
  "Fleet Profiles" \
  "Maintain a robot roster and choose which robot this browser controls"

make_card \
  "Explore safely in Demo Mode" \
  "No robot required — then self-host the full UI when you are ready for ROS 2"

VIDEO_ONLY="${VIDEO_TMP_DIR}/video-only.mp4"
ffmpeg -hide_banner -loglevel error \
  -f concat -safe 0 -i "${CONCAT_LIST}" \
  -c copy -movflags +faststart "${VIDEO_ONLY}"

VIDEO_DURATION="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "${VIDEO_ONLY}")"
ffmpeg -hide_banner -loglevel error \
  -i "${VIDEO_ONLY}" \
  -f lavfi -i "anullsrc=channel_layout=stereo:sample_rate=48000" \
  -t "${VIDEO_DURATION}" -shortest \
  -map 0:v:0 -map 1:a:0 \
  -c:v copy -c:a aac -b:a 128k \
  -metadata title="OpenAMRobot UI Feature Tour" \
  -metadata comment="Generated from repository screenshots by scripts/create_asset_video.sh" \
  -movflags +faststart "${OUTPUT_PATH}"

echo "Created: ${OUTPUT_PATH}"
ffprobe -v error \
  -show_entries format=duration,size:stream=codec_name,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 "${OUTPUT_PATH}"
