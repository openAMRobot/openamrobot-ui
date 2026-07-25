#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET_DIR="${REPO_ROOT}/docs/assets"
SOURCE_VIDEO="${1:-${ASSET_DIR}/openamrobot_ui_feature_tour.mp4}"
VOICEOVER_AUDIO="${2:-}"
OUTPUT_VIDEO="${3:-${ASSET_DIR}/openamrobot_ui_feature_tour_with_audio.mp4}"
OUTPUT_AUDIO="${4:-${ASSET_DIR}/openamrobot_ui_feature_tour_narration.m4a}"

if [ -z "${VOICEOVER_AUDIO}" ]; then
  echo "Usage: $0 SOURCE_VIDEO VOICEOVER_AUDIO [OUTPUT_VIDEO] [OUTPUT_AUDIO]"
  exit 1
fi

for command_name in ffmpeg ffprobe awk; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: ${command_name} is required."
    exit 1
  fi
done

for input_path in "${SOURCE_VIDEO}" "${VOICEOVER_AUDIO}"; do
  if [ ! -f "${input_path}" ]; then
    echo "ERROR: input file not found: ${input_path}"
    exit 1
  fi
done

# Source boundaries correspond to the title, 16 feature slides, and closing card
# produced by create_asset_video.sh.
SOURCE_STARTS=(
  0 4 7.5 11 14.5 18 21.5 25 28.5
  32 35.5 39 42.5 46 49.5 53 56.5 60
)
SOURCE_ENDS=(
  4 7.5 11 14.5 18 21.5 25 28.5 32
  35.5 39 42.5 46 49.5 53 56.5 60 64
)

# These durations align the slides with the natural sentence pauses in the
# supplied 83.775-second English voice-over.
OUTPUT_DURATIONS=(
  7.61073 5.53537 3.0339 4.5602 4.3371 3.2727 4.2848 5.1623 4.219
  4.8047 4.7487 4.4158 4.9639 3.9661 5.9242 3.4382 4.6327 4.8643
)

VOICEOVER_DURATION="$(
  ffprobe -v error -show_entries format=duration -of csv=p=0 "${VOICEOVER_AUDIO}"
)"
EXPECTED_DURATION="83.7747"
if ! awk -v actual="${VOICEOVER_DURATION}" -v expected="${EXPECTED_DURATION}" \
  'BEGIN { difference = actual - expected; if (difference < 0) difference = -difference; exit !(difference <= 0.10) }'; then
  echo "ERROR: this timing map expects an approximately ${EXPECTED_DURATION}-second voice-over."
  echo "Received: ${VOICEOVER_DURATION} seconds."
  exit 1
fi

ffmpeg -hide_banner -loglevel error \
  -i "${VOICEOVER_AUDIO}" \
  -af "loudnorm=I=-18:TP=-1.5:LRA=11,aresample=48000,pan=stereo|c0=c0|c1=c0" \
  -c:a aac -b:a 160k -ar 48000 \
  -metadata title="OpenAMRobot UI Feature Tour Narration" \
  -metadata artist="openAMRobot" \
  -movflags +faststart \
  -y "${OUTPUT_AUDIO}"

FILTER_COMPLEX=""
CONCAT_INPUTS=""
for index in "${!SOURCE_STARTS[@]}"; do
  source_duration="$(
    awk -v start="${SOURCE_STARTS[${index}]}" -v end="${SOURCE_ENDS[${index}]}" \
      'BEGIN { print end - start }'
  )"
  speed_factor="$(
    awk -v output="${OUTPUT_DURATIONS[${index}]}" -v source="${source_duration}" \
      'BEGIN { printf "%.10f", output / source }'
  )"
  FILTER_COMPLEX+="[0:v]trim=start=${SOURCE_STARTS[${index}]}:end=${SOURCE_ENDS[${index}]},"
  FILTER_COMPLEX+="setpts=(PTS-STARTPTS)*${speed_factor}[v${index}];"
  CONCAT_INPUTS+="[v${index}]"
done
FILTER_COMPLEX+="${CONCAT_INPUTS}concat=n=${#SOURCE_STARTS[@]}:v=1:a=0[vout]"

ffmpeg -hide_banner -loglevel error \
  -i "${SOURCE_VIDEO}" -i "${OUTPUT_AUDIO}" \
  -filter_complex "${FILTER_COMPLEX}" \
  -map "[vout]" -map 1:a:0 \
  -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -r 30 \
  -c:a copy -shortest \
  -metadata title="OpenAMRobot UI Feature Tour with Human Voice-over" \
  -metadata comment="Slides synchronized by scripts/add_feature_tour_voiceover.sh" \
  -movflags +faststart \
  -y "${OUTPUT_VIDEO}"

echo "Created narration: ${OUTPUT_AUDIO}"
echo "Created synchronized video: ${OUTPUT_VIDEO}"
ffprobe -v error \
  -show_entries format=duration,size:stream=codec_name,width,height,sample_rate,channels \
  -of default=noprint_wrappers=1 "${OUTPUT_VIDEO}"
