#!/usr/bin/env bash
# PNG frames from build_scene.py --anim -> hero videos with title, legend and HiRISE credit:
#   tools/blender/encode.sh <frames_dir> <out_dir>
# writes  <out_dir>/hero_1080p.mp4  master (large, not committed)
#         <out_dir>/hero.mp4        720p, small enough for the repo, linked from the README
#         <out_dir>/hero.gif        short README clip (GIF_FROM / GIF_LEN seconds, kept under ~5 MB)
set -euo pipefail
cd "$(dirname "$0")/../.."
FRAMES=$1
OUT=$2
FPS=30
HOLD=2                          # seconds to hold the last frame (the route overview)
GIF_FROM=${GIF_FROM:-4}
GIF_LEN=${GIF_LEN:-6}
FF=$(.venv/bin/python -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())")
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
BOLD=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
STYLE="fontcolor=white:shadowcolor=black@0.6:shadowx=2:shadowy=2"
FADE="alpha='if(lt(t,3),1,max(0,1-(t-3)))'"  # title holds 3 s, fades out over 1 s

VF="tpad=stop_mode=clone:stop_duration=$HOLD"
VF+=",drawtext=fontfile=$BOLD:text='Autonomous navigation on Jezero crater terrain':fontsize=56:$STYLE:x=(w-tw)/2:y=h*0.1:$FADE"
VF+=",drawtext=fontfile=$FONT:text='ROS 2 Nav2 + MuJoCo  |  recorded physics run, scenario rocks_01 (Smac Hybrid-A* + DWB)':fontsize=30:$STYLE:x=(w-tw)/2:y=h*0.1+80:$FADE"
VF+=",drawtext=fontfile=$FONT:text='cyan = Nav2 plan, replanned live      white = true driven path      flag = goal':fontsize=28:$STYLE:x=40:y=h-th-50:enable='gte(t,4)'"
VF+=",drawtext=fontfile=$FONT:text='Terrain  HiRISE DTM, NASA/JPL-Caltech/UArizona (false colour)':fontsize=20:fontcolor=white@0.8:x=w-tw-30:y=h-th-20"

mkdir -p "$OUT"
"$FF" -hide_banner -loglevel error -y -framerate $FPS -i "$FRAMES/%04d.png" -vf "$VF" \
    -c:v libx264 -pix_fmt yuv420p -crf 18 -movflags +faststart "$OUT/hero_1080p.mp4"
"$FF" -hide_banner -loglevel error -y -i "$OUT/hero_1080p.mp4" -vf "scale=1280:-2:flags=lanczos" \
    -c:v libx264 -preset slow -pix_fmt yuv420p -crf 28 -movflags +faststart "$OUT/hero.mp4"
"$FF" -hide_banner -loglevel error -y -ss "$GIF_FROM" -t "$GIF_LEN" -i "$OUT/hero_1080p.mp4" \
    -vf "fps=10,scale=480:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" \
    "$OUT/hero.gif"
ls -la "$OUT"/hero_1080p.mp4 "$OUT"/hero.mp4 "$OUT"/hero.gif
