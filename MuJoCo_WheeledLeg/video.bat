@echo off
cd /d "%~dp0"
title Wheel-legged Robot Video
echo Recording demo.mp4 (1280x720 @ 50fps, 1-2 min)...
echo.
python render_video.py --width 1280 --height 720 --track
echo.
echo Saved to demo.mp4
pause
