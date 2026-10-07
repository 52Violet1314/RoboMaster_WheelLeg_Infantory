@echo off
cd /d "%~dp0"
title Wheel-legged Robot Drive
echo Keyboard drive - directly.
echo Hold 8/5 speed  Hold 4/6 turn  9/3 leg  Space stop  X e-stop  Z reset
echo.
python view_gui.py
pause
