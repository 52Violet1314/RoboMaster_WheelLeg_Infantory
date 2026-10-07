@echo off
cd /d "%~dp0"
title Wheel-legged Robot Verification
echo Running full verification (stand / push / leg / K recompute / firmware cross-check)...
echo.
python verify.py
echo.
echo Verification finished.
pause >nul
