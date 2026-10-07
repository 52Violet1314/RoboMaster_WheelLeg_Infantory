@echo off
setlocal
cd /d "%~dp0"
title wheelbipeV14_2 - VMC+LQR MuJoCo Sim

:menu
cls
echo ============================================================
echo   wheelbipeV14_2  Wheel-legged robot  VMC + LQR  (MuJoCo)
echo ============================================================
echo.
echo   [1]  Keyboard drive        (8/5 move / 9/3 leg / 4/6 turn)
echo   [2]  Keyboard drive        (slow-motion 0.5x)
echo   [3]  Record demo video     (demo.mp4)
echo   [4]  View full 37-link model
echo   [5]  View exported URDF
echo   [6]  Run full verification (stand / push / leg / K)
echo   [7]  Stand for 60 seconds  (print state)
echo.
echo   [0]  Exit
echo.
set /p choice=  Choose [0-7]:

if "%choice%"=="1" goto drive
if "%choice%"=="2" goto drive_slow
if "%choice%"=="3" goto video
if "%choice%"=="4" goto full
if "%choice%"=="5" goto urdf
if "%choice%"=="6" goto verify
if "%choice%"=="7" goto stand
if "%choice%"=="0" goto end
echo  Invalid choice.
timeout /t 1 >nul
goto menu

:drive
cls
echo  Starting keyboard drive...
echo  (hold 8/5 speed  hold 4/6 turn  9/3 leg  Space stop  X e-stop  Z reset)
echo.
python view_gui.py
goto menu

:drive_slow
cls
echo  Starting keyboard drive (0.5x)...
python view_gui.py --realtime 0.5
goto menu

:video
cls
echo  Recording demo.mp4 (1280x720 @ 50fps, 1-2 min)...
echo.
python render_video.py --width 1280 --height 720 --track
echo.
echo  Done. Saved to demo.mp4
pause
goto menu

:full
cls
echo  Opening full 37-link model (6 spherical loop joints)...
python view_model.py --full
goto menu

:urdf
cls
echo  Opening exported URDF...
python view_model.py --urdf
goto menu

:verify
cls
echo  Running full verification...
echo.
python verify.py
echo.
pause
goto menu

:stand
cls
echo  Standing for 60 seconds...
echo.
python sim_lqr.py --seconds 60
echo.
pause
goto menu

:end
endlocal
echo  Bye.
exit /b 0
