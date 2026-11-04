@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo H10 분석 창을 엽니다...
python h10_gui.pyw
echo.
echo 창이 열리지 않았다면 위 오류 메시지를 확인하세요.
pause
