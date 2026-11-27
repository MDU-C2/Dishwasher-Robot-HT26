@echo off
cd /d "%~dp0"
python capture_training_images.py %*
pause
