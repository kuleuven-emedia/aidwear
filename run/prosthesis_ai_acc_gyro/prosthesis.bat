@echo on
call .venv\Scripts\activate
@REM set PYTHONPATH="%cd%"

set "FILE=.\run\prosthesis_ai_acc_gyro\trial_auto_id.txt"
if exist "%FILE%" (
    < "%FILE%" set /p "TRIAL_ID="
) else (
    set "TRIAL_ID=0"
)
set /a TRIAL_ID=%TRIAL_ID% + 1
echo %TRIAL_ID% > "%FILE%"

call hermes-cli -o .\data -f .\run\prosthesis_ai_acc_gyro\prosthesis.yml -e project=AidwearAiAccGyro trial=%TRIAL_ID%
