@echo on
call ..\.venv\Scripts\activate
@REM set PYTHONPATH="%cd%"

set "FILE=.\run\ai_intent_imu\trial_auto_id.txt"
if exist "%FILE%" (
    < "%FILE%" set /p "TRIAL_ID="
) else (
    set "TRIAL_ID=0"
)
set /a TRIAL_ID=%TRIAL_ID% + 1
echo %TRIAL_ID% > "%FILE%"

call hermes-cli -o .\data -f .\run\ai_intent_imu\exo.yml -e project=RevalexoAiIntentImu type=Test trial=%TRIAL_ID%
