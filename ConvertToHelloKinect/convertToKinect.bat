@echo off
REM Current folder (where the .bat is)
set "BASEDIR=%cd%"

REM Source folder = .\OrbbecFiles
set "SRC=%BASEDIR%\KinectFiles"

REM Destination folder = one level up
for %%I in ("%BASEDIR%\..") do set "DST=%%~fI"

echo Copying files from %SOURCE% to %DEST%


REM === Move files (overwrites if exists) ===
copy /Y "%SRC%\k4a.dll" "%DST%"
copy /Y "%SRC%\k4arecord.dll" "%DST%"
copy /Y "%SRC%\depthengine_2_0.dll" "%DST%"


echo Files moved successfully!
pause