@echo off

rem for some reason this needs to go up here or it doesn't work, no idea why
set lpcpath=
for /f %%D in ('wmic volume get DriveLetter^, Label ^| find "CRP DISABLD"') do set lpcpath=%%D

if "%1" == "" (
	set filepath=firmware.bin
) else (
	set filepath=%1
)

if not exist %filepath% (
	echo Firmware file %filepath% not found
) else (
	echo ...Detecting LPC device
	
	if "%lpcpath%" == "" (
		echo LPC device not detected
	) else (
		echo 	Device found at %lpcpath%
		echo ...Removing old firmware
		if exist %lpcpath%\firmware.bin (
			del %lpcpath%\firmware.bin
			echo 	Removed old firmware
		) else (
			echo 	No old firmware found, that's unusual!
		)
		
		echo ...Copying new firmware
		copy %filepath% %lpcpath%\firmware.bin
		
		echo ...Finished
	)
)

pause