@ECHO OFF

REM The text start with 'REM' is annotation, the following options are corresponding to Options on CopterSim

REM Start index of vehicle number (should larger than 0)
REM This option is useful for simulation with multi-computers
SET /a START_INDEX=1
SET /a TOTOAL_COPTER=15

REM Set the start UDP port for SIMULINK/OFFBOARD API
REM This option should not be modified for swarm simulation
SET /a UDP_START_PORT=20100


REM Set use DLL model name or not, use number index or name string
REM This option is useful for simulation with other types of vehicles instead of multicopters
set DLLModel=AircraftMathworks

REM Set the simulation mode on CopterSim, use number index or name string
REM e.g., SimMode=0 equals to  SimMode=PX4_HITL
set SimMode=0

REM Set the vehicle-model (airframe) of PX4 SITL simulation, the default airframe is a quadcopter: iris
REM Check folder Firmware\ROMFS\px4fmu_common\init.d-posix for supported airframes (Note: You can also create your airframe file here)
REM E.g., fixed-wing aircraft: PX4SitlFrame=plane; small cars: PX4SitlFrame=rover
:: set PX4SitlFrame=iris


REM Set the map, use index or name of the map on CopterSim
REM e.g., UE4_MAP=1 equals to UE4_MAP=Grasslands
SET UE4_MAP=Desert_online

REM Set the origin x,y position (m) and yaw angle (degree) at the map
SET /a ORIGIN_POS_X=0
SET /a ORIGIN_POS_Y=-9
SET /a ORIGIN_YAW=0

REM Set the interval between two vehicle, unit:m 
SET /a VEHICLE_INTERVAL=2


REM Set broadcast to other computer; 0: only this computer, 1: broadcast; or use IP address to increase speed
REM e.g., IS_BROADCAST=0 equals to IS_BROADCAST=127.0.0.1, IS_BROADCAST=1 equals to IS_BROADCAST=255.255.255.255
SET IS_BROADCAST=192.168.3.6

REM Set UDP data mode; 0: UDP_FULL, 1:UDP_Simple, 2: Mavlink_Full, 3: Mavlink_simple. input number or string
REM e.g., UDPSIMMODE=1 equals to UDPSIMMODE=UDP_Simple
SET UDPSIMMODE=1

REM Set the path of the RflySim tools
SET PSP_PATH=C:\PX4PSP
SET PSP_PATH_LINUX=/mnt/c/PX4PSP
C:


::    :Top
::    ECHO.
::    ECHO ---------------------------------------
::    REM Max vehicle number 100
::    SET /a MAX_VEHICLE=100
::    SET /P VehicleNum=Please input UAV swarm number:
::    SET /A Evaluated=VehicleNum
::    if %Evaluated% EQU %VehicleNum% (
::        IF %VehicleNum% GTR 0 (
::            IF %VehicleNum% GTR %MAX_VEHICLE% (
::                ECHO The vehicle number is too large, which may cause a crash
::                pause
::            )
::            GOTO StartSim

::        )
::        ECHO Not a positive integer
::        GOTO Top
::    ) ELSE (
::        ECHO Not a integer
::        GOTO Top
::    )
::    :StartSim

ECHO.
ECHO ---------------------------------------
REM Get the Com port number
for /f "delims=" %%t in ('%PSP_PATH%\CopterSim\GetComList.exe 2') do set ComNumExe=%%t

REM Get the Com port list
for /f "delims=" %%t in ('%PSP_PATH%\CopterSim\GetComList.exe 0') do set ComNameList=%%t

REM Get the Com port info
for /f "delims=" %%t in ('%PSP_PATH%\CopterSim\GetComList.exe 1') do set ComInfoList=%%t

echo Please input the Pixhawk COM port list for HIL
echo Use ',' as the separator if more than one Pixhawk
echo E.g., input 3 for COM3 of Pixhawk on the computer
echo Input 3,6,7 for COM3, COM6 and COM7 of Pixhawks 
echo.
set remain=%ComInfoList%
if %ComNumExe% EQU 0 (
    echo Warning: there is no available COM port
) else (
    echo Available COM ports on this computer are:
    :loopInfo
    for /f "tokens=1* delims=;" %%a in ("%remain%") do (
        echo %%a
        set remain=%%b
    )
    if defined remain goto :loopInfo
    echo.
    echo Recommended COM list input is: %ComNameList%
)




ECHO.
ECHO ---------------------------------------
SET /P ComNum=My COM list for HITL simulation is:
SET string=%ComNum%
set subStr = ""
set /a VehicleNum=0
:split
    for /f "tokens=1,* delims=," %%i in ("%string%") do (
    set subStr=%%i
    set string=%%j
    )
    set /a eValue=subStr
    if not %eValue% EQU %subStr% (
        echo Error: Input '%subStr%' is not a integer!
        goto EOF
    )
    set /a VehicleNum = VehicleNum +1
if not "%string%"=="" goto split


SET /A VehicleTotalNum=%VehicleNum% + %START_INDEX% - 1
if not defined TOTOAL_COPTER (
    SET /A TOTOAL_COPTER=%VehicleTotalNum%
)

set /a sqrtNum=1
set /a squareNum=1
:loopSqrt
set /a squareNum=%sqrtNum% * %sqrtNum%
if %squareNum% EQU %TOTOAL_COPTER% (
    goto loopSqrtEnd
)
if %squareNum% GTR %TOTOAL_COPTER% (
    goto loopSqrtEnd
)
set /a sqrtNum=%sqrtNum%+1
goto loopSqrt
:loopSqrtEnd


REM UE4Path
cd %PSP_PATH%\RflySim3D
tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe
choice /t 5 /d y /n >nul


tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
ECHO Kill all CopterSims


REM CptSmPath
cd %PSP_PATH%\CopterSim

set /a cntr = %START_INDEX%
set /a endNum = %VehicleTotalNum% +1
set /a portNum = %UDP_START_PORT% + ((%START_INDEX%-1)*2)
SET string=%ComNum%
:split1
    for /f "tokens=1,* delims=," %%i in ("%string%") do (
    set subStr=%%i
    set string=%%j
    )
    set /a PosXX=-((%cntr%-1) / 10)*%VEHICLE_INTERVAL% + %ORIGIN_POS_X%
    set /a PosYY=((%cntr%-1) %% 10)*%VEHICLE_INTERVAL% + %ORIGIN_POS_Y%
    REM echo start CopterSim
    start /realtime CopterSim.exe 1 %cntr% %portNum% %DLLModel% %SimMode% %UE4_MAP% %IS_BROADCAST% %PosXX% %PosYY% %ORIGIN_YAW% %subStr% %UDPSIMMODE%
    choice /t 1 /d y /n >nul
    set /a cntr=%cntr%+1
    set /a portNum = %portNum% +2
    REM TIMEOUT /T 1
if not "%string%"=="" goto split1

REM QGCPath
tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe
ECHO Start QGroundControl

if "%IS_BROADCAST%" == "0" (
    SET IS_BROADCAST=0
) else (
    SET IS_BROADCAST=1
)

pause

REM kill all applications when press a key
tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"
tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"

ECHO Start End.
