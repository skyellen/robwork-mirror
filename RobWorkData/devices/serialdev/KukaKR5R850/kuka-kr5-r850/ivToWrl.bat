echo %~f1
set PATH=%~f1;%PATH%
set CURRDIR=%~f2
set BACKUPDIRECTORY=%~f1
PUSHD %BACKUPDIRECTORY%
FOR %%A in (*.iv) do CALL :Subroutine %%A
POPD
GOTO:EOF

:Subroutine
PUSHD %CURRDIR%
ivvrml %BACKUPDIRECTORY%\%~nx1 -1 %BACKUPDIRECTORY%\%~n1.wrl
POPD
GOTO:EOF