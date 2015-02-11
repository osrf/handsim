@set build_type=Release
@if not "%1"=="" set build_type=%1

@echo Configuring for build type %build_type%
cmake -DCMAKE_INSTALL_PREFIX="install/%build_type%" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE="%build_type%" ..
@if %errorlevel% neq 0 exit /b %errorlevel%
@echo Configuration complete.  To build, run `nmake`
