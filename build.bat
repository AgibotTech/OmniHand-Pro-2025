
@echo off
setlocal
if exist .\build\install (
    rmdir /s /q .\build\install
)
cmake -B build ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_INSTALL_PREFIX=ON ^
    -DBUILD_PYTHON_BINDING="./build/install" ^
    %*
if %errorlevel% neq 0 (
    echo cmake failed
    exit /b 1
)
cmake --build build --config Release --target install --parallel %NUMBER_OF_PROCESSORS%
if %errorlevel% neq 0 (
    echo build failed
    exit /b 1
)
endlocal