cmake -DLOOPUNROLL=ON ^
-DBASICMATH=ON ^
-DCOMPLEXMATH=OFF ^
-DCONTROLLER=OFF ^
-DFASTMATH=OFF ^
-DFILTERING=OFF ^
-DMATRIX=OFF ^
-DSTATISTICS=OFF ^
-DSUPPORT=OFF ^
-DTRANSFORM=OFF ^
-DCONFIGTABLE=ON ^
-DCONVOLUTION=OFF ^
-DACTIVATION=OFF ^
-DPOOLING=OFF ^
-DSOFTMAX=OFF ^
-DNEON=ON ^
-DCMAKE_PREFIX_PATH="C:/PROGRA~1/ARM/DEVELO~1.0/sw/ARMCOM~1.12" ^
-DCMAKE_TOOLCHAIN_FILE=../../armcc.cmake ^
-DARM_CPU="cortex-a5" ^
-DPLATFORM="FVP" ^
-G "Unix Makefiles" ..

REM cmake -DLOOPUNROLL=OFF ^
REM -DCMAKE_TOOLCHAIN_FILE=../../armcc.cmake ^
REM -DARM_CPU="cortex-a5" ^
REM -DPLATFORM="FVP" ^
REM -G "Unix Makefiles" ..