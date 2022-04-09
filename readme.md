## install OSQP
cmake -G "Unix Makefiles" ..
cmake --build .
cmake --build . --target install

## 部署
* `run.sh`中Windows端IP--`UE4IP`
* `run.sh`中1号机在RflySim中设置的原点偏移--`MAVX`, `MAVY`, `MAVZ`