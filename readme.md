## 安装依赖
相关文件位于thirdparty，参考 https://blog.csdn.net/qq_34859576/article/details/121094060
* 安装`OSQP`
```
    cmake -G "Unix Makefiles" ..
    cmake --build .
    cmake --build . --target install
```
* 安装`OSQP-Eigen`
```
    cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local ../
    make
    sudo make install
```

## 部署
* `run.sh`中Windows端IP--`UE4IP`
* `run.sh`中1号机在RflySim中设置的原点偏移--`MAVX`, `MAVY`, `MAVZ`
