# 依赖
1. qpOASES
```
git clone https://github.com/coin-or/qpOASES.git
mkdir build
cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..  # 必须要加这个选项，如果你已经安装了qpOASES，但是没加这个选项，就要重新安装
make
sudo make install
```
