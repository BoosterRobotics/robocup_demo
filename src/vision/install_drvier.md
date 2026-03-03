
camera driver installation

## ZED

install zed driver for orin box with **jetpack 6.0**
```bash
# download driver
wget https://download.stereolabs.com/zedsdk/5.0/l4t36.3/jetsons?_gl=1*1i05mdb*_gcl_au*NDU2NTIyNDkzLjE3NDIyOTkwOTg. -O ZED_SDK_Tegra_L4T36.3_v5.0.0.zstd.run

chmod +x ZED_SDK_Tegra_L4T36.3_v5.0.0.zstd.run
./ZED_SDK_Tegra_L4T36.3_v5.0.0.zstd.run -- silent skip_cuda # passwd required
```

refer to [this](https://github.com/stereolabs/zed-ros2-wrapper) to build and launch zed ros wrapper


## Realsense

**DO NOT INSTALL librealsense VIA APT !!!** driver installed via apt fails to enumerate camera correcly. Instead following instructions in this [issue](https://github.com/stereolabs/zed-ros2-wrapper) and build it maunally.
