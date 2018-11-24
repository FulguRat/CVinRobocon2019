### Hades Canyon Configuration

Copyright © 2018 FortiSTray_Action. All Rights Reserved.

---

[TOC]

#### Update firmware if this NUC is a new machine

* Download the BIOS file (`.bio` file) from this website

  > https://downloadcenter.intel.com/download/28073/BIOS-Update-HNKBLi70-86A-

  (You can choose the latest version)

* Update the BIOS using instructions here

  > https://www.intel.com/content/www/us/en/support/articles/000005850/mini-pcs.html

#### Install Ubuntu 16.04.5 LTS with default kernel

* Visit ubuntu 16.04.5 LTS image website

  > http://releases.ubuntu.com/16.04.5/

  Download image with name `ubuntu-16.04.5-desktop-amd64.iso `

  Make an Ubuntu startup media on a USB flash driver

* Boot up NUC8i7HVK

  When see the BIOS screen, press `F2` to enter the BIOS settings

  Press `F9` to reset all settings

  Press `F10` to save all and reboot

* Plug in your install driver

  When see the BIOS screen, press `F10` and choose your USB driver

  When NUC shows the grub screen (You can choose try or install Ubuntu here),  press `e`

  Replace the words `quiet splash` with`nomodeset`

  Press `Ctrl + X` to exit and boot

* You will see a Ubuntu GUI after boot

  Run Install icon on the desktop and go through the usual installation (set the password to `action`)

  reboot

* You will get a black screen here

  Press `Ctrl + Alt + F3` to enter the terminal 3 (tty3)

  Input your user name and password to log in the Ubuntu

  ```bash
  sudo nano /etc/default/grub
  ```

  Changed the line `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"` to `GRUB_CMDLINE_LINUX_DEFAULT="nomodeset"`

  ```bash
  sudo update-grub2
  reboot
  ```

  Unplug your USB driver

* When see the BIOS screen, press `F2` to enter the BIOS settings

  Set `Advanced > Boot > Secure Boot install cmake > Secure Boot Config > Secure Boot = unchecked`

* ```bash
  uname -r
  ```

  Check if the kernel version is 4.15.0

#### Preparation before installing libraries

##### Change Ubuntu source to tuna

```bash
sudo nano /etc/apt/sources.list
```

Replace all contains to follows

```bash
# 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-updates main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-backports main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-security main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-security main restricted universe multiverse

# 预发布软件源，不建议启用
# deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-proposed main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-proposed main restricted universe multiverse
```

```bash
sudo apt update && sudo apt upgrade && sudo apt dist-upgrade
```

##### Install Git

```bash
sudo apt install git
```

##### Install CMake

```bash
sudo apt install cmake
```

#### Install librealsense (v2.16.1)

* Download source code from Github (Release > Intel® RealSense™ SDK 2.0 (build 2.16.1))

  Extract the `.tar.gz` file to `~/Library`

  Unplug any connected Intel RealSense camera

* Navigate to librealsense root directory

  ```bash
  sudo apt-get install libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
  sudo apt-get install libglfw3-dev
  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
  sudo udevadm control --reload-rules && udevadm trigger
  ./scripts/patch-realsense-ubuntu-lts.sh
  echo 'hid_sensor_custom' | sudo tee -a /etc/modules
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  sudo make uninstall && make clean && make -j8 && sudo make install
  ```

#### Install PCL (v1.8.1)

* Download source code from Github (Release > pcl-1.8.1)

  Extract the `.tar.gz` file to `~/Library`

* Navigate to PCL root directory

  ```bash
  sudo apt install libboost-all-dev libeigen3-dev libflann-dev libvtk6-dev
  sudo apt install libproj-dev
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j8
  sudo make -j8 install
  ```

#### Install OpenCV (v3.4.4) with OpenCV_Contrib (v3.4.4)

- Download OpenCV source code from Github (Release > OpenCV 3.4.4)

  Download OpenCV_Contrib source code from Github (Release > 3.4.4)

  Extract the `.tar.gz` file to `~/Library`

  Copy OpenCV_Contrib folder into OpenCV folder and rename it as `opencv_contrib`

- Navigate to OpenCV root directory

  ```bash
  sudo apt-get install build-essential
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ..
  make -j8
  sudo make install
  ```

#### Work environment optimization

##### Install chromium

```bash
sudo apt install chromium-browser
```

##### Install Sogou Pinyin input method

Download `.deb` file from official website

```bash
sudo dpkg -i sogoupinyin_2.2.0.0108_amd64.deb
sudo install -f
```

Set `System Settings > Language Support > Keyboard input method system = fcitx`

Log out &  Log in

Keyboard icon on the right top > add `Sogou Pinyin`

```bash
sudo apt-get remove fcitx-ui-qimpanel
```

Log out &  Log in

##### Install vscode

Download `.deb` file from official website

```bash
sudo dpkg -i code_1.29.1-1542309157_amd64.deb
sudo install -f
```

##### Install Typora

```bash
# or run:
# sudo apt-key adv --keyserver keyserver.ubuntu.com--recv-keys BA300B7755AFCFAE
wget -qO - https://typora.io/linux/public-key.asc | sudo apt-key add -

# add Typora's repository
sudo add-apt-repository 'deb https://typora.io/linux ./'
sudo apt-get update

# install typora
sudo apt-get install typora
```



