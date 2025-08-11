# سناریوی اول و دوم
### نحوه نصب و نیازمندی ها

نیازمندی های اصلی:

 ```ROS2 humblle```
 
 ```Ubuntu 22.04```
 
```Gazebo multi-robot simulator, version 11.10.2```

```PX4 Autopilot```

نیازمندی های اولیه:
---

```shell
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```


همچنین از نصب بودن ```gazebo```  نیز اطمینان حاصل کنید.[لینک](https://classic.gazebosim.org/tutorials?tut=install_ubuntu).


### نحوه نصب PX4 Autopilot


```shell
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl

pip install --user -U empy==3.3.4 pyros-genmsg setuptools

git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### ساخت محیط کار (workspace)


   ```shell
    cd ~
    mkdir -p ros2_ws/src/S1&S2
   ```
- سپس این ریپازیتوری رو در  ```ros2_ws/src/S1&S2``` کلون کنبد و اسم آن را به ```px4_swarm_controller``` تغییر بدهید.
  ```shell
  cd ros2_ws/src/S1&S2
  git clone https://github.com/MH-Rostami17/PX4_Swarm_Controller_S1-S2.git
  mv PX4_Swarm_Controller_S1&S2 px4_swarm_controller
  ```
- سپس ```overwrite```  کنید.
  ```shell
  mv -i px4_swarm_controller/sitl_multiple_run.sh ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh
  ```
سپس کامند های زیر رو به ترتیب اجرا کنید.
  ```shell
  mv px4_swarm_controller/custom_msgs/ ~/ros2_ws/src/
  ```

  ```shell
  colcon build
  ```

### نحوه اجرای سناریو اول:

پس از کلون کردن لازم است تا یک فایل ```bash``` به فرم زیر بسازید؛

```shell
cd ~
nano runkon1.bash
# کد پایین را در فایل bash پیست کنید.
cd /your/path/ros2_ws/S1&S2
colcon build
source /your/path/ros2_ws/S1&S2/install/setup.bash
ros2 launch px4_swarm_controller launch_simulation.py
```
از قابل اجرا بودن فایل با کامند ‍‍‍```chmod +x ~/runkon1.bash``` اطمینان حاصل کنید.

سپس با اجرای فایل ```runkon1.bsh/.``` تمام فایل ها اجرا خواهند شد.


تعداد پهپادها، ابعاد هر شکل، ارتفاع و شکل مورد نظر چهارصلعی(quadrilateral)، مثلث(triangle) و خط(line) از فایل ```config/swarm_config.json``` قابل تنظیم هستند.

## نحوه اجرای سناریو دوم:
در حالتی که سناریو اول در حال اجرا میباشد، در ترمینال با استفاده کلید ترکیب ```Ctrl+c``` لانچ فایل اول را متوقف کنید و سپس بلافاصله لانچ فایل سناریو دوم را با استفاده از دستور مقابل اجرا کنید:
```shell
ros2 launch px4_swarm_controller launch_shape_move.py
```
مقدار انتقال شکل(x,y,z,yaw) از طریق تابع ```generate_launch_description()``` موجود در فایل ```launch/launch_shape_move.py``` قابل تنظیم میباشد.


### ویدیوی اجرای سناریو را از این [لینک](https://drive.google.com/file/d/1gHe1KzhkQBPXjn70YhlDvls9rS6gomHj/view?usp=drive_link) تماشا کنید. کنید.

