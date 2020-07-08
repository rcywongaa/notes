# Rufus' Notes

A bunch of random scribbles about random stuff that might randomly be useful

## Embedded Linux

### Setting Static IP (Beaglebone)

<https://github.com/leesy24/BBB_Web_Manager/wiki/%5BBBB%5D-Set-static-IP-address-on-eth0>

1. Get network interface name

       $ connmanctl services
       *AO Wired                ethernet_6ceceb5cc3e2_cable

1. Set static IP

       sudo connmanctl config <service> --ipv4 manual <ip_addr> <netmask> <gateway> --nameservers <dns_server>

    Example:

       sudo connmanctl config ethernet_6ceceb5cc3e2_cable --ipv4 manual 10.0.33.33 255.255.0.0
       sudo connmanctl config ethernet_6ceceb5cc3e2_cable --ipv4 manual 10.0.33.33 255.255.0.0 10.0.1.1 --nameservers 10.0.1.1 8.8.8.8


### Device Tree Overlay

- .dts file

        /dts-v1/;
        /plugin/;

        /{
               compatible = "ti,beaglebone", "ti,beaglebone-black";
               part-number = "my_part_number";
               version = "00A0";

               fragment@0 {
                     target = <&am33xx_pinmux>;

                     __overlay__ {
                            my_label: my_pinctrl_name {
                                pinctrl-single,pins = <

                                    0x038 0x37  /* P8_16 14 INPUT  MODE7 pullup */
                                    0x02c 0x37  /* P8_17 11 INPUT  MODE7 pullup */
                                    0x08c 0x37  /* P8_18 35 INPUT  MODE7 pullup */

                                >;
                            };
                     };
               };

               fragment@1 {
                   /* On chip peripherals */
                   target = <&ocp>;
                   __overlay__ {
                       my_ocp_helper_name {
                           compatible = "bone-pinmux-helper";
                           pinctrl-names = "default";
                           pinctrl-0 = <&my_label>;
                           status = "okay";
                       };
                   };
               };
        };

    - `am33xx_pinmux` comes from [amc335x-boneblack-common.dtsi](https://github.com/beagleboard/linux/blob/master/arch/arm/boot/dts/am335x-boneblack-common.dtsi) and [amc335x-bone-common.dtsi](https://github.com/beagleboard/linux/blob/master/arch/arm/boot/dts/am335x-bone-common.dtsi)

      Notice that we are overriding the pin modes specified under am33xx_pinmux

    - [`ti,beaglebone`](https://github.com/beagleboard/linux/blob/7a920684860a790099061b67961d0b5ffa033fdf/Documentation/misc-devices/bone_capemgr.txt)

- Compile `.dts` file to `.dtbo` file and put it in `/lib/firmware/`

      sudo dtc -O dtb -o /lib/firmware/my_dt_name.dtbo my_dt_name.dts

#### dts documentation:

<https://github.com/torvalds/linux/blob/master/Documentation/devicetree/overlay-notes.txt>
<https://github.com/beagleboard/linux/blob/master/Documentation/devicetree/bindings/pinctrl/pinctrl-bindings.txt>
<http://derekmolloy.ie/gpios-on-the-beaglebone-black-using-device-tree-overlays/>

### Pinmux

    cat /sys/kernel/debug/pinctrl/44e10800.pinmux/pinmux-pins
    GPIO NO. for gpioX[Y] = 32*X + Y

### Check I2C chip
    i2cdetect -y -r 0

### Set I2C chip
    i2cset -y 0 0x60 0x40 0x00FF w

### [Bring Up CAN Interface](https://www.elinux.org/Bringing_CAN_interface_up)

    sudo slcand -o -c -f -s6 /dev/ttyUSB0 canusb_name
    sudo ip link set canusb_name up

| ASCII Command | CAN Bitrate |
|---            |---          |
| s0            | 10 Kbit/s   |
| s1            | 20 Kbit/s   |
| s2            | 50 Kbit/s   |
| s3            | 100 Kbit/s  |
| s4            | 125 Kbit/s  |
| s5            | 250 Kbit/s  |
| s6            | 500 Kbit/s  |
| s7            | 800 Kbit/s  |
| s8            | 1000 Kbit/s |

#### Raspberry

- [Forward wireless through ethernet](https://major.io/2015/03/29/share-a-wireless-connection-via-ethernet-in-gnome-3-14/)
- <http://xmodulo.com/remote-control-raspberry-pi.html>
- <http://downloads.raspberrypi.org/raspbian/release_notes.txt>
- <https://www.raspberrypi.org/documentation/remote-access/vnc/>
- SSH disabled by default; can be enabled by creating a file with name "ssh" in BOOT partition (not /boot folder)

## Linux

### `rsync` for remote build

Note that `rsync` treats `directory` and `directory/` differently, unlike `cp`

    rsync -r --delete --exclude build --exclude devel ../project/ <host@ip:project>

OR (if already in `project/` directory)

    rsync -r --delete --exclude build --exclude devel . <host@ip:project>

### Scan IPs

    sudo arp-scan --interface=enp0s31f6 --local

### Connect to wifi via command line

    nmcli device wifi connect <SSID|BSSID> password <password>

<https://docs.fedoraproject.org/en-US/Fedora/20/html/Networking_Guide/sec-Connecting_to_a_Network_Using_nmcli.html>
<https://fedoraproject.org/wiki/Networking/CLI>
<https://developer.gnome.org/NetworkManager/stable/nmcli.html>

### Network configuration files (permanent / CLI)
- `/etc/network/interfaces`
      auto lo
      iface lo inet loopback

      auto eth0
      iface eth0 inet static
         address 192.168.88.110
         netmask 255.255.255.0
         gateway 10.11.12.13

      auto wlan0
      #iface wlan0 inet static
      #   address STATIC_IP_ADDRESS
      #   netmask MASK
      #   gateway 10.11.12.13
      #   wpa-ssid "static wifi"
      #   wpa-psk "static_password"
      #   wpa-conf /etc/wpa_supplicant.conf
      #   pre-up iptables-restore < /etc/iptables.ipv4.nat
      iface wlan0 inet dhcp
          wpa-ssid "FREE WIFI"
          wpa-psk "freewifi"
          wpa-conf /etc/wpa_supplicant.conf
          dns-nameservers 8.8.8.8 8.8.4.4
- `/etc/wpa_supplicant.conf`
      network={
          ssid="FREE WIFI"
          #scan_ssid=1 # Uncomment if hidden wifi
          psk="freewifi"
      }

### Network configuration files (GUI)
- `/etc/NetworkManager/system-connections`

### Unsafe SSH

    ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no user@ip

### Unsafe rsync

    rsync -e "ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no"

### SSH with `sudo` (BAD)

    sshpass -p "ssh_password" ssh user@ip 'echo "sudo_password" | sudo -S my_cmd'

### [Get Script Directory](https://stackoverflow.com/questions/59895/get-the-source-directory-of-a-bash-script-from-within-the-script-itself)

    DIR=$(dirname "$(readlink -f "$0")")

### Run multiple commands as another user in script
`-H` to make sure home directory is correctly updated
```
sudo -uH someuser bash<<EOF
...
EOF
```

### Starting common GUI applications from CLI

    nm-connection-editor
    gnome-control-center
    unity-control-center

### Script with sudo and regular commands
- Run script with sudo / as root
```
sudo root_commands

su user <<EOF
user_commands
EOF
```
- Note that after running `su`, subsequent commands are run in the home directory of the user

### View Image (Eye Of Gnome)

    eog

### [Mount dual boot hard drive as read only](http://www.hecticgeek.com/2012/10/make-ntfs-partitions-read-only-in-ubuntu/)

### Enable core dump

    sudo echo "	* soft core 100000" >> /etc/security/limits.conf
        * soft core 100000
    sudo vim /etc/sysctl.conf
        kernel.core_pattern=core_%t
    sudo sysctl --system
    ulimit -c unlimited

### Important server config files

    /var/www/<site>/public/.htaccess
    /etc/apache2/sites-available/<site>.com.conf

### vi-like commandline controls

    set -o vi

### [Reasons to hate Linux](http://itvision.altervista.org/why.linux.is.not.ready.for.the.desktop.current.html)

### Window manipulation

- devilspie2
- wmctrl

### [Resize home / root partitions](https://unix.stackexchange.com/questions/213245/increase-root-partition-by-reducing-home)

### Find which package provides command

    dpkg -S $(which <command>)

### Purging removed packages

    apt-get purge $(dpkg -l | awk '/^rc/ { print $2 }')

### [Hibernate on close lid](https://help.ubuntu.com/16.04/ubuntu-help/power-hibernate.html)

    sudo vim /etc/systemd/logind.conf

change `HandleLidSwitch=hibernate`

    sudo systemctl restart systemd-logind.service

### List all installed packages

    dpkg -l

<http://askubuntu.com/questions/18804/what-do-the-various-dpkg-flags-like-ii-rc-mean>

### [Restore Windows Bootloader after uninstalling grub](https://askubuntu.com/questions/429610/uninstall-grub-and-use-windows-bootloader)

    c:\> bootsect /nt60 <drive name>: /mbr

### [Get Total Memory Usage](https://unix.stackexchange.com/questions/288589/get-chromes-total-memory-usage)

    smem -t -k -c pss -P /opt/google/chrome | tail -n 1

### Recover from accidentally clearing home directory

    cp -r /etc/skel/.* ~

### Test audio with sample sounds (may require sudo if not in `audio` group)

    aplay /usr/share/sounds/alsa/Noise.wav

### Increase volume above 100%

    pactl list short sinks # List sources
    pactl set-sink-volume 2 250% # Set source to 250%

## GIT
### [Git only stage non-whitespace changes](https://stackoverflow.com/questions/3515597/add-only-non-whitespace-changes)

    git diff -U0 -w --no-color | git apply --cached --ignore-whitespace --unidiff-zero -

### [If upstream removes submodule, must run `git rm removed_submodule`](https://stackoverflow.com/a/16162000/3177701)

## C / C++ / CMake

### Cross Compiling

1. Download the correct toolchain (<https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)>
1. Export the toolchain prefix for ease of compiling

      export ARM_CC="<toolchain_location>/gcc-xxxxxxx/bin/arm-xxxxxx"

1. Setup cmake to use cross compiler

       cmake .. -DCMAKE_C_COMPILER="$ARM_CC"-gcc -DCMAKE_CXX_COMPILER="$ARM_CC"-g++

   OR use a [toolchain file](https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/CrossCompiling)

### `getBinDir()`

    inline const std::string getBinDir()
    {
        const unsigned int LEN = 128;
        char buf[LEN];
        readlink("/proc/self/exe", buf, LEN);
        std::string bin_path(buf);
        bin_path.erase(bin_path.rfind('/'));
        return bin_path + "/";
    }

### `getSrcDir()`

    inline const std::string getSrcDir()
    {
        return std::string(__FILE__).erase(std::string(__FILE__).rfind('/')) + "/";
    }

### No-op default function pointer

    void call_func(std::function<void()>func = []{})

### `std::mutex`
Always consider whether or not the mutex should be `static` or not

    {
        std::lock_guard<std::mutex> lock(mtx);
        ...
    }

### CMake using a system library

1. Check if corresponding cmake module exists

       cmake --help-module-list
       find /usr -name -o "Find*.cmake"

   If found (`FindXXX`), in `CMakeLists.txt`:

       find_package(XXX REQUIRED)

   See [module documentation](https://cmake.org/cmake/help/latest/manual/cmake-modules.7.html)
   for options and variable names

1. Search install location for `.cmake`

       find /usr -name "*.cmake"

   If `XXXConfig.cmake` found, in `CMakeLists.txt`:

       find_package(XXX REQUIRED)

   Open `XXXConfig.cmake` to find variable names (usually `XXX_INCLUDE_DIRS`, `XXX_LIBRARIES`)

1. Search install location for `.pc` file

       find /usr -name "*.pc"

   If `XXX.pc` found, in `CMakeLists.txt`

       find_package(PkgConfig REQUIRED)
       pkg_check_modules(NyName REQUIRED XXX)

   Variable names: `MyName_LIBRARIES`, `MyName_INCLUDE_DIRS`
   See [FindPkgConfig documentation](https://cmake.org/cmake/help/v3.0/module/FindPkgConfig.html)

### [CMake `find_package` fallback](https://stackoverflow.com/questions/50978724/how-can-i-make-find-package-search-with-config-mode-and-fallback-on-module-mode)

    # First time do not use common *REQUIRED* but use QUIET for do not output error messages on fail.
    find_package(XXX CONFIG QUIET)
    if(NOT XXX_FOUND)
        # Previous call has been failed. Fallback with MODULE mode.
        find_package(XXX MODULE REQUIRED) # Now it is OK to use REQUIRED if needed.
        # ... There could be additional actions for wrap result "as if" CONFIG mode.
    endif()
    # ... use XXX

### CMake Default Compiler Option

    OPTION(MY_DEFINE "MY DEFINE" OFF)

    MESSAGE(STATUS "MY_DEFINE=${MY_DEFINE}")

    IF (${MY_DEFINE} MATCHES ON)
        add_compile_options(-DMY_DEFINE)
    ENDIF(${MY_DEFINE} MATCHES ON)

Remember to remove cmake caches when this changes

### Use specific C++ standard in CMake
```
cmake_minimum_required(VERSION 3.0.0)
set (CMAKE_CXX_STANDARD 11)
project(xxx)
```

### Link `pthread` (fix `undefined reference to pthread_create`)
```
find_package(Threads REQUIRED)
...
target_link_libraries(...
    ${CMAKE_THREAD_LIBS_INIT}
)
```

### Make C++ Program Respond To SIGINT

    #include <signal.h>

    void sigint_handler(int s){
        printf("--- SIGINT ---\n");
        isContinue = false;
    }

    int main(int argc, char** argv)
    {
        signal(SIGINT, sigint_handler);
    }

### C++ `unique_ptr`

`std::move` only nulls pointer if the returning value is assigned

    auto other_ptr = std::move(ptr); // Nulls the ptr
    std::move(other_ptr); // Does not null other_ptr

### `static` member gotchas

      class Class1
      {
          public:
              Class1() {}
              void hello() {}
      };

      class Class2
      {
          public:
              Class2() {}
              const static Class1 class1;
      };

      const Class1 Class2::class1; // Don't forget this!

      int main(int argc, char** argv)
      {
          Class2 class2;
          class2.class1.hello();
      };

### Assigning abstract class returned by function
```
AbstractClass& abstract = function_returning_abstract(); // OK
AbstractClass* abstract = &function_returning_abstract(); // OK
AbstractClass abstract = function_returning_abstract(); // Not OK
```

### Using `std::bind` with reference
std::bind by default copies (non-pointer) or moves (pointer) unless specified with std::ref
```
std::bind(&MyClass::func, std::ref(my_class_instance))));
```

### Run arbitrary command after building
```
add_custom_command (OUTPUT my_script_output
    COMMAND ${PROJECT_SOURCE_DIR}/my_script.sh
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/)

...

add_custom_target(
     MyScriptTarget ALL
     DEPENDS my_script_output
)
```

### Using std::chrono::duration as parameter
<https://stackoverflow.com/a/34932923/3177701>

## ROS

### Must read
<https://answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/>

### ROS Dependencies (Fedora)

    sudo dnf install --skip-broken python-empy console-bridge console-bridge-devel poco-devel boost boost-devel eigen3-devel pyqt4 qt-devel gcc gcc-c++ python-devel sip sip-devel tinyxml tinyxml-devel qt-devel qt5-devel python-qt5-devel sip sip-devel python3-sip python3-sip-devel qconf curl curl-devel gtest gtest-devel lz4-devel urdfdom-devel assimp-devel qhull-devel qhull uuid uuid-devel uuid-c++ uuid-c++-devel libuuid libuuid-devel gazebo gazebo-devel collada-dom collada-dom-devel yaml-cpp yaml-cpp-devel python2-defusedxml python-netifaces pyparsing pydot python-pyqtgraph python2-matplotlib

- `rqt_plot` breaks due to both PyQt4 and PyQt5
- do not use anaconda & pip to install missing packages

### [ROS `package.xml` rosdep key](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml)
<https://docs.ros.org/kinetic/api/catkin/html/howto/format1/system_library_dependencies.html>

### CMakeLists.txt for libraries
```
project(my_package_name) # Must match package name defined in package.xml
...
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_library_name # Must match library name defined in add_library(my_library_name ...)
)
...
add_library(my_library_name src/CountedTryFunc.cpp)
add_dependencies(my_library_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(my_library_name PUBLIC ${PROJECT_SOURCE_DIR}/include)
...
install(
  TARGETS
  my_library_name
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### CMakeLists.txt for header only libraries (CMake 3.0+ only)
<https://cmake.org/cmake/help/v3.2/manual/cmake-buildsystem.7.html#interface-libraries>
<http://www.mariobadr.com/creating-a-header-only-library-with-cmake.html>

Note that `colcon` automatically puts header files from `include` into `include/${PROJECT_NAME}`,
hence it is recommended to follow the convention for interoperability between `colcon` and `catkin`
```
include_directories(include)

catkin_package(
    INCLUDE_DIRS include
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
```

### ROS assumes robot is facing +x direction

### Build only some packages & dependencies
```
catkin_make --only-pkg-with-deps pkgs
```

### ROS interface code should be separate from main function / class code
Only bind relevant topics to relevant functions in the node main
```
using namespace std::placeholders;
MyClass my_class;
ros::Subscriber sub = n.subscribe("my_topic", std::bind([&](const std_msgs::Int8::ConstPtr& msg){
        my_class.my_func(msg->data);
    }, _1));
```

### Using functor with `subscribe`, `advertiseService`, `sendGoal`
```
sub = nh.subscribe<MyMessage>("my_topic", 1,
        [&](const MyMessage::ConstPtr& message)
        {
            ... *message ...
        });

service =  nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("my_service",
            [&](std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) -> bool
            {
                ...
            });

//https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer(ExecuteCallbackMethod)
move_base.sendGoal(move_base_goal, [this](const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
        {
            ...
        });

```

### Build Targets
```
catkin_make tests # Build tests only
catkin_make run-tests # Build and run tests only
catkin_make all # All non-tests only
```

### Print tf from `/map` to `/base_link`
```
rosrun tf tf_echo /map /base_link
```

### RPY Specification
In ROS, roll, pitch, yaw are specified w.r.t. the initial frame.

### Linking libraries from another package
<https://answers.ros.org/question/240602/error-in-linking-a-catkin-library-against-another-catkin-package/>

### getMD5Sum error
```
/opt/ros/melodic/include/ros/message_traits.h:126:14: error: ‘const class std::__cxx11::basic_string<char>’ has no member named ‘__getMD5Sum’
     return m.__getMD5Sum().c_str();
```
Occurs when you
```
std::string msg = "...";
pub.publish(msg);
```
instead of
```
std_msgs::String msg;
msg.data = "...";
pub.publish(msg);
```

### Set log level from launch file
```
<node pkg="rosservice" type="rosservice" name="set_move_base_log_level" args="call --wait /move_base/set_logger_level 'ros.move_base' 'debug'" />
```

### `forming pointer to reference type` error
Check if function for subscription callback is fully defined, especially parameter type, i.e.
```
std::function<void(const my_msg_type::SharedPtr)> ...
```

## Eigen

### Initialize constant Eigen::Matrix
```
const Eigen::Matrix3d A((Eigen::Matrix3d() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished());
```

## Python

### Create multiple tests for multiple inputs
<https://stackoverflow.com/a/3772008/3177701>

    import unittest

    # Define a TestCase that takes the test-specific input as constructor parameter
    class Test_my_func(unittest.TestCase):
        def __init__(self, input):
            super(Test_my_func, self).__init__()
            self.input = input

        def runTest(self):
            self.assertAlmostEqual(my_func(self.input), expected_output, delta=TOLERANCE)

    # Create test suite of different tests, each using a different input
    def suite():
        suite = unittest.TestSuite()
        suite.addTests(Test_my_func(input) for input in inputs)
        return suite

    # Run test suite
    if __name__ == '__main__':
        unittest.TextTestRunner().run(suite())

### Argparse multiple positional and optional arguments

    parser = argparse.ArgumentParser()
    parser.add_argument('--optional1', help='Help message for optional1', action="store_true")
    parser.add_argument('--optional2', help='Help message for optional2', action="store_true")
    parser.add_argument('positional', metavar='positional', type=str, help='Help message for positional')
    args = parser.parse_args()

    if arg.optional1:
        print("optional1 == True")
    if arg.optional2:
        print("optional2 == True")
    print(arg.positional)

### Get Current Directory

    curr_dir = os.path.dirname(os.path.abspath(__file__))

## Bumblebee / OpenGL

### OpenGL Libraries

    sudo apt-get install mesa-common-dev mesa-utils-extra libgl1-mesa-dev libglapi-mesa libglu1-mesa-dev

    #include <GL/glx.h> //Required for glX functions (unix systems)

### Extras (Requires Mesa)

    sudo apt-get install libglew-dev libglfw3-dev libglm-dev

    #include <GL/glew.h>
    #include <GLFW/glfw3.h>
    #include <glm/glm.hpp>
    #include <GL/gl.h>

### Bumblebee (required for laptops with 2 cards)

- Check if Bumblebee is needed

      lspci | egrep “3D|VGA”

    - if two lines of output with different cards, bumblebee is needed
    - Note down the BusID (first few numbers, e.g. `01:00.0`)

- <http://askubuntu.com/questions/36930/is-a-nvidia-geforce-with-optimus-technology-supported-by-ubuntu/36936#36936>
- <https://github.com/Bumblebee-Project/Bumblebee/wiki/Configuration>
- <http://askubuntu.com/questions/131506/how-can-i-get-nvidia-cuda-or-opencl-working-on-a-laptop-with-nvidia-discrete-car>
- <https://wiki.ubuntu.com/Bumblebee>
- Open Software & Updates --> Additional Drivers
    - Select Nouveau

- Installation

    sudo apt-get purge "nvidia*"
    rm /etc/X11/xorg.conf
    apt-get --reinstall install libgl1-mesa-glx (prevents the scary low-graphics mode error)
    reboot
    sudo apt-get install bumblebee bumblebee-nvidia linux-headers-generic primus

- Configure bumblebee

      update-alternatives nvidia
      vim /etc/bumblebee/bumblebee.conf

          ...
          [bumblebeed]
              Driver=nvidia
          ...

      :%s/nvidia-current/nvidia-[installed version]/g

    - Make sure BusID is correct (if `01:00.0`)

          vim /etc/bumblebee/xorg.conf.nvidia
          BusID "PCI:01:00:0"

    - Enable bumblebee

          sudo cp /etc/bumblebee/xorg.conf.nvidia /etc/X11/xorg.conf

          sudo systemctl bumblebeed enable

          sudo cp /etc/bumblebee/bumblebee.conf /etc/init

DO NOT INSTALL NVIDIA DRIVERS MANUALLY, let bumblebee install it for you

#### Uninstallation

Uninstall bumblebee --> Uninstall drivers --> Re-install drivers

#### Test

    optirun glxgears

### [OpenCL Libraries](http://wiki.tiker.net/OpenCLHowTo#Downloading_the_OpenCL_headers)

- General

      sudo apt-get install ocl-icd-opencl-dev ocl-icd-libopencl1

- Nvidia

      sudo apt-get install nvidia-opencl-dev ocl-icd-libopencl1 opencl-headers nvidia-cuda-toolkit

- <http://wiki.tiker.net/OpenCLHowTo#Downloading_the_OpenCL_headers>

### Nvidia Cuda

    sudo apt-get install nvidia-cuda-toolkit

- C++ binding to be released with xenial xerus...

### [OpenGL highgui error](http://stackoverflow.com/questions/16287488/runtime-opencv-highgui-error-highgui-error-v4l-v4l2-vidioc-s-crop-opencv-c)

### Check OpenGL version (Ubuntu)

    glxinfo | grep "OpenGL version"

### Check GPU info

    lspsi -v | grep “vga\|3d\|2d”

## Hackintosh Zone

<https://null-byte.wonderhowto.com/how-to/osx-vm-image-install-guide-0170145/>
- If `Guru Meditation`, try reducing memory to 2048
- If  `BIOS disk read error at sector: 00000011`, try choosing UEFI, F12 to enter boot loader and selecting boot device OR try another VM version
- If stuck at BluetoothController, try choosing PIIX chipset
- If stuck at about 2 minutes remaining, <https://www.youtube.com/watch?v=Mx6KtptCePg>

## XMLRPC

- [XMLRPC CLI](http://xmlrpc-c.sourceforge.net/doc/xmlrpc.html)
- List available methods

    xmlrpc http://localhost:8080/RPC2 system.listMethods

## Project Management

- Assumptions Page before signing requirements
- Consider adding "Verified by our own disgression" to avoid unexpected testing / verification procedures from clients
- *How to avoid repeating effort?*
- Design for testability
- Design for reworkability
- Requirement must be backed by evidence before signing off
- Scope Trading
- Prevent prototype to product creep (do not let customers treat prototype as product)
- System diagram must be verifiable (every system block must have an associated test procedure)
- Trusted suppliers / parts list
- Project managers have better opportunity to look for opportunities than sales team
- Limit customer point of contact with engineering team (also control information deliberately)
- Site visit (during intended operation conditions) must be done ASAP, and ideally with sensors and actuators
- Document every component including how to test and how to fix / replace
- Requirements should include minimum (things I guarantee I can deliver) and maximum (things I guarantee I won't do)
- Mention chief driving factor of design
- If party A is responsible for party B, but I depend on party B, create test plan for A to enforce on B
