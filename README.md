# Rufus' Notes

A bunch of random scribbles about random stuff that might randomly be useful

## Beaglebone

### Setting Static IP

https://github.com/leesy24/BBB_Web_Manager/wiki/%5BBBB%5D-Set-static-IP-address-on-eth0

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

https://github.com/torvalds/linux/blob/master/Documentation/devicetree/overlay-notes.txt
https://github.com/beagleboard/linux/blob/master/Documentation/devicetree/bindings/pinctrl/pinctrl-bindings.txt
http://derekmolloy.ie/gpios-on-the-beaglebone-black-using-device-tree-overlays/

### Pinmux

    cat /sys/kernel/debug/pinctrl/44e10800.pinmux/pinmux-pins
    GPIO NO. for gpioX[Y] = 32*X + Y

### Check I2C chip
    i2cdetect -y -r 0

### Set I2C chip
    i2cset -y 0 0x60 0x40 0x00FF w

## Raspberry

- [Forward wireless through ethernet](https://major.io/2015/03/29/share-a-wireless-connection-via-ethernet-in-gnome-3-14/)
- http://xmodulo.com/remote-control-raspberry-pi.html
- http://downloads.raspberrypi.org/raspbian/release_notes.txt
- https://www.raspberrypi.org/documentation/remote-access/vnc/
- SSH disabled by default; can be enabled by creating a file with name "ssh" in BOOT partition (not /boot folder)

## Unsafe SSH

    ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no user@ip

## SSH with `sudo` (BAD)

    sshpass -p “ssh_password” ssh user@ip 'echo "sudo_password" | sudo -S my_cmd'

## Cross Compiling

1. Download the correct toolchain (https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
1. Export the toolchain prefix for ease of compiling

      export ARM_CC=”<toolchain_location>/gcc-xxxxxxx/bin/arm-xxxxxx”

1. Setup cmake to use cross compiler

       cmake .. -DCMAKE_C_COMPILER="$ARM_CC"-gcc -DCMAKE_CXX_COMPILER="$ARM_CC"-g++

   OR use a [toolchain file](https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/CrossCompiling)

## `getBinDir()`

    inline const std::string getBinDir()
    {
        const unsigned int LEN = 128;
        char buf[LEN];
        readlink("/proc/self/exe", buf, LEN);
        std::string bin_path(buf);
        bin_path.erase(bin_path.rfind('/'));
        return bin_path + "/";
    }

## `getSrcDir()`

    inline const std::string getSrcDir()
    {
        return std::string(__FILE__).erase(std::string(__FILE__).rfind('/')) + "/";
    }

## No-op default function pointer

    void call_func(std::function<void()>func = []{})

## [Bring Up CAN Interface](https://www.elinux.org/Bringing_CAN_interface_up)

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

## CMake module path (for FindXXX.cmake)

    usr/share/cmake/Modules/

## CMake check if module exists

    cmake --help-module-list

## Starting common GUI applications from CLI

    nm-connection-editor
    gnome-control-center
    unity-control-center

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

- http://askubuntu.com/questions/36930/is-a-nvidia-geforce-with-optimus-technology-supported-by-ubuntu/36936#36936
- https://github.com/Bumblebee-Project/Bumblebee/wiki/Configuration
- http://askubuntu.com/questions/131506/how-can-i-get-nvidia-cuda-or-opencl-working-on-a-laptop-with-nvidia-discrete-car
- https://wiki.ubuntu.com/Bumblebee
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

- http://wiki.tiker.net/OpenCLHowTo#Downloading_the_OpenCL_headers

### Nvidia Cuda

    sudo apt-get install nvidia-cuda-toolkit

- C++ binding to be released with xenial xerus...

### [OpenGL highgui error](http://stackoverflow.com/questions/16287488/runtime-opencv-highgui-error-highgui-error-v4l-v4l2-vidioc-s-crop-opencv-c)

### Check OpenGL version (Ubuntu)

    glxinfo | grep "OpenGL version"

### Check GPU info

    lspsi -v | grep “vga\|3d\|2d”

## View Image (Eye Of Gnome)

    eog

## [Mount dual boot hard drive as read only](http://www.hecticgeek.com/2012/10/make-ntfs-partitions-read-only-in-ubuntu/)

## Enable core dump

    sudo echo "	* soft core 100000" >> /etc/security/limits.conf
        * soft core 100000
    sudo vim /etc/sysctl.conf
        kernel.core_pattern=core_%t
    sudo sysctl --system
    ulimit -c unlimited

## Important server config files

    /var/www/<site>/public/.htaccess
    /etc/apache2/sites-available/<site>.com.conf

## vi-like commandline controls

    set -o vi

## [Reasons to hate Linux](http://itvision.altervista.org/why.linux.is.not.ready.for.the.desktop.current.html)

## Window manipulation

- devilspie2
- wmctrl

## [Resize home / root partitions](https://unix.stackexchange.com/questions/213245/increase-root-partition-by-reducing-home)

## [Git only stage non-whitespace changes](https://stackoverflow.com/questions/3515597/add-only-non-whitespace-changes)

    git diff -U0 -w --no-color | git apply --cached --ignore-whitespace --unidiff-zero -

## Purging removed packages

    apt-get purge $(dpkg -l | awk '/^rc/ { print $2 }')

## [Hibernate on close lid](https://help.ubuntu.com/16.04/ubuntu-help/power-hibernate.html)

    sudo vim /etc/systemd/logind.conf

change `HandleLidSwitch=hibernate`

    sudo systemctl restart systemd-logind.service

## List all installed packages

    dpkg -l

http://askubuntu.com/questions/18804/what-do-the-various-dpkg-flags-like-ii-rc-mean

## [Restore Windows Bootloader after uninstalling grub](https://askubuntu.com/questions/429610/uninstall-grub-and-use-windows-bootloader)

    c:\> bootsect /nt60 <drive name>: /mbr

## ROS Dependencies (Fedora)

    sudo dnf install --skip-broken python-empy console-bridge console-bridge-devel poco-devel boost boost-devel eigen3-devel pyqt4 qt-devel gcc gcc-c++ python-devel sip sip-devel tinyxml tinyxml-devel qt-devel qt5-devel python-qt5-devel sip sip-devel python3-sip python3-sip-devel qconf curl curl-devel gtest gtest-devel lz4-devel urdfdom-devel assimp-devel qhull-devel qhull uuid uuid-devel uuid-c++ uuid-c++-devel libuuid libuuid-devel gazebo gazebo-devel collada-dom collada-dom-devel yaml-cpp yaml-cpp-devel python2-defusedxml python-netifaces pyparsing pydot python-pyqtgraph python2-matplotlib 

- `rqt_plot` breaks due to both PyQt4 and PyQt5
- do not use anaconda & pip to install missing packages

## Hackintosh Zone

https://null-byte.wonderhowto.com/how-to/osx-vm-image-install-guide-0170145/
- If `Guru Meditation`, try reducing memory to 2048
- If  `BIOS disk read error at sector: 00000011`, try choosing UEFI, F12 to enter boot loader and selecting boot device OR try another VM version
- If stuck at BluetoothController, try choosing PIIX chipset
- If stuck at about 2 minutes remaining, https://www.youtube.com/watch?v=Mx6KtptCePg

## CMake Default Compiler Option

    OPTION(MY_DEFINE "MY DEFINE" OFF)

    MESSAGE(STATUS "MY_DEFINE=${MY_DEFINE}")

    IF (${MY_DEFINE} MATCHES ON)
        add_compile_options(-DMY_DEFINE)
    ENDIF(${MY_DEFINE} MATCHES ON)

Remember to remove cmake caches when this changes

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
