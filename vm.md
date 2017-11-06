# Referenz vm: #
https://box.hu-berlin.de/d/d1c0dbde976a460b94f9/

* test
	* ros2 run demo_nodes_cpp talker -- -t chatter2


# virtualbox 5.2.0 #
http://download.virtualbox.org/virtualbox/5.2.0/virtualbox-5.2_5.2.0-118431~Debian~stretch_amd64.deb
oder
http://download.virtualbox.org/virtualbox/5.2.0/virtualbox-5.2_5.2.0-118431~Debian~stretch_amd64.deb

* install:
  * downloaden
  * dpkg -i virtualbox...
  * sudo apt-get install linux-headers-amd64
  * falls was nicht klappt "rcvboxdrv setup"

# Ubuntu xenial 1604 LTS 3 x64 #
http://releases.ubuntu.com/16.04/ubuntu-16.04.3-desktop-amd64.iso.torrent

* install:
  * https://askubuntu.com/questions/142549/how-to-install-ubuntu-on-virtualbox
  * user: ros pwd: ros
  * guest additions:
    * install linux-headers-generic dkms
    * restart
    * run insert & run guest additions
    * maybe install virtualbox-guest-dkms
    * check if shared clipboard works
  * enable root user : sudo passwd root
  * apt-get purge zeitgeist* aisleriot* deja-dup* cheese* firefox* image-magic* libreoffice* gnome-mahjongg gnome-mines seahorse remmina* rhythmbox* shotwell* gnome-sudoku thunderbird* transmission* vim* simple-scan*

# ros2 #
https://github.com/ros2/ros2/wiki/Linux-Install-Debians
* install binaries
  * apt update && sudo apt install curl
  * curl http://repo.ros2.org/repos.key | sudo apt-key add -
  * sudo sh -c 'echo "deb http://repo.ros2.org/ubuntu/main xenial main" > /etc/apt/sources.list.d/ros2-latest.list'
  * SNAPSHOT
  * apt update
  * apt install `apt list ros-r2b3-* 2> /dev/null | grep "/" | awk -F/ '{print $1}' | grep -v -e ros-r2b3-ros1-bridge -e ros-r2b3-turtlebot2- | tr "\n" " "`
  * source /opt/ros/r2b3/setup.bash
  * append "source /opt/ros/r2b3/setup.bash" in .bashrc


* building from source
  * apt-get install git wget
  * apt-get install build-essential cppcheck cmake libopencv-dev libpoco-dev libpocofoundation9v5 libpocofoundation9v5-dbg python-empy python3-dev python3-empy python3-nose python3-pip python3-setuptools python3-vcstool python3-yaml libtinyxml-dev libeigen3-dev
  * apt-get install clang-format pydocstyle pyflakes python3-coverage python3-mock python3-pep8 uncrustify
  * pip install --upgrade pip
  * pip install argcomplete
  * activate-global-python-argcomplete
  * source /opt/ros/r2b3/share/ros2cli/environment/ros2-argcomplete.bash
  * pip3 install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes
  * apt-get install libasio-dev libtinyxml2-dev
  * apt-get install libopensplice67