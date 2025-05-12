# Mac Setup Instructions for GEMstack

This guide will help you set up GEMstack on a Mac computer using UTM to run Ubuntu 20.04 in a virtual machine.

## Resources

- [YouTube Tutorial: Ubuntu on M1 Macs](https://www.youtube.com/watch?v=MVLbb1aMk24)
- [UTM App Download](https://mac.getutm.app)
- [Ubuntu 20.04.5 ISO Download](https://old-releases.ubuntu.com/releases/20.04.5/)
- [ROS Noetic Installation](https://wiki.ros.org/ROS/Installation/TwoLineInstall)
- [GEMstack Repository](https://github.com/krishauser/GEMstack/tree/main)

## Ubuntu Installation and Setup

1. **Download and Install UTM**
   - Download UTM dmg file from [mac.getutm.app](https://mac.getutm.app)
   - Install the application

2. **Download Ubuntu ISO**
   - Download `ubuntu-20.04-live-server-arm64.iso` from [Ubuntu 20.04.5 releases](https://old-releases.ubuntu.com/releases/20.04.5/)
   - You will need to scroll down in the website to find the correct file

3. **Create a New Virtual Machine**
   - Open UTM app
   - Click "Create a new virtual machine"
   - Click "Virtualize"
   - Click "Linux"
   - Choose the Ubuntu ISO image file from your finder
   - Adjust the hardware (RAM and CPU cores) if desired (defaults work fine)
   - Adjust storage size (default works fine)
   - Click "Save"

4. **Install Ubuntu**
   - Click the play button to start the VM
   - Choose "Install Ubuntu Server"
   - Go through the Ubuntu setup (you can choose default/done for all options)
   - Make sure to install OpenSSH server
   - You can choose which packages to install (not necessary)
   - Wait for the installation/updates to finish (this step can take 10-30 minutes)
   - Choose "Reboot Now"

   > **Note:** This can get you to a frozen blinking cursor screen with no progress. That's ok, it's a known bug. To fix, click option + f1. You will see a message that says: "Please remove the installation medium, then reboot".

5. **Remove Installation Medium**
   - Click the shut down button (located in the menu bar) to power off the VM
   - Go back to the main UTM screen and find your VM on the left
   - Right-click the VM and click "Edit"
   - Choose the USB Drive and delete it
   - Click save to save your changes
   - Start the VM again

6. **Install Ubuntu Desktop Environment**
   - Enter your username and password
   - Run the following commands in the terminal:
     ```
     sudo apt install tasksel
     sudo tasksel install ubuntu-desktop  # this step will take some time to run
     sudo reboot
     ```

7. **Enable Clipboard and Directory Sharing**
   - Run the following command:
     ```
     sudo apt install spice-vdagent spice-webdavd
     ```
   - Open Software Updater and install any available updates

## Installing ROS and GEMstack

1. **Install ROS Noetic**
   - Follow the [ROS Noetic Installation Guide](https://wiki.ros.org/ROS/Installation/TwoLineInstall)
   - Or run these commands:
     ```
     sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
     sudo apt install curl
     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
     sudo apt update
     sudo apt install ros-noetic-desktop-full
     echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
     source ~/.bashrc
     ```

2. **Clone GEMstack Repository**
   - Run:
     ```
     git clone https://github.com/krishauser/GEMstack.git
     cd GEMstack
     ```

3. **Install GEMstack Dependencies**
   - Run:
     ```
     pip install -r requirements.txt
     ```

4. **Set up the Development Environment**
   - Follow the instructions in the main README.md file for additional setup steps

## Troubleshooting

If you encounter performance issues:
- Try allocating more RAM and CPU cores to the VM
- Disable unnecessary visual effects in Ubuntu
- Make sure you have enough free disk space on both your Mac and in the VM 