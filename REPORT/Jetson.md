#  Setup and Operation Guide: Jetson & RC Jimny

This wiki outlines the complete steps to connect the Jetson hardware, configure the SSH network from your laptop, and launch the necessary nodes to control the vehicle.

---

##  1. Hardware Connections

Before powering on the system, make sure the following components are properly connected. The connection order for some components is crucial for a successful boot.

![Jetson ports view](images/WhatsApp%Image%2026-07-21%at%12.27.33.jpeg)


1. **HDMI Module (Dummy Plug):** Connect this **first**. It is essential to trick the Jetson into booting properly without a physical monitor (headless mode).
2. **Main Power:** Connect the power source to the Jetson. 
   *⚠️ Warning: Ensure the Jetson battery is fully charged (above 24V, which will last about 2-3 hours). If the voltage drops below 21V, the battery is fully drained.*
3. **Cube Orange:** Plug the Cube Orange USB cable into the **vertical slot** of the Jetson.
4. **Additional Boards (Steering-board and Sterfboard):** Plug the two USB cables into the horizontal ports (one top and one bottom). **The physical order does not matter**: the correct assignment will be checked and managed via software from the terminal.

![Jetson connected to power](images/WhatsApp%Image%2026-07-21%at%12.27.34%(4).jpeg)


---

##  2. Accessing the Jetson (SSH or via Ethernet)

All the working repository is currently on the Jetson. To control it, we will use an SSH connection or via an Ethernet cable from your laptop. 

The following commands apply if you are running **Ubuntu 22.04 LTS (Jammy Jellyfish)** on your laptop.

### Step 1: Find your laptop's Ethernet interface name
Open your laptop's terminal and run:
```bash
nmcli device
```
*Note down your Ethernet interface name (e.g., `enp3s0`, `eth0`).*

### Step 2: Create the Jetson Network Profile
Run this command to create a permanent network profile that forces your laptop to act as the master with IP `192.168.50.1`. 
(Replace `<INTERFACE_NAME>` with the name you found in Step 1!)
```bash
sudo nmcli con add type ethernet con-name "Jetson-Direct" ifname <INTERFACE_NAME> ipv4.method manual ipv4.addresses 192.168.50.1/24
```

### Step 3: Wake up the port
Force your laptop to apply the new profile:
```bash
sudo nmcli con up "Jetson-Direct"
```

### Step 4: SSH into the Jetson
Now that the pipeline is open, access the Jetson. The password is `admin`:
```bash
ssh user@192.168.50.2
```

---

##  3. Boot Procedure

Follow this order **strictly** to start the vehicle safely:

1. Connect the main power source coming from the car's battery to the boards and drivers.
2. Check the voltage and power on the Jetson.
3. Connect the Ethernet cable from your computer to the Jetson.
4. Switch the ignition ON on the Jimny, but **do not start the engine yet**.
5. Open **4 separate terminals** on your laptop and SSH into the Jetson in each one.

### USB Ports Verification
Before running the ROS nodes, you must check which logical ports the system assigned to the horizontal USB boards. In one of the terminals, run:
```bash
ls /dev/tty*CH* /dev/ttyUSB*
```
*📌 Note: The correct port for the steering is usually `USB0`. If the output shows a different assignment, change the `USB0` or `USB1` strings in the commands below to the right port.*

### Running ROS2 Nodes

Run one of the following command blocks in each of the 4 opened terminals.

**Terminal 1 (Cuberos):**
```bash
cd ~/workspaces/jimny/jimny_ws
ros2 run cuberos cuberos_node.py
```
*(Note: expect a "no message received" message, this is fine).*

**Terminal 2 (To Vehicle):**
```bash
cd ~/workspaces/jimny/jimny_ws
ros2 run to_vehicle to_vehicle /dev/ttyUSB1
# Use the line below if you have a CH341 chip:
# ros2 run to_vehicle to_vehicle /dev/ttyCH341USB1
```

**Terminal 3 (Steering Serial):**
```bash
cd ~/ackermann_to_steering_serial
python3 ackermann_to_steering_serial_final.py /dev/ttyUSB0
# Use the line below if you have a CH341 chip:
# python3 ackermann_to_steering_serial_final.py /dev/ttyCH341USB0
```

**Terminal 4 (Mode Switching):**
```bash
cd ~/workspaces/jimny/jimny_ws
ros2 run mode_switching switch
```

---

## 🏁 4. Final Startup and Safe Shutdown

### Ready to drive
Once the 4 terminals are running without errors:
1. Start the car engine.
2. Turn on the Herelink Handheld controller.
3. Select **Manual mode**.
4. **Arm**.

### 🛑 Safe Shutdown (CRUCIAL)
To avoid OS corruption on the Jetson, **ALWAYS do this before disconnecting power from the Jetson**. 

Go to one of the SSH terminals and run:
```bash
sudo poweroff
```
Wait for the Jetson LEDs to turn off completely before disconnecting the power source.
## 🛠 5. Troubleshooting & To-Do: Missing CH341 Module

**Issue:** If the `/dev/ttyCH341*` ports are missing, the `ch341.ko` driver module likely dropped after a reboot. Currently, it is not persistent and must be reloaded every time the system starts because it was never installed in `/lib/modules` or added to the boot sequence.

### 1. Verify the module status
Run this command to check if the module is currently loaded:
```bash
lsmod | grep ch341
ls /dev/ttyCH341* /dev/ttyUSB*
```

### 2. Temporary Fix (Manual Reload)
If `lsmod` shows nothing, the module was unloaded. Reload it manually:
```bash
cd ~/ch341ser_linux-main/driver
sudo insmod ch341.ko
ls /dev/ttyCH341*
```

### 3. Permanent Fix (To-Do: Make it load on boot)
To avoid having to reload the driver manually every time, install the module into the system so it loads automatically at boot:
```bash
cd ~/ch341ser_linux-main/driver
sudo make install
```
*If `make install` is not supported by the Makefile or doesn't work, run these commands to do it manually:*
```bash
sudo cp ch341.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
sudo depmod -a
echo "ch341" | sudo tee -a /etc/modules-load.d/ch341.conf
```
