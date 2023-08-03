# Indoor-Flight
This repo provides documentation/tutorials of setup for indoor flight with OptiTrack motion capture system in offboard mode of pixhawk.

### ODROID XU4 Setup

#### Static ip for ODROID XU4

It is recommended to use static ip address if you plan to use ODROID via a WiFi network. Follow these steps for setting the static ip:

Check the WiFi card number with the following command
`ifconfig -a`
To set a static IP address open `/etc/network/interfaces` file for editing by the following command
`sudo nano /etc/network/interfaces`
Add or edit following lines to the file, and make sure it matches your WiFi network. Added lines should look similar to this.

```
auto wlan0 # The following will auto-start connection after boot
allow-hotplug wlan0 # wlan0 WiFi card number
iface wlan0 inet static
address 192.168.0.xxx # Choose a static IP, usually you change the last number only for different devices
netmask 255.255.255.0
broadcast 192.168.0.255
gateway 192.168.0.1 # Your router IP address
dns-nameservers 8.8.8.8
wpa-ssid "DroneLab" # WiFi name (case sensitive)
wpa-psk "12345dronelab" # WiFi password
```

#### Install ROS

To install ROS noetic on ODROID or ARM-based single-board-computer, follow these [instructions](http://wiki.ros.org/noetic/Installation/Ubuntu) available on the ROS website.
Make sure to install the `ROS-Base: (Bare Bones)` and not the full desktop version.

So when you reach the step to install ROS with apt get  don't execute `sudo apt install ros-noetic-desktop-full`, instead use `sudo apt install ros-noetic-ros-base`.

#### Install MAVROS

This package is used to interface MAVLink-based autopilots to ROS.

We will simply follow the well documented wiki on MAVROS github page. For the source installation while building the packages with catkin build make sure to use the argument -j1. So you execute `catkin build -j1`

Check if mavros works fine, e.g. `roslaunch mavros px4.launch`

#### Install vrpn_client_ros ROS package

In order to recieve data in ROS from an OptiTrack system running the Motive software, VRPN streaming engine with the [vrpn-client-ros](https://wiki.ros.org/vrpn_client_ros) ROS package can be used.

The following describes the ways to setup and install the package

`cd ~/catkin_ws/src`

`git clone -b kinetic-devel https://github.com/ros-drivers/vrpn_client_ros`

`rosdep install --from-paths src/vrpn_client_ros --ignore-src`

`cd ~/catkin_ws`

`catkin build`

### Motion Capture System Setup : OptiTrack 

#### Basic setup

To create a rigid body for streaming, select the markers of the object (at least three) and right-click in Motive and select `Rigid Body -> Create From Selected Markers`. Make sure the `Name` for the object to be tracked does not have any whitespaces.

<img src="/images/asset_pane.png" width="200">

Then you can see a new rigid body in the `Assests` pane. Note that only those objects would be streamed where the checkbox is checked in the Assests pane. 

<img src="/images/broadcast_frame.png" width="200">

Activate the checkbox for `Broadcast Frame Data` from the `Streaming` pane. 

<img src="/images/motive .png" width="550">
### Feeding MOCAP data into Pixhawk

You need to set your flight controller firmware PX4, to accept mocap data. PX4 has two state estimators, EKF2(default) an extended Kalman filter, and LPE. 

#### EKF2 parameters update

* `SYS_COMPANION` = Companion Link (921600 baud, 8N1)
* `MAV_1_CONFIG` = TELEM 2
* `MAV_1_MODE` = Onboard
* `SER_TEL2_BAUD` = 921600
* `EKF2_AID_MASK` = 24
Set EKF2_AID_MASK to use vision position fusion and vision yaw fusion.
* `EKF2_HGT_MODE` = Vision
* `EKF2_ASP_DELAY` = 0.0 ms
* `EKF2_EV_DELAY` = 50.0 ms
* `EKF2_OF_DELAY` = 0.0 ms

#### Getting pose data into ROS

Run the `vrpn_client_node` as follows

`roslaunch vrpn_client_ros sample.launch server:=192.168.0.101`

If you named the rigidbody as `robot1`, you will get the MoCap data on the topic like `/vrpn_client_node/robot1/pose`.

#### Relaying/remapping the pose data

Run the MAVROS node

`roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:230400 gcs_url:=udp://@192.168.1.54:14550`

MAVROS provides a plugin to relay pose data published on `/mavros/vision_pose/pose` to PX4. Assuming that MAVROS is running, you just need to remap the pose topic that you get from MoCap `/vrpn_client_node/<rigid_body_name>/pose` directly to `/mavros/vision_pose/pose`. 

`rosrun topic_tools relay /vrpn_client_node/droneLOP/pose /mavros/vision_pose/pose`

####  Running the node for offboard mode control

`rosrun offboard_py offb_node.py`

### Results

#### Position Setpoints
![offboard gif](/results/offboard.gif)

<p align = "left">
&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;x = $0$, y = $0$, z = $1.5$
</p>

#### Square Trajectory
![square gif](/results/square.gif)

### Troubleshooting

```Errors     << mavros:make /home/odroid/catkin_ws/logs/mavros/build.make.002.log
c++: fatal error: Killed signal terminated program cc1plus
compilation terminated.
make[2]: *** [CMakeFiles/mavros.dir/build.make:154: CMakeFiles/mavros.dir/src/lib/uas_data.cpp.o] Error 1
make[2]: *** Waiting for unfinished jobs....
c++: fatal error: Killed signal terminated program cc1plus
compilation terminated.
make[2]: *** [CMakeFiles/mavros.dir/build.make:128: CMakeFiles/mavros.dir/src/lib/mavros.cpp.o] Error 1
In file included from /usr/include/c++/9/unordered_map:47,
                 from /home/odroid/catkin_ws/src/mavros/mavros/src/lib/enum_sensor_orientation.cpp:17:
/usr/include/c++/9/bits/unordered_map.h: In constructor ‘std::unordered_map<_Key, _Tp, _Hash, _Pred, _Alloc>::unordered_map(std::initializer_list<typename std::_Hashtable<_Key, std::pair<const _Key, _Tp>, _Alloc, std::__detail::_Select1st, _Pred, _Hash, std::__detail::_Mod_range_hashing, std::__detail::_Default_ranged_hash, std::__detail::_Prime_rehash_policy, std::__detail::_Hashtable_traits<std::__not_<std::__and_<std::__is_fast_hash<_Hash>, std::__is_nothrow_invocable<const _Hash&, const _Key&> > >::value, false, true> >::value_type>, std::unordered_map<_Key, _Tp, _Hash, _Pred, _Alloc>::size_type, const hasher&, const key_equal&, const allocator_type&) [with _Key = unsigned char; _Tp = const std::pair<const std::__cxx11::basic_string<char>, const Eigen::Quaternion<double> >; _Hash = std::hash<unsigned char>; _Pred = std::equal_to<unsigned char>; _Alloc = std::allocator<std::pair<const unsigned char, const std::pair<const std::__cxx11::basic_string<char>, const Eigen::Quaternion<double> > > >]’:
/usr/include/c++/9/bits/unordered_map.h:227:7: note: parameter passing for argument of type ‘std::initializer_list<std::pair<const unsigned char, const std::pair<const std::__cxx11::basic_string<char>, const Eigen::Quaternion<double> > > >’ changed in GCC 7.1
  227 |       unordered_map(initializer_list<value_type> __l,
      |       ^~~~~~~~~~~~~
/usr/include/c++/9/bits/unordered_map.h:232:40: note: parameter passing for argument of type ‘std::initializer_list<std::pair<const unsigned char, const std::pair<const std::__cxx11::basic_string<char>, const Eigen::Quaternion<double> > > >’ changed in GCC 7.1
  232 |       : _M_h(__l, __n, __hf, __eql, __a)
      |                                        ^
In file included from /usr/include/c++/9/unordered_map:46,
                 from /home/odroid/catkin_ws/src/mavros/mavros/src/lib/enum_sensor_orientation.cpp:17:
/usr/include/c++/9/bits/hashtable.h: In constructor ‘std::_Hashtable<_Key, _Value, _Alloc, _ExtractKey, _Equal, _H1, _H2, _Hash, _RehashPolicy, _Traits>::_Hashtable(std::initializer_list<_Value>, std::_Hashtable<_Key, _Value, _Alloc, _ExtractKey, _Equal, _H1, _H2, _Hash, _RehashPolicy, _Traits>::size_type, const _H1&, const key_equal&, const allocator_type&) [with _Key = unsigned char; _Value = std::pair<const unsigned char, const std::pair<const std::__cxx11::basic_string<char>, const Eigen::Quaternion<double> > >; _Alloc = std::allocator<std::pair<const unsigned char, const std::pair<const std::__cxx11::basic_string<char>, const Eigen::Quaternion<double> > > >; _ExtractKey = std::__detail::_Select1st; _Equal = std::equal_to<unsigned char>; _H1 = std::hash<unsigned char>; _H2 = std::__detail::_Mod_range_hashing; _Hash = std::__detail::_Default_ranged_hash; _RehashPolicy = std::__detail::_Prime_rehash_policy; _Traits = std::__detail::_Hashtable_traits<false, false, true>]’:
/usr/include/c++/9/bits/hashtable.h:492:7: note: parameter passing for argument of type ‘std::initializer_list<std::pair<const unsigned char, const std::pair<const std::__cxx11::basic_string<char>, const Eigen::Quaternion<double> > > >’ changed in GCC 7.1
  492 |       _Hashtable(initializer_list<value_type> __l,
      |       ^~~~~~~~~~
/home/odroid/catkin_ws/src/mavros/mavros/src/lib/enum_sensor_orientation.cpp: In function ‘void __static_initialization_and_destruction_0(int, int)’:
/home/odroid/catkin_ws/src/mavros/mavros/src/lib/enum_sensor_orientation.cpp:120:2: note: parameter passing for argument of type ‘std::initializer_list<std::pair<const unsigned char, const std::pair<const std::__cxx11::basic_string<char>, const Eigen::Quaternion<double> > > >’ changed in GCC 7.1
  120 | }};
      |  ^
make[1]: *** [CMakeFiles/Makefile2:1185: CMakeFiles/mavros.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
cd /home/odroid/catkin_ws/build/mavros; catkin build --get-env mavros | catkin env -si  /usr/bin/make --jobserver-auth=3,4; cd -

...............................................................................
Failed     << mavros:make                [ Exited with code 2 ]                
Failed    <<< mavros                     [ 3 minutes and 32.1 seconds ]        
[build] Summary: 3 of 4 packages succeeded.                                    
[build]   Ignored:   2 packages were skipped or are skiplisted.                
[build]   Warnings:  None.                                                     
[build]   Abandoned: None.                                                     
[build]   Failed:    1 packages failed.                                        
[build] Runtime: 4 minutes and 30.0 seconds total.
```


