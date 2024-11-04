## Installing ubuntu for ROS2
There are different options , but suggested ones are -
1. VMware workstation (good for 3d visualization tool)
2. Docker
### Check installation of ROS2 in your system
```bash
cat .bashrc
```
- **`.bashrc`**:  is the configuration in home. Built for Unix like OS, This is a hidden file located in your home directory (denoted by the dot at the beginning of its name) that contains configuration settings for your bash shell. It typically includes environment variables, aliases, and functions that customize your shell experience.

- **`cat`**: This stands for "concatenate" and is a command-line utility used to read and display the contents of files. It can also be used to concatenate multiple files into one or create new files.


#### Terminator , for using multiple commands lines


## Commands for ros2

```bash
gedit ~/.bashrc

edit configuration file

source .bashrc

refresh the command line


printenv | grep TURTLE

printing the environment, the turtle 

gazebo

to run gazebo

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

to launch turtle in gazebo
```


In gazebo default Lidar has 4m of range

```Shell
ros2 run turtlebot3_teleop teleop_keyboard

to launch the keyboard control of turtlebot

ros2 topic list

for viewing the ros2 communciations

rqt_graph


```

```shell
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

to launch rviz for SLAM
```



```shell
mkdir maps
ros2 run nav2_map_server map_saver_cli -f maps/my_map

to save maps

nano my_map.pgm

to open what inside the file
```


## Meaning of my_map.pgm
  GNU nano 6.2                              my_map2.pgm                                       
P5
279 237   __pixel x and pixel y__
255
���������������������������������������������������������������������������������������������>

#### meaning of above
The text you've provided appears to be a partial representation of a PGM (Portable Gray Map) file in the P5 format. The P5 format is a binary representation of grayscale images, which is often used for occupancy maps in robotics and computer vision. Let's break down the components you've shared:

### Breakdown of the PGM File

### 1. **Header Information**
   - **P5**: This indicates that the image is in binary PGM format. The "P5" signifies that it is a binary format, as opposed to "P2," which would indicate ASCII.
   - **279 237**: These numbers represent the width and height of the image in pixels, respectively. In this case, the image is 279 pixels wide and 237 pixels tall.
   - **255**: This value indicates the maximum gray value for the image. In a PGM file, pixel values range from `0` (black) to `255` (white). Therefore, `255` means that the image can represent 256 different shades of gray.

### 2. **Pixel Data**
   - The section that follows the header contains the pixel data for the image. In binary PGM files (P5), this data is stored in binary format rather than ASCII characters.
   - The string of characters (represented by `��������������������������������������������������������������������������������������>`) indicates that there are pixel values following the header. Each pixel value will be represented as a single byte (0-255), corresponding to its grayscale intensity.

### Understanding the Pixel Data
- Each pixel's intensity will be read as a byte following the header, where:
  - `0` represents black,
  - `255` represents white,
  - Values in between represent varying shades of gray.
  
In a typical occupancy grid context, these pixel values would correspond to whether areas are occupied, free, or unknown based on the thresholds you provided earlier.

#### Summary
This snippet represents a binary PGM file with dimensions of 279x237 pixels and a maximum gray value of 255. The actual pixel data follows this header and is crucial for interpreting the occupancy information encoded within the map.

If you need to process this file further or have specific questions about how to use it, please let me know!

It looks like you are working with a map file, specifically in the PGM (Portable Gray Map) format, which is commonly used in robotics and computer vision for representing occupancy grids. Let's break down the components of your input:

### Components Explained

### 1. **Image: my_map2.pgm**
   - This indicates the filename of the map image. The `.pgm` format is a grayscale image format that can represent occupancy information.

### 2. **Mode: trinary**
   - The "trinary" mode suggests that the occupancy grid uses three states: occupied, free, and unknown. This is useful in robotics for distinguishing between areas that are occupied by obstacles, areas that are free for navigation, and areas where no information is available.

### 3. **Resolution: 0.05**
   - The resolution of `0.05` likely refers to the size of each pixel in the map, indicating that each pixel represents a physical area of 5 cm x 5 cm in the real world. This is important for understanding the scale when interpreting the map.

### 4. **Origin: [-3.35, -5.5, 0]**
   - The origin specifies the coordinates of the lower-left corner of the map in a Cartesian coordinate system. In this case, it indicates that the origin is located at (-3.35, -5.5) with a height (z-coordinate) of 0, which typically represents a 2D plane.

### 5. **Negate: 0**
   - The `negate` parameter indicates whether to invert the occupancy values in the map. A value of `0` means that no negation will be applied; thus, occupied areas remain occupied.

### 6. **Occupied Thresh: 0.65**
   - This threshold defines the cutoff for determining whether a cell in the map is considered occupied. If the value at a cell exceeds `0.65`, it will be marked as occupied.

### 7. **Free Thresh: 0.25**
   - Conversely, this threshold indicates when a cell is considered free. If the value at a cell is below `0.25`, it will be marked as free.

## Summary
This configuration describes how to interpret a PGM occupancy grid map for navigation or environmental mapping tasks in robotics or related fields. The trinary mode allows for nuanced representation of space, and the thresholds help determine how to classify different areas based on sensor data or other inputs.

If you have any specific questions about any part of this configuration or need further assistance, feel free to ask!



------------------------------------
