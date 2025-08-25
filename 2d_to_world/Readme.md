# 2d_to_world: Generate Gazebo Harmonic Worlds from 2D Occupancy Maps

## Overview

The `2d_to_world` package is a ROS 2 utility that automates the creation of Gazebo Harmonic world files from 2D occupancy grid maps (e.g., PGM/PNG images). Its primary functions are:

1.  **Converts 2D Maps to 3D Worlds**: Transforms 2D occupancy grid data into 3D wall structures within a Gazebo SDF world file.
2.  **Automates Environment Setup**: Eliminates manual Gazebo world building, enabling rapid simulation environment creation from LIDAR-generated maps.
3.  **Configurable Output**: Allows customization of wall height, thickness, and other parameters to suit various simulation needs.
   
## How 2D Map is Converted to Gazebo World

The `2d_to_world` package converts a 2D occupancy grid map into a 3D Gazebo world by first loading the map image and its metadata, then creating a binary mask to identify occupied areas. It extracts and simplifies contours from this mask, converting them to real-world coordinates. These contours are then broken into straight segments, which are merged if co-linear to form efficient wall representations. Finally, for each wall segment, the package generates corresponding 3D box models in Gazebo SDF format, positioned and oriented to match the 2D map, and outputs a complete Gazebo world file ready for simulation.

![](https://github.com/Divya777777/2d_map_to_gazebo_world/blob/humble/Gifs/2d_to_world.gif)

## Usage Steps

### 1. Prerequisites & Dependencies

Ensure you have ROS 2 (Humble, Iron, Jazzy, or compatible) and Gazebo Harmonic installed. This package requires several Python libraries and ROS 2 packages. You can install the Python dependencies using pip:

```bash
pip install numpy Pillow scikit-image scipy PyYAML
```

### 2. Building the Package

1.  **Create a ROS 2 workspace (if needed):**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
2.  **Clone the repository into your `src` folder:**
    ```bash
    git clone https://github.com/Divya777777/2d_map_to_gazebo_world.git
    ```
3.  **Build the package:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select 2d_to_world
    ```
4.  **Source your workspace:**
    ```bash
    source install/setup.bash
    ```

### 3. Prepare Your Map Files

Place your 2D occupancy map files (`.pgm` or `.png` image and its corresponding `.yaml` metadata file) in a known location.

### 4. Generate the Gazebo World

Use the provided ROS 2 launch file, specifying your map and desired output path:

```bash
ros2 launch 2d_to_world 2D_to_world.launch.py \
    map_path:=/path/to/your/maps/office_2.yaml \
    output_world:=/path/to/generate/world/my_generated_world.world \
    wall_height:=2.2 \
    wall_thickness:=0.15
```

Replace `office_2.yaml` with your map file and `~/my_generated_world.world` with your desired output path and filename. Adjust `wall_height` and `wall_thickness` as needed.

### 5. Open in Gazebo

Once the world file is generated, open it with Gazebo Harmonic:

```bash
gz sim /path/to/generated/world/my_generated_world.world
```



