## Approach

### Obtaining the intrinsic parameters of the ZED2 camera

We used the `wait_for_message` function to create a new subscription to the `/zed2/zed_node/rgb/camera_info` topic, receive one message and then unsubscribe. The intrinsic matrix $K$ and the image dimensions can be obtained by reading the contents of the obtained message.

### Computing the lidar to ZED2 transform

A dataset of paired images and lidar scans is captured by placing different obtacles (eg: stop sign, foam cubes, traffic cone, etc) in front of the vehicle.   

The depth information captured by the camera is used to create a point cloud. This stereo point cloud is matched with the lidar point cloud using the Iterative Closest Point (ICP) registration algorithm. The initial transformation matrix is created based on the directions of the X, Y and Z axes of the lidar and the zed camera. Since, the zed frame is defined with the Z axis pointing forward, X to the right, and Y pointing down; the rotation component is initialized as:

```math
R_{init} = \begin{pmatrix}
                0 & -1 & 0 \\\
                0 & 0 & -1 \\\
                1 & 0 & 0
           \end{pmatrix}
```

The translation component of the transform is set using a trial and error method (by viewing if points of interest in the lidar point cloud map to appropriate 2D points in the image plane).

### Computing the lidar to vehicle transform

#### Measuring roll

<img align = "right"
    src = "https://github.com/krishauser/GEMstack/blob/s2024_group3/GEMstack/offboard/calibration/figures/roll.png"
    style = "width: 400px"
/>

The roll of the lidar is calibrated with the help of a flat surface (eg: white board) placed parallel to the XZ plane of the vehicle. The angle between the Y axes (or Z axes) of the vehicle and the lidar, $\gamma$ is computed as follows:

- The unit normal of the white board ($\hat{n}$) is along the Y axis of the vehicle.
- If $\hat{y}$ is the unit vector along the Y axis of the lidar, then $\gamma = \cos^{-1}({\hat{n}} \cdot {\hat{y}})$

The rotation matrix is:

```math
R_x(\gamma) = \begin{pmatrix}
                1 & 0 & 0 \\\
                0 & \cos \gamma & -\sin \gamma \\\
                0 & \sin \gamma & \cos \gamma
              \end{pmatrix}
```

#### Measuring pitch

<img align = "right"
    src = "https://github.com/krishauser/GEMstack/blob/s2024_group3/GEMstack/offboard/calibration/figures/pitch.png"
    style = "width: 560px"
/>

The pitch of the lidar is measured by placing a flat surface (eg: white board) parallel to the YZ plane of the vehicle. The angle $\beta$ between the X axes (or Z axes) of the vehicle of the vehicle and the lidar is obtained as follows:

- The unit normal of the white board ($\hat{n}$) is along the X axis of the vehicle.
- If $\hat{x}$ is the unit vector along the X axis of the lidar, then $\beta = \cos^{-1}({\hat{n}} \cdot {\hat{x}})$.

The rotation matrix is:

```math
R_y(\beta) = \begin{pmatrix}
                \cos \beta & 0 & \sin \beta \\\
                0 & 1 & 0 \\\
                -\sin \beta & 0 & \cos \beta
              \end{pmatrix}
```

#### Measuring yaw

<img align = "right"
    src = "https://github.com/krishauser/GEMstack/blob/s2024_group3/GEMstack/offboard/calibration/figures/yaw.png"
    style = "height: 480px"
/>

Foam cubes are stacked on top of each other along the centerline of the vehicle. Let $P$ be the points in the point cloud representing the foam cubes. Let $(x_l, y_l, z_l) \in P$ be the coordinates (in the lidar frame) of the point along the lidar's XY plane closest to the vehicle's XZ plane. If $d$ is the distance between the lidar and P, the angle between the X axes of the vehicle and the lidar, $\alpha$ can be computed as follows:

$$ x_l = d \cos \alpha $$

$$ y_l = -d \sin \alpha $$

$$ z_l \approx 0 $$

```math
\alpha = -\tan^{-1} (y_l / x_l)
```

The rotation matrix is given by:

```math
R_z(\alpha) = \begin{pmatrix}
                \cos \alpha & -\sin \alpha & 0 \\\
                \sin \alpha & \cos \alpha & 0 \\\
                0 & 0 & 1
              \end{pmatrix}
```

#### Rotation matrix

A single rotation matrix is obtained by multiplying the yaw, pitch and roll rotation matrices.

```math
R(\alpha, \beta, \gamma) = R_z(\alpha) \: R_y(\beta) \: R_x(\gamma)
```

#### Measuring translation

The translation vector is $t = [t_x, t_y, t_z]^{T}$ where:
- $t_x$ is the distance between the center of the rear axle and the lidar, measured along the X axis of the vehicle.
- $t_y$ is $0$ as the lidar is placed along the vehicle's XZ plane.
- $t_z$ is the distance between the rear axle and the lidar, measured along the vehicle's Z axis.

The resulting transformation matrix is:

```math
T_{lidar}^{vehicle} = \begin{pmatrix}
                        R_{0,0} & R_{0,1} & R_{0,2} & t_x \\\
                        R_{1,0} & R_{1,1} & R_{1,2} & 0 \\\
                        R_{2,0} & R_{2,1} & R_{2,2} & t_z \\\
                        0 & 0 & 0 & 1
                      \end{pmatrix}
```


### Computing the ZED2 to vehicle transform

The transformation that converts points from the zed frame to the vehicle frame is computed using the previous 2 matrices as:

```math
T_{zed}^{vehicle} = T_{lidar}^{vehicle} \: \left( T_{lidar}^{zed} \right)^{-1}
```


## Usage instructions

- Extract `data/calibration1.zip` and `data/calibration2.zip`.
- Run `python3 testing/test_calibration.py` from the main GEMstack folder. 
- Four options (1 through 4) will be displayed. 
- Enter choice to run the corresponding calibration code.

#### Choices:

1. Obtain ZED2 intrinsics
    - The message received is processed to obtain the intrinsic matrix $K$, the height and width of the image.
    - The parameters are written to `GEMstack/knowledge/calibration/gem_e2_zed_intrinsics.yaml`.

2. Compute Velodyne to ZED2 transform
    - $T_{zed}^{vehicle}$ is computed and written to `GEMstack/knowledge/calibration/gem_e2_transforms.yaml`.
    - An Open3D visualization will open up with the lidar point cloud corresponding to scan 9 in the `data/calibration1` dataset.
    - Hold <kbd>Shift</kbd> and drag the mouse to draw a rectangle around the points of interest. For example, in the figure below, points in the stop sign and the foam cubes are selected (in green). Close the window.
        
        <p align="center">
        <img
            src = "https://github.com/krishauser/GEMstack/blob/s2024_group3/testing/figures/hw3_step1_pcd.png"
            style = "width: 800px"
        />
        </p>

    - The selected lidar points are projected onto the zed frame using $T_{lidar}^{zed}$. The projected points are then conveted to pixel coordinates using the intrinsic matrix.
    - The projected image pixels are colored yellow as shown.

        <p align="center">
        <img
            src = "https://github.com/krishauser/GEMstack/blob/s2024_group3/testing/figures/hw3_step1_proj.png"
            style = "width: 1000px"
        />
        </p>

3. Compute Velodyne to vehicle transform    
    - An Open3D visualization will open up with the lidar point cloud corresponding to scan 5 in the `data/calibration2` dataset.
    - Select the points for the white board placed *parallel* to the vehicle. Close the window.

        <p align="center">
        <img
            src = "https://github.com/krishauser/GEMstack/blob/s2024_group3/testing/figures/hw3_step2_pcd1.png"
            style = "width: 800px"
        />
        </p>

    - Another visualization will open up with the point cloud corresponding to scan 6.
    - Select the points for the white board *in front* of the vehicle and close the window.

        <p align="center">
        <img
        src = "https://github.com/krishauser/GEMstack/blob/s2024_group3/testing/figures/hw3_step2_pcd2.png"
        style = "width: 800px"
        />
        </p>

    - The third visualization will open up with the point cloud corresponding to scan 8.
    - Select the points for the foam cubes placed along the centerline and close the window.

        <p align="center">
        <img
            src = "https://github.com/krishauser/GEMstack/blob/s2024_group3/testing/figures/hw3_step2_pcd3.png"
            style = "width: 800px"
        />
        </p>

    - The rotation matrix will then be computed based on the selection.
    - The translation matrix is computed based on the physical measurements taken from the vehicle.
    - The matrices are written to `GEMstack/knowledge/calibration/gem_e2_transforms.yaml`.

4. Compute ZED2 to vehicle transform
    - $T_{lidar}^{zed}$ and $T_{lidar}^{vehicle}$ are read from `GEMstack/knowledge/calibration/gem_e2_transforms.yaml`.
    - The program computes and writes $T_{zed}^{vehicle}$ to file.

The values of the transforms are then written to the files referenced by `GEMstack/knowledge/calibration/gem_e2.yaml`.