# MIDTERM 

## How to use this package
>Note: This package was test on ros2 humble 
1. Clone respitory
    ```bash
    cd ~
    git clone https://github.com/phattanaratjeedjeen-sudo/FRA631.git
    cd ~/FRA631
    ```
2. Build workspace
    ```bash
    colcon build && source install/setup.bash
    ```
3. Environment setup
    ```bash
    echo "source ~/FRA631/install/setup.bash" >> ~/.bashrc  
    source ~/.bashrc
    ```
4. Usage
    ```bash
    ros2 launch arm_description bringup.launch.py 
    ```
    
    Set task to `rectangle` for `joint space` trajectory
    ```bash
    # On new terminal
    ros2 param set /joint_space/arm task rec
    ```

    Set task to `triangle` for `task space` trajectory
    ```ash
    # On new terminal
    ros2 param set /task_space/arm task tri
    ```
    Or viseversa also available

## DH Parameter

<p align="center">
  <img src="./images/DH.jpeg" width="100%"/>
</p>


| $i-1$ | $i$ | $d_i$ | $\theta_i$ | $a_i$ | $\alpha_i$ |
|---|---|---|---|---|---|
| 0 | 1 | $L_0$ | $\theta_1$ | 0 | $\pi/2$
| 1 | 2 | 0 | $\theta_2$ | $L_1$ | 0
| 2 | 3 | 0 | $\theta_3 + pi/2$ | 0 | $\pi/2$
| 3 | 4 | $L_2$ | $\theta_4$ | 0 | $-\pi/2$
| 4 | 5 | 0 | $\theta_5$ | 0 | $\pi/2$
| 5 | 6 | 0 | $\theta_6$ | 0 | 0

where
- $L_0$ = 0.1 m
- $L_1$ = 0.6 m
- $L_2$ = 0.8 m
## Example Trajectory Calculation


## Result
**Triangle**

<table align="center">
  <tr>
    <td align="center" width="50%">
      <img src="./images/strl.png" width="100%"/>
      <br>
      <em>Trajectory Comparison</em>
    </td>
    <td align="center" width="45%">
      <img src="./media/wind_strl_2d.gif" width="100%"/>
      <br>
      <em>Wind Simulation</em>
    </td>
  </tr>
</table>

**Rectangle**

<table align="center">
  <tr>
    <td align="center" width="50%">
      <img src="./images/strl.png" width="100%"/>
      <br>
      <em>Trajectory Comparison</em>
    </td>
    <td align="center" width="45%">
      <img src="./media/wind_strl_2d.gif" width="100%"/>
      <br>
      <em>Wind Simulation</em>
    </td>
  </tr>
</table>