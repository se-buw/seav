
Some times, algo fails to find the path. So, Nav2 , will then recovery behaviour program to find and fix the current issue in the path for robot.


- Behaviour server wil be called when there is issue. It tries to find soln or otherwise abort it. 



---

# Reading Nav2 Logs in Terminal

Nav2 logs are essential for understanding how the navigation stack operates and troubleshooting any issues in real-time. Here’s how to access and interpret these logs effectively.

## Step-by-Step Guide

### 1. Open the Terminal

To view Nav2 logs, start by opening the terminal on your system.

```shell
# Example command to open a terminal (Linux):
ctrl + alt + t
```

---

### 2. Start Navigation2

To see the logs in real time, launch Nav2 through your preferred ROS launch file.

```shell
# Command to launch Navigation2
ros2 launch nav2_bringup navigation_launch.py
```

> **Tip**: Make sure you’re in the appropriate workspace and that your ROS environment is sourced correctly. For example:
> ```shell
> source ~/ros2_ws/install/setup.bash
> ```

---

### 3. Filtering Logs

Nav2 generates a lot of information. Filtering logs can help you focus on what’s important.

#### Use Log Levels

Nav2 supports different log levels. You can set these levels to control the verbosity of the log output.

```shell
# Example for setting log level
ros2 launch nav2_bringup navigation_launch.py params_file:=path_to_your_params_file.yaml --ros-args --log-level INFO
```

**Log Levels Explained**:
- **DEBUG**: Shows all messages, useful for in-depth debugging.
- **INFO**: General operational information.
- **WARN**: Warnings that may need attention.
- **ERROR**: Errors that require immediate attention.
- **FATAL**: Severe errors, often stopping the navigation node.

#### Grep for Specific Logs

You can use `grep` to filter specific logs, for example, to focus only on warnings and errors.

```shell
# Example command to filter only warning and error logs
ros2 launch nav2_bringup navigation_launch.py | grep -E "WARN|ERROR"
```

> **Tip**: You can save filtered logs to a file:
> ```shell
> ros2 launch nav2_bringup navigation_launch.py | grep -E "WARN|ERROR" > nav2_warnings.log
> ```

---

### 4. Using Log Directories

Nav2 logs can also be stored in log directories for later analysis.

#### Locating Log Files

By default, ROS 2 logs are saved in the following directory:

```shell
~/.ros/log/latest/
```

> **Pro Tip**: Obsidian supports linking to files on your local system. You can create a quick-access link to your log directory:
> ```markdown
> [Open Nav2 Logs Directory](file:///home/your_username/.ros/log/latest/)
> ```

#### Listing Log Files

You can list all log files using:

```shell
ls ~/.ros/log/latest/
```

---

## Additional Tips for Managing Logs

### Adjusting Logging Format

For better readability, you can adjust the log format to include timestamps, node names, and severity levels. This is particularly useful when troubleshooting timing-sensitive issues.

```shell
ros2 launch nav2_bringup navigation_launch.py --ros-args --log-format '[{severity}] [{time}] [{name}]: {message}'
```

### Checkpoints for Efficient Log Analysis

#### [ ] **Identify Key Nodes**: Focus on critical nodes like `planner_server`, `controller_server`, `bt_navigator`, and `costmap`.
#### [ ] **Set Log Rotation**: Large log files can be overwhelming. Set up log rotation if you’re running long tests.
#### [ ] **Review Log History**: Obsidian allows tagging; consider tagging entries based on date or severity level for easy retrieval later.

---

## Summary of Common Commands

| Command                                  | Description                                     |
|------------------------------------------|-------------------------------------------------|
| `ros2 launch nav2_bringup navigation_launch.py` | Starts Navigation2 and shows real-time logs. |
| `grep "WARN"`                            | Filters logs to display warnings only.          |
| `ls ~/.ros/log/latest/`                  | Lists the latest log files.                     |
| `--log-format`                           | Adds format options to log entries.             |

---

## Resources for Further Reading

- **Nav2 Documentation**: [Navigation2 on ROS.org](https://navigation.ros.org/)
- **ROS 2 Logging Guide**: [ROS 2 Logging](https://docs.ros.org/en/foxy/Tutorials/Logging-and-logger-configuration.html)
- **ROS Command Line Tools**: [ROS CLI](http://wiki.ros.org/ROS/CommandLineTools)

> 📝 **Note**: Consider creating a checklist in Obsidian for log review routines or any recurring troubleshooting steps you frequently perform.

---

Here's an example Nav2 log illustrating a scenario where a robot moves from a starting point to a destination but encounters an obstacle and collides. This log example is formatted to highlight different log levels and relevant information, which can be especially helpful for troubleshooting.

---

# Example Nav2 Log: Collision Scenario

### Scenario Description
- The robot starts moving from **Point A** to **Point B**.
- The initial path is calculated successfully.
- Midway, the robot encounters an obstacle.
- The local planner attempts to avoid the obstacle but fails, resulting in a collision.

---

### Log Output

```plaintext
[INFO] [1638904950.000000] [bt_navigator]: Received goal at coordinates (5.0, 7.5)
[INFO] [1638904950.001000] [planner_server]: Generating global path from (0.0, 0.0) to (5.0, 7.5)
[DEBUG] [1638904950.500000] [planner_server]: Global path generated successfully with 30 waypoints.
[INFO] [1638904950.502000] [controller_server]: Beginning to follow path
[INFO] [1638904950.600000] [controller_server]: Current Position: (0.5, 1.0)
[INFO] [1638904951.100000] [controller_server]: Current Position: (1.0, 2.0)
[INFO] [1638904951.600000] [controller_server]: Current Position: (1.5, 3.0)
[WARN] [1638904952.100000] [controller_server]: Obstacle detected within inflation radius at (1.7, 3.1)
[INFO] [1638904952.102000] [controller_server]: Recomputing local path to avoid obstacle
[INFO] [1638904952.600000] [controller_server]: Current Position: (1.8, 3.5)
[ERROR] [1638904953.000000] [controller_server]: Collision detected at (1.9, 3.6)
[FATAL] [1638904953.001000] [bt_navigator]: Path execution failed due to collision
```

---

### Breakdown of Log Entries

| **Timestamp**          | **Log Level** | **Component**       | **Message**                                                                                       |
|------------------------|---------------|---------------------|---------------------------------------------------------------------------------------------------|
| `1638904950.000000`    | INFO          | bt_navigator        | Received goal at coordinates (5.0, 7.5).                                                          |
| `1638904950.001000`    | INFO          | planner_server      | Generating global path from (0.0, 0.0) to (5.0, 7.5).                                             |
| `1638904950.500000`    | DEBUG         | planner_server      | Global path generated successfully with 30 waypoints.                                             |
| `1638904950.502000`    | INFO          | controller_server   | Beginning to follow path.                                                                         |
| `1638904952.100000`    | WARN          | controller_server   | Obstacle detected within inflation radius at (1.7, 3.1).                                          |
| `1638904952.102000`    | INFO          | controller_server   | Recomputing local path to avoid obstacle.                                                         |
| `1638904953.000000`    | ERROR         | controller_server   | Collision detected at (1.9, 3.6).                                                                 |
| `1638904953.001000`    | FATAL         | bt_navigator        | Path execution failed due to collision.                                                           |

---

### Explanation

1. **Starting Goal**: 
   - The `bt_navigator` component receives the goal coordinates and hands off the goal to `planner_server`.
   - `planner_server` successfully generates the global path with 30 waypoints.

2. **Following Path**: 
   - The `controller_server` begins moving the robot, periodically logging the current position as it progresses along the path.

3. **Obstacle Detection**:
   - The `controller_server` detects an obstacle at `(1.7, 3.1)` within its inflation radius. 
   - A **Warning** is logged, and the controller attempts to replan a local path to avoid the obstacle.

4. **Collision Detection**:
   - Despite replanning, the robot fails to avoid the obstacle and collides at `(1.9, 3.6)`.
   - An **Error** log indicates the collision, followed by a **Fatal** log from `bt_navigator` noting that the path execution failed.

---

### Tips for Log Analysis in Collision Scenarios

- **Warnings for Early Detection**: Look for warning messages about obstacle detection within the inflation radius. This can signal an impending risk of collision.
- **Replanning Events**: Check if the controller server attempts to replan; repeated replanning may indicate insufficient inflation radius or overly aggressive path-following behavior.
- **Error and Fatal Logs**: Error and fatal logs provide critical information about points of failure, helping to pinpoint collision coordinates and potential areas for parameter tuning.

---

This example log can be used to better understand and troubleshoot path-following and collision handling in Nav2.