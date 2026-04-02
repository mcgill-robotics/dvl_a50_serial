# DVL A50 Serial ROS 2 Driver

A modern, high-performance ROS 2 Lifecycle Node driver for the Water Linked DVL-A50 over serial. This module reads directly from the RS232/USB serial port rather than an Ethernet socket, eliminating network overhead and enabling tightly synchronized timestamps (`this->now()`) to prevent TF extrapolation failures typical of raw TCP/IP DVL configurations.

## Setup & Udev Rules
To allow ROS to access the serial port (e.g. `/dev/ttyUSB0`) without requiring `sudo` privileges, you must add your user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
```
*Note: A system reboot or re-login is required for the group change to take effect.*

## Launching
The driver uses the standard ROS 2 component layout and lifecycle protocols. Launching will automatically trigger the node to `configure` and then `activate`.

```bash
source install/setup.bash
ros2 launch dvl_a50_serial dvl_a50_serial.launch.py
```

## Configuration Parameters
Parameters can be configured in `/config/dvl_a50_serial.yaml` or over the `ros2 param` interface. Setting runtime parameters like `speed_of_sound` will automatically sync with the DVL directly.

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `port` | string | `/dev/ttyUSB0` | The device file for the serial port. |
| `baud_rate` | int | `115200` | Baud rate for the serial interface. |
| `frame` | string | `dvl_a50_link` | The TF2 frame ID affixed to the DVL headers. |
| `rate` | double | `30.0` | Publishing frequency (Hz) for decoupled ROS 2 reports. |
| `enable_on_activate`| bool | `false` | Whether to automatically enable acoustic pings on lifecycle activation. |
| `speed_of_sound` | int | `1500` | Speed of sound in m/s used by the sensor. |
| `led_enabled` | bool | `true` | Toggles the internal DVL illumination LED. |
| `mounting_rotation_offset` | int | `0` | Hardware mounting offset in degrees. |
| `range_mode` | string | `auto` | Operating mode (`auto`, `=3`, `1<=3`, etc). |
| `timeout_configure_ms` | int | `3000` | Serial command wait limit for configure frames. |
| `timeout_calibrate_gyro_ms` | int | `15000` | Gyro calibration can take up to 15 seconds. |

*(Also seamlessly supports standard ping timeouts `timeout_reset_dead_reckoning_ms`, `timeout_trigger_ping_ms`, `timeout_set_protocol_ms`)*

## Topics

### Publishers
- `dvl/velocity` (`marine_acoustic_msgs/msg/Dvl`) - Core parsed velocities mapping standard acoustic speeds, covariance arrays, active transducer metrics, and altitude boundaries. Quality of Service is set to `SensorDataQoS` to prevent latency blocking.
- `dvl/dead_reckoning` (`geometry_msgs/msg/PoseWithCovarianceStamped`) - Positional drift mapped natively across DVL frames.
- `dvl/odometry` (`nav_msgs/msg/Odometry`) - Unified Odometry representing twist velocities combined seamlessly when dead reckoning tracks merge. 

## Services
The driver maps state operations down safely to the DVL backend using short timeouts to avoid stalling other executor nodes:
- `enable` (`std_srvs/srv/Trigger`) -> Turns precisely on acoustic transmission.
- `disable` (`std_srvs/srv/Trigger`) -> Secures acoustic noise output mode off.
- `calibrate_gyro` (`std_srvs/srv/Trigger`) -> Queues a fresh hardware gyro calibration. Takes up to 15 seconds.
- `reset_dead_reckoning` (`std_srvs/srv/Trigger`) -> zeroes DVL dead reckoning internal maps.
- `trigger_ping` (`std_srvs/srv/Trigger`) -> Mutes persistent ping mode and issues one standalone transducer pulse.

## Usage Examples

Here are some standard ROS 2 CLI commands you can run in your terminal to easily interact with the module on the AUV:

### Listening to Topics
```bash
# Listen to the raw velocity reports at full frequency
ros2 topic echo /dvl/velocity

# Monitor the integrated odometry containing twist & pose data
ros2 topic echo /dvl/odometry

# Check the DVL publication rate for performance tracking
ros2 topic hz /dvl/velocity
```

### Calling Services
```bash
# Enable acoustics (turns on the DVL pings)
ros2 service call /enable std_srvs/srv/Trigger

# Disable acoustics (securely mutes the DVL pings)
ros2 service call /disable std_srvs/srv/Trigger

# Trigger the hardware gyro calibration routine (takes up to ~15 seconds)
ros2 service call /calibrate_gyro std_srvs/srv/Trigger

# Reset the internal dead reckoning positional tracks back to (0, 0, 0)
ros2 service call /reset_dead_reckoning std_srvs/srv/Trigger

# Fire a single ping manually (requires acoustics to be disabled first)
ros2 service call /trigger_ping std_srvs/srv/Trigger
```

### Changing Parameters Dynamically
Because of the `param_sub_` implementation, you can safely hot-swap core limits at runtime without restarting the lifecycle node!

```bash
# Change the speed of sound safely at runtime
ros2 param set /dvl_a50_serial speed_of_sound 1520

# Disable the internal LED
ros2 param set /dvl_a50_serial led_enabled false

# Switch range mode tracking to exclusively track 7.7m to 36m (mode 3)
ros2 param set /dvl_a50_serial range_mode "=3"

# Switch range mode tracking to search dynamically between 0.3m (1) and 36m (3)
ros2 param set /dvl_a50_serial range_mode "1<=3"
```

### Manual Lifecycle Management
If you disable the auto-startup parameter in the launch file (`autostart:=false`), you must manually step the driver through its lifecycle states:

```bash
# Check the current lifecycle state of the node (e.g. 'unconfigured', 'inactive', 'active')
ros2 lifecycle get /dvl_a50_serial

# Configure the node (Attempts to connect to the serial port and configure the DVL)
ros2 lifecycle set /dvl_a50_serial configure

# Activate the node (Starts publishing ROS messages and enables acoustics if enable_on_activate is true)
ros2 lifecycle set /dvl_a50_serial activate

# Deactivate the node (Stops publishing and disables acoustics)
ros2 lifecycle set /dvl_a50_serial deactivate

# Clean up (Closes the serial port and unloads memory boundaries)
ros2 lifecycle set /dvl_a50_serial cleanup
```

