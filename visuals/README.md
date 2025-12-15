# Point Cloud Visualizer

A real-time visualizer for displaying point cloud data from your MSP432 robot navigation system via serial/Bluetooth.

## Features

- **Ever-expanding canvas** - Automatically scales and pans to show all data
- **Multiple data formats supported** - Flexible parsing for different output formats
- **Real-time visualization** - Displays points as they arrive
- **Interactive controls** - Zoom, pan, clear, and save
- **Trajectory tracking** - Shows robot path and pose history
- **Color-coded scans** - Different colors for different point clouds

## Requirements

- Processing 3.x or 4.x (download from https://processing.org)
- Serial connection to MSP432 (USB or Bluetooth)

## Setup

1. **Install Processing** if you haven't already
2. **Open the sketch**: `pointcloud_visualizer.pde` in Processing
3. **Configure your serial port** (line ~146):
   ```processing
   int portIndex = 0;  // Change this to match your COM port
   ```
   
   Run the sketch once to see available ports in the console, then update the index.

4. **Match baud rate** (line ~151):
   ```processing
   int baudRate = 115200;  // Must match your MSP432 baud rate
   ```

5. **Run the sketch** - Click the play button in Processing

## Supported Data Formats

The visualizer automatically detects and handles three formats:

### Format 1: Structured (Recommended)
```
POSE,x,y,theta
SCAN_START
PT,x,y
PT,x,y
...
SCAN_END
```

**Example:**
```
POSE,0.5,0.2,1.57
SCAN_START
POINT,10.5,5.2
POINT,10.3,5.8
POINT,11.2,6.1
SCAN_END
```

### Format 2: Simple Points
```
P,x,y
```

**Example:**
```
P,10.5,5.2
P,10.3,5.8
P,11.2,6.1
```

### Format 3: Raw Coordinates
```
x,y
```

**Example:**
```
10.5,5.2
10.3,5.8
11.2,6.1
```

## Controls

| Key | Action |
|-----|--------|
| `A` | Toggle auto-scale (automatically fit all data) |
| `C` | Clear all data |
| `+`/`-` | Zoom in/out |
| Arrow Keys | Pan the view |
| `R` | Reset view to auto-scale mode |
| `S` | Save screenshot (saves to sketch folder) |
| `G` | Toggle grid display |
| `T` | Toggle trajectory display |
| `I` | Toggle info panel |
| `1`/`2`/`3` | Change point size (1, 2, or 4 pixels) |

## MSP432 Integration

To send data from your MSP432, use `printf()` with the structured format:

```c
// Send transformed point cloud
printf("POSE,%.3f,%.3f,%.3f\n", 
       current_pose.x, current_pose.y, current_pose.theta);

printf("SCAN_START\n");

for (k = 0; k < transformed_cloud.num_points; k++) {
    printf("POINT,%.3f,%.3f\n",
           transformed_cloud.points[k].x,
           transformed_cloud.points[k].y);
}

printf("SCAN_END\n");
```

Or simpler (just points):
```c
for (k = 0; k < transformed_cloud.num_points; k++) {
    printf("P,%.3f,%.3f\n",
           transformed_cloud.points[k].x,
           transformed_cloud.points[k].y);
}
```

## Color Scheme

- **Background**: Dark gray/black
- **Grid**: Light gray
- **Axes**: Brighter gray (X and Y at origin)
- **Points**: Cyan blue (individual points)
- **Point Clouds**: Rainbow gradient based on timestamp
- **Trajectory**: Yellow line
- **Current Pose**: Red circle with orientation arrow

## Troubleshooting

### "Could not connect to serial port"

1. Check that no other program is using the port (Serial Monitor, etc.)
2. Verify the port name/index in the code
3. Make sure your Bluetooth module is paired (if using Bluetooth)
4. Try unplugging and replugging the device

### "No data received"

1. Verify baud rate matches (115200)
2. Check that your MSP432 is actually sending data
3. Try other serial monitoring software first to confirm data flow
4. Check for proper line endings (`\n`)

### Points not showing up

1. Check that coordinates are in reasonable range
2. Try clearing data (`C` key) and resetting view (`R` key)
3. Toggle auto-scale (`A` key)
4. Manually zoom out (`-` key multiple times)

### Performance issues

1. Reduce point size (press `1`)
2. Clear old data periodically (`C` key)
3. Send fewer points per scan
4. Increase the transmission interval

## Screenshots

Press `S` to save a screenshot. Files are saved as:
```
pointcloud_YYYYMMDD_HHMMSS.png
```

## Tips

- **Start with auto-scale ON** to see your first data
- **Use structured format** (POSE + SCAN) for best visualization with trajectory
- **Keep coordinates in meters** for proper scaling
- **Clear data** before starting a new test run
- **Save screenshots** of interesting maps before clearing

## Future Enhancements

- [ ] Export point cloud to CSV
- [ ] Replay saved data
- [ ] Multiple visualization modes
- [ ] Distance measurement tool
- [ ] Frame rate display
- [ ] Data statistics panel
