/**
 * Real-Time Point Cloud Visualizer
 * 
 * Receives point cloud data from serial port and displays an ever-expanding map
 * 
 * Serial Data Formats Supported:
 * 
 * Format 1 (Structured):
 *   POSE,x,y,theta
 *   SCAN_START
 *   P,x,y
 *   P,x,y
 *   ...
 *   SCAN_END
 * 
 * Format 2 (Simple):
 *   PT,x,y        - Just points (auto-accumulates)
 * 
 * Format 3 (Raw):
 *   x,y           - Raw coordinate pairs
 * 
 * Controls:
 *   A - Toggle auto-scale
 *   C - Clear all data
 *   +/- - Zoom in/out
 *   Arrow keys - Pan view
 *   S - Save screenshot
 *   R - Reset view
 *   1/2/3 - Change point size
 */

import processing.serial.*;

Serial serial_port;

// Data structures
class Point2D {
    float x,
          y;
    long  timestamp;
    
    Point2D(float x, float y) {
        this.x          = x;
        this.y          = y;
        this.timestamp  = millis();
    }
}

class Pose {
    float x,
          y,
          theta;
    long timestamp;
    
    Pose(float x, float y, float theta) {
        this.x         = x;
        this.y         = y;
        this.theta     = theta;
        this.timestamp = millis();
    }
}

class PointCloud {
    ArrayList<Point2D> points;
    Pose pose;
    color cloud_color;
    long timestamp;
    
    PointCloud(Pose pose) {
        float hue;
        
        this.points = new ArrayList<Point2D>();
        this.pose = pose;
        this.timestamp = millis();
        
        // Generate color based on timestamp for gradient effect
        hue   = (this.timestamp % 10000) / 10000.0 * 255;
        colorMode(HSB);
        this.cloud_color = color(hue, 200, 255, 150);
        colorMode(RGB);
    }
    
    void addPoint(float x, float y) {
        points.add(new Point2D(x, y));
    }
}

// Global state
ArrayList<Point2D>     all_points = new ArrayList<Point2D>();
ArrayList<PointCloud>  all_clouds = new ArrayList<PointCloud>();
ArrayList<Pose>        trajectory = new ArrayList<Pose>();
PointCloud          current_cloud = null;

// Synchronization lock
Object data_lock = new Object();

// View boundaries (world coordinates)
float min_x = 0,  max_x = 100;
float min_y = 0,  max_y = 100;

// Display settings
float   scale       = 10.0;  // pixels per unit
float   offset_x    = 0;
float   offset_y    = 0;
boolean autoscale   = true;
int     point_size  = 2;

// Pan offset (for manual panning)
float pan_x = 0;
float pan_y = 0;

// UI state
String  status_message    = "Waiting for data...";
int     total_point_count = 0;
int     pose_count        = 0;
int     scan_count        = 0;
long    last_data_time    = 0;
boolean show_grid         = true;
boolean show_trajectory   = true;
boolean show_info         = true;

// Color scheme
color background_color    = color(15, 15, 20);
color grid_color          = color(40, 40, 50);
color axis_color          = color(80, 80, 100);
color trajectory_color    = color(255, 200, 0, 150);
color current_pose_color  = color(255, 50, 50);
color point_color         = color(100, 200, 255, 200);


void setup() {
    size(1600, 900);
    smooth();
    
    // List available serial ports
    println("=== Available Serial Ports ===");
    String[] ports = Serial.list();
    for (int i = 0; i < ports.length; i++) {
        println("[" + i + "] " + ports[i]);
    }
    println("==============================");
    
    // Connect to serial port
    try {
        String port_name = "COM5";
        
        // String port_name = Serial.list()[portIndex];
        int baud_rate = 115200;  // Match your MSP432 baud rate
        
        serial_port = new Serial(this, port_name, baud_rate);
        serial_port.bufferUntil('\n');
        
        status_message = "Connected to " + port_name + " @ " + baud_rate;
        println(status_message);
        
    } catch (Exception e) {
        status_message = "ERROR: Could not connect to serial port";
        println(status_message);
        println(e.getMessage());
    }
    
    background(background_color);
}

void draw() {
    background(background_color);
    
    // Update view if auto-scaling
    if (    autoscale
         && (all_points.size() > 0 || all_clouds.size() > 0))
    {
        updateBoundaries();
        calculateScaleAndOffset();
    }
    
    // Apply transformations
    pushMatrix();
    translate(pan_x, pan_y);
    
    // Draw grid
    if (show_grid) {
        drawGrid();
    }
    
    // Draw trajectory
    synchronized(data_lock) {
        if (show_trajectory && trajectory.size() > 0) {
            drawTrajectory();
        }
        
        // Draw all individual points
        drawAllPoints();
        
        // Draw all point clouds
        for (PointCloud cloud : all_clouds) {
            drawPointCloud(cloud);
        }
        
        // Draw current pose
        if (trajectory.size() > 0) {
            drawPose(trajectory.get(trajectory.size() - 1),
                     current_pose_color);
        }
    }
    
    popMatrix();
    
    // Draw UI (not affected by pan)
    if (show_info) {
        drawUI();
    }
    
    // Check for timeout
    if (millis() - last_data_time > 5000 && total_point_count > 0) {
        status_message = "No data received (idle)";
    }
}

void serialEvent(Serial port) {
    String data = port.readStringUntil('\n');
    if (data == null) return;
    
    data = trim(data);
    if (data.length() == 0) return;
    
    
    synchronized(data_lock) {
        last_data_time = millis();
        
        String[] parts = split(data, ',');
        if (parts.length == 0) return;
    
        // Parse different formats
        
        // Format 1: Structured (POSE, SCAN_START, P, SCAN_END)
        if (parts[0].equals("POSE") && parts.length >= 4) {
            handlePose(float(parts[1]), float(parts[2]), float(parts[3]));
            
        } else if (parts[0].equals("SCAN_START")) {
            handleScanStart();
            
        } else if (parts[0].equals("P") && parts.length >= 3) {
            handlePoint(float(parts[1]), float(parts[2]));
            
        } else if (parts[0].equals("SCAN_END")) {
            handleScanEnd();
            
        } else if (parts[0].equals("SAVE")) {
            saveScreenshot();
            
        } else if (parts[0].equals("CLEAR")) {
            clearData();
            
            
        // Format 2: Simple point format (PT,x,y)
        } else if (parts[0].equals("PT") && parts.length >= 3) {
            
            float x, y;
            
            x = float(parts[1]);
            y = float(parts[2]);
            all_points.add(new Point2D(x, y));
            total_point_count++;
            status_message = "Point received: (" + nf(x, 0, 2) + ", " + nf(y, 0, 2) + ")";
            
            
        // Format 3: Raw coordinate pairs (x,y)
        } else if (parts.length == 2) {
            try {
                float x, y;
                
                x = float(parts[0]);
                y = float(parts[1]);
                all_points.add(new Point2D(x, y));
                total_point_count++;
                status_message = "Raw point: (" + nf(x, 0, 2) + ", " + nf(y, 0, 2) + ")";
            } catch (Exception e) {
                // Not a valid coordinate pair, ignore
            }
        }
    }  // End synchronized block
}

void handlePose(float x, float y, float theta) {
    Pose new_pose = new Pose(x, y, theta);
    trajectory.add(new_pose);
    pose_count++;
    status_message  = "Pose " + pose_count + ": (" + nf(x, 0, 2) + ", " + nf(y, 0, 2) + ", " + nf(theta, 0, 3) + ")";
}

void handleScanStart() {
  
    if (trajectory.size() > 0) {
        current_cloud   = new PointCloud(trajectory.get(trajectory.size() - 1));
        status_message  = "Recording scan " + (scan_count + 1) + "...";

    // No pose, create default
    } else {
        current_cloud   = new PointCloud(new Pose(0, 0, 0));
        status_message  = "Recording scan (no pose)...";
    }
    
}

void handlePoint(float x, float y) {
  
    if (current_cloud != null) {
        current_cloud.addPoint(x, y);
        total_point_count++;
        
        
    } else {
        // No active cloud, add to general points
        all_points.add(new Point2D(x, y));
        total_point_count++;
    }
    
}

void handleScanEnd() {
  
    if (current_cloud != null) {
        all_clouds.add(current_cloud);
        scan_count++;
        status_message = "Scan " + scan_count + " complete: " + current_cloud.points.size() + " points";
        current_cloud = null;
    }
    
}

void updateBoundaries() {
  
    synchronized(data_lock) {
        
        boolean has_data = false;
        
        min_x =  Float.MAX_VALUE;
        max_x = -Float.MAX_VALUE;
        min_y =  Float.MAX_VALUE;
        max_y = -Float.MAX_VALUE;
        
        // Check individual points
        for (Point2D p : all_points) {
            min_x     = min(min_x, p.x);
            max_x     = max(max_x, p.x);
            min_y     = min(min_y, p.y);
            max_y     = max(max_y, p.y);
            has_data  = true;
                      
        }
        
        // Check point clouds
        for (PointCloud cloud : all_clouds) {
            for (Point2D p : cloud.points) {
                min_x     = min(min_x, p.x);
                max_x     = max(max_x, p.x);
                min_y     = min(min_y, p.y);
                max_y     = max(max_y, p.y);
                has_data  = true;
            }
        }
        
        // Check trajectory
        for (Pose pose : trajectory) {
            min_x     = min(min_x, pose.x);
            max_x     = max(max_x, pose.x);
            min_y     = min(min_y, pose.y);
            max_y     = max(max_y, pose.y);
            has_data  = true;
        }
        
        if (has_data) {
            
            float range_x, range_y;
            float margin_x, margin_y;
            // Add 10% margin
            range_x   = max_x - min_x;
            range_y   = max_y - min_y;
            
            if (range_x < 0.1) range_x = 1.0;
            if (range_y < 0.1) range_y = 1.0;
            
            margin_x  = range_x * 0.1;
            margin_y  = range_y * 0.1;
            
            min_x  -= margin_x;
            max_x  += margin_x;
            min_y  -= margin_y;
            max_y  += margin_y;
            
        } else {
            // Default bounds
            min_x = -10;
            max_x =  10;
            min_y = -10;
            max_y =  10;
        }
    }  // End synchronized block
}

void calculateScaleAndOffset() {
  
    float range_x, range_y;
    float scale_x, scale_y;
  
    range_x = max_x - min_x;
    range_y = max_y - min_y;
    
    if (range_x < 0.1) range_x = 1.0;
    if (range_y < 0.1) range_y = 1.0;
    
    // Calculate scale to fit window (leave room for UI)
    scale_x = (width - 250) / range_x;
    scale_y = (height - 150) / range_y;
    scale = min(scale_x, scale_y);
    
    // Center the map
    offset_x = width  / 2 - (min_x + max_x) / 2 * scale;
    offset_y = height / 2 + (min_y + max_y) / 2 * scale;  // Flip Y
}

void drawGrid() {
    stroke(grid_color);
    strokeWeight(1);
    
    // Determine grid spacing based on scale
    float grid_size = 1.0;  // Default 1 unit
    if (scale < 5)
        grid_size = 10.0;
    else if (scale < 20)
        grid_size = 5.0;
    else if (scale > 100)
        grid_size = 0.5;
    
    // Draw vertical lines
    for (float wx = floor(min_x / grid_size) * grid_size; wx <= max_x; wx += grid_size) {
        float sx = worldToScreenX(wx);
        line(sx, 0, sx, height);
    }
    
    // Draw horizontal lines
    for (float wy = floor(min_y / grid_size) * grid_size; wy <= max_y; wy += grid_size) {
        float sy = worldToScreenY(wy);
        line(0, sy, width, sy);
    }
    
    // Draw axes (thicker)
    stroke(axis_color);
    strokeWeight(2);
    float origin_x = worldToScreenX(0);
    float originY = worldToScreenY(0);
    
    if (origin_x >= 0 && origin_x <= width) {
        line(origin_x, 0, origin_x, height);
    }
    if (originY >= 0 && originY <= height) {
        line(0, originY, width, originY);
    }
}

void drawTrajectory() {
    if (trajectory.size() < 2) return;
    
    stroke(trajectory_color);
    strokeWeight(3);
    noFill();
    
    beginShape();
    for (Pose pose : trajectory) {
        float sx = worldToScreenX(pose.x);
        float sy = worldToScreenY(pose.y);
        vertex(sx, sy);
    }
    endShape();
    
    // Draw pose markers
    for (Pose pose : trajectory) {
        drawPose(pose, trajectory_color);
    }
}

void drawAllPoints() {
    stroke(point_color);
    strokeWeight(point_size);
    
    for (Point2D p : all_points) {
        float sx = worldToScreenX(p.x);
        float sy = worldToScreenY(p.y);
        
        // Only draw if on screen
        if (sx >= -10 && sx <= width + 10 && sy >= -10 && sy <= height + 10) {
            point(sx, sy);
        }
    }
}

void drawPointCloud(PointCloud cloud) {
    stroke(cloud.cloud_color);
    strokeWeight(point_size);
    
    for (Point2D p : cloud.points) {
        float sx = worldToScreenX(p.x);
        float sy = worldToScreenY(p.y);
        
        // Only draw if on screen
        if (sx >= -10 && sx <= width + 10 && sy >= -10 && sy <= height + 10) {
            point(sx, sy);
        }
    }
}

void drawPose(Pose pose, color c) {
    float sx = worldToScreenX(pose.x);
    float sy = worldToScreenY(pose.y);
    
    // Draw pose as filled circle
    fill(c, 200);
    noStroke();
    ellipse(sx, sy, 12, 12);
    
    // Draw orientation arrow
    stroke(c);
    strokeWeight(3);
    float arrowLen = 0.5 * scale;
    float end_x = sx + arrowLen * cos(pose.theta);
    float end_y = sy - arrowLen * sin(pose.theta);  // Flip Y
    line(sx, sy, end_x, end_y);
    
    // Draw arrowhead
    float angle = pose.theta;
    float arrow_size = 0.15 * scale;
    float x1 = end_x - arrow_size * cos(angle - PI/6);
    float y1 = end_y + arrow_size * sin(angle - PI/6);
    float x2 = end_x - arrow_size * cos(angle + PI/6);
    float y2 = end_y + arrow_size * sin(angle + PI/6);
    line(end_x, end_y, x1, y1);
    line(end_x, end_y, x2, y2);
}

void drawUI() {
    // Main info panel (top left)
    fill(0, 0, 0, 200);
    noStroke();
    rect(10, 10, 320, 180);
    
    fill(255);
    textAlign(LEFT);
    textSize(16);
    text("Point Cloud Visualizer", 20, 30);
    
    textSize(12);
    text("Status: " + status_message, 20, 55);
    text("Total Points: " + total_point_count, 20, 75);
    text("Scans: " + scan_count, 20, 95);
    text("Poses: " + pose_count, 20, 115);
    text("Scale: " + nf(scale, 0, 2) + " px/unit", 20, 135);
    text("Bounds: [" + nf(min_x, 0, 1) + ", " + nf(max_x, 0, 1) + "] x [" + 
         nf(min_y, 0, 1) + ", " + nf(max_y, 0, 1) + "]", 20, 155);
    text("Point Size: " + point_size, 20, 175);
    
    // Controls panel (top right)
    fill(0, 0, 0, 200);
    rect(width - 230, 10, 220, 240);
    
    fill(255, 255, 100);
    textSize(14);
    text("Controls", width - 220, 30);
    
    fill(255);
    textSize(11);
    int y = 50;
    int dy = 18;
    text("A - Toggle Auto-scale", width - 220, y); y += dy;
    text("C - Clear All Data", width - 220, y); y += dy;
    text("+/- - Zoom In/Out", width - 220, y); y += dy;
    text("Arrow Keys - Pan View", width - 220, y); y += dy;
    text("R - Reset View", width - 220, y); y += dy;
    text("S - Save Screenshot", width - 220, y); y += dy;
    text("G - Toggle Grid", width - 220, y); y += dy;
    text("T - Toggle Trajectory", width - 220, y); y += dy;
    text("I - Toggle Info", width - 220, y); y += dy;
    text("1/2/3 - Point Size", width - 220, y); y += dy;
    
    // Auto-scale indicator
    fill(autoscale ? color(0, 255, 0) : color(255, 0, 0));
    text("Auto-scale: " + (autoscale ? "ON" : "OFF"), width - 220, y);
}

void keyPressed() {
    float panSpeed = 20;
    
    if (key == 'a' || key == 'A') {
        autoscale = !autoscale;
        status_message = "Auto-scale: " + (autoscale ? "ON" : "OFF");
        
    } else if (key == 'c' || key == 'C') {
        clearData();
        
    } else if (key == 'r' || key == 'R') {
        resetView();
        
    } else if (key == 's' || key == 'S') {
        saveScreenshot();
        
    } else if (key == 'g' || key == 'G') {
        show_grid = !show_grid;
        status_message = "Grid: " + (show_grid ? "ON" : "OFF");
        
    } else if (key == 't' || key == 'T') {
        show_trajectory = !show_trajectory;
        status_message = "Trajectory: " + (show_trajectory ? "ON" : "OFF");
        
    } else if (key == 'i' || key == 'I') {
        show_info = !show_info;
        
    } else if (key == '1') {
        point_size = 1;
        status_message = "Point size: 1";
        
    } else if (key == '2') {
        point_size = 2;
        status_message = "Point size: 2";
        
    } else if (key == '3') {
        point_size = 4;
        status_message = "Point size: 4";
        
    } else if (key == '+' || key == '=') {
        scale *= 1.2;
        autoscale = false;
        status_message = "Zoomed in";
        
    } else if (key == '-' || key == '_') {
        scale *= 0.8;
        autoscale = false;
        status_message = "Zoomed out";
    }
    
    // Arrow key panning
    if (keyCode == UP) {
        pan_y += panSpeed;
        autoscale = false;
    } else if (keyCode == DOWN) {
        pan_y -= panSpeed;
        autoscale = false;
    } else if (keyCode == LEFT) {
        pan_x += panSpeed;
        autoscale = false;
    } else if (keyCode == RIGHT) {
        pan_x -= panSpeed;
        autoscale = false;
    }
}

void clearData() {
    synchronized(data_lock) {
        all_points.clear();
        all_clouds.clear();
        trajectory.clear();
        current_cloud = null;
        total_point_count = 0;
        pose_count = 0;
        scan_count = 0;
        status_message = "Data cleared";
        println("Data cleared at " + millis());
    }
}

void resetView() {
    autoscale = true;
    pan_x = 0;
    pan_y = 0;
    status_message = "View reset";
}

void saveScreenshot() {
    String filename =   "pointcloud_" + year() + "_" + nf(month(), 2) + "_" + nf(day(), 2)
                      + "_" + "-" + "_"
                      + nf(hour(), 2) + "-" + nf(minute(), 2) + "-" + nf(second(), 2) + ".png";
    save(filename);
    status_message = "Screenshot saved: " + filename;
    println("Screenshot saved: " + filename);
}

// Coordinate conversion helpers
float worldToScreenX(float worldX) {
    return worldX * scale + offset_x;
}

float worldToScreenY(float worldY) {
    return -worldY * scale + offset_y;  // Flip Y axis
}

float screenToWorldX(float screenX) {
    return (screenX - offset_x) / scale;
}

float screenToWorldY(float screenY) {
    return -(screenY - offset_y) / scale;  // Flip Y axis
}
