/**
 * Real-Time Quaternion Orientation Visualizer
 * 
 * Receives orientation and vector data from serial port and displays:
 * - 3D airplane model showing sensor orientation
 * - Gravity vector (world and sensor frame)
 * - Magnetic field vector (world and sensor frame)
 * - Acceleration vector (world and sensor frame)
 * 
 * Serial Data Format:
 *   Q,w,x,y,z      - Quaternion (w is scalar, x,y,z are vector components) [PRIMARY]
 *   E,psi,theta,phi - Euler angles in degrees (yaw,pitch,roll) [OPTIONAL - for display only]
 *   G,x,y,z        - Gravity vector
 *   M,x,y,z        - Magnetometer vector (µT)
 *   A,x,y,z        - World-frame acceleration (m/s²)
 * 
 * Why Quaternions?
 *   Quaternions provide smooth, continuous rotation without gimbal lock or discontinuities.
 *   Euler angles can cause sudden jumps/spins when converted from quaternions.
 * 
 * Controls:
 *   G - Toggle gravity vector display
 *   M - Toggle magnetometer vector display
 *   A - Toggle acceleration vector display
 *   W - Toggle world-frame vectors
 *   S - Toggle sensor-frame vectors
 *   R - Reset camera view
 *   C - Clear all data
 *   +/- - Zoom in/out
 *   Arrow keys - Rotate camera view
 *   Mouse drag - Rotate view
 */

import processing.serial.*;

Serial serial_port;

// Quaternion data
float quat_w = 1.0;
float quat_x = 0.0;
float quat_y = 0.0;
float quat_z = 0.0;

// Euler angles (for Processing's rotateX/Y/Z functions)
float euler_psi = 0.0;    // yaw (Z-axis rotation)
float euler_theta = 0.0;  // pitch (Y-axis rotation) 
float euler_phi = 0.0;    // roll (X-axis rotation)

// Vector data
PVector gravity_vec = new PVector(0, 0, 1);      // Gravity vector
PVector mag_vec = new PVector(1, 0, 0);          // Magnetometer vector
PVector accel_vec = new PVector(0, 0, 0);        // Acceleration vector

// Display toggles
boolean show_gravity = true;
boolean show_magnetometer = true;
boolean show_acceleration = true;
boolean show_world_frame = true;
boolean show_sensor_frame = true;
boolean show_axes = true;
boolean show_grid = true;

// Camera control
float camera_rot_x = -30;
float camera_rot_y = 0;
float camera_rot_z = 0;
float camera_zoom = 200;
float mouse_sensitivity = 0.5;
int prev_mouse_x = 0;
int prev_mouse_y = 0;

// Statistics
int frame_count = 0;
int last_fps_time = 0;
float current_fps = 0;
long packets_received = 0;
long last_packet_time = 0;

// Synchronization lock
Object data_lock = new Object();

// Serial settings
String serial_port_name = "COM8";  // Change this to match your port
int baud_rate = 115200;

// Scale factors for vectors (for visualization)
float gravity_scale = 100;    // Unit vector ~1.0
float mag_scale = 2;          // µT (~50)
float accel_scale = 10;       // m/s²

// Colors
color color_gravity = color(0, 200, 0);      // Green
color color_mag = color(200, 0, 200);        // Magenta
color color_accel = color(0, 150, 255);      // Cyan
color color_airplane = color(200, 200, 200); // Gray
color color_x_axis = color(255, 0, 0);       // Red
color color_y_axis = color(0, 255, 0);       // Green
color color_z_axis = color(0, 0, 255);       // Blue

void setup() {
    size(1200, 800, P3D);
    
    // Initialize serial connection
    println("Available serial ports:");
    printArray(Serial.list());
    
    try {
        serial_port = new Serial(this, serial_port_name, baud_rate);
        serial_port.bufferUntil('\n');
        println("Connected to: " + serial_port_name);
    } catch (Exception e) {
        println("Error opening serial port: " + e.getMessage());
        println("Visualization will run without serial data.");
    }
    
    frameRate(60);
    last_fps_time = millis();
    
    println("\n=== Quaternion Orientation Visualizer Started ===");
    println("Using quaternions for smooth, gimbal-lock-free rotation");
    println("Controls:");
    println("  G - Toggle gravity vector");
    println("  M - Toggle magnetometer vector");
    println("  A - Toggle acceleration vector");
    println("  W - Toggle world-frame vectors");
    println("  S - Toggle sensor-frame vectors");
    println("  R - Reset camera view");
    println("  +/- - Zoom in/out");
    println("  Arrow keys - Rotate camera");
    println("  Mouse drag - Rotate view");
}

void draw() {
    background(20);
    
    // Update FPS counter
    frame_count++;
    if (millis() - last_fps_time > 1000) {
        current_fps = frame_count * 1000.0 / (millis() - last_fps_time);
        frame_count = 0;
        last_fps_time = millis();
    }
    
    // Set up 3D camera
    pushMatrix();
    translate(width/2, height/2, 0);
    
    // Apply camera rotation
    rotateX(radians(camera_rot_x));
    rotateY(radians(camera_rot_y));
    rotateZ(radians(camera_rot_z));
    
    // Draw grid and world axes
    if (show_grid) {
        drawGrid();
    }
    if (show_axes) {
        drawWorldAxes();
    }
    
    // Draw world-frame vectors (fixed in world space)
    if (show_world_frame) {
        pushMatrix();
        if (show_gravity) {
            drawVector(gravity_vec, gravity_scale, color_gravity, "G_world");
        }
        if (show_magnetometer) {
            drawVector(mag_vec, mag_scale, color_mag, "M_world");
        }
        if (show_acceleration) {
            drawVector(accel_vec, accel_scale, color_accel, "A_world");
        }
        popMatrix();
    }
    
    // Draw sensor and sensor-frame vectors (rotated with airplane)
    pushMatrix();
    
    // Apply quaternion rotation using rotation matrix
    synchronized(data_lock) {
        applyQuaternionRotation(quat_w, quat_x, quat_y, quat_z);
    }
    
    // Draw airplane
    drawAirplane();
    
    // Draw sensor-frame axes
    if (show_axes) {
        drawSensorAxes();
    }
    
    // Draw sensor-frame vectors (in sensor's local frame)
    if (show_sensor_frame) {
        if (show_gravity) {
            // Transform gravity to sensor frame (inverse rotation)
            PVector g_sensor = worldToSensor(gravity_vec);
            drawVector(g_sensor, gravity_scale, color(0, 255, 0, 150), "G_sensor");
        }
        if (show_magnetometer) {
            PVector m_sensor = worldToSensor(mag_vec);
            drawVector(m_sensor, mag_scale, color(255, 0, 255, 150), "M_sensor");
        }
        if (show_acceleration) {
            PVector a_sensor = worldToSensor(accel_vec);
            drawVector(a_sensor, accel_scale, color(0, 200, 255, 150), "A_sensor");
        }
    }
    
    popMatrix();
    
    popMatrix();
    
    // Draw 2D HUD overlay
    drawHUD();
}

// Convert world-frame vector to sensor-frame using quaternion
PVector worldToSensor(PVector world_vec) {
    // Create quaternion from vector: q_v = [0, x, y, z]
    float[] q_v = {0, world_vec.x, world_vec.y, world_vec.z};
    
    // Get quaternion conjugate: q* = [w, -x, -y, -z]
    float[] q_conj = {quat_w, -quat_x, -quat_y, -quat_z};
    
    // Compute: q* * q_v * q
    float[] temp = quatMultiply(q_conj, q_v);
    float[] q = {quat_w, quat_x, quat_y, quat_z};
    float[] result = quatMultiply(temp, q);
    
    return new PVector(result[1], result[2], result[3]);
}

// Quaternion multiplication: result = q1 * q2
float[] quatMultiply(float[] q1, float[] q2) {
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    return new float[] {
        w1*w2 - x1*x2 - y1*y2 - z1*z2,  // w
        w1*x2 + x1*w2 + y1*z2 - z1*y2,  // x
        w1*y2 - x1*z2 + y1*w2 + z1*x2,  // y
        w1*z2 + x1*y2 - y1*x2 + z1*w2   // z
    };
}

// Apply quaternion rotation using 3x3 rotation matrix
void applyQuaternionRotation(float w, float x, float y, float z) {
    // Normalize quaternion
    float mag = sqrt(w*w + x*x + y*y + z*z);
    if (mag > 0.0001) {
        w /= mag;
        x /= mag;
        y /= mag;
        z /= mag;
    }
    
    // Convert quaternion to 3x3 rotation matrix
    // Reference: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
    float m00 = 1 - 2*(y*y + z*z);
    float m01 = 2*(x*y - w*z);
    float m02 = 2*(x*z + w*y);
    
    float m10 = 2*(x*y + w*z);
    float m11 = 1 - 2*(x*x + z*z);
    float m12 = 2*(y*z - w*x);
    
    float m20 = 2*(x*z - w*y);
    float m21 = 2*(y*z + w*x);
    float m22 = 1 - 2*(x*x + y*y);
    
    // Apply the rotation matrix to Processing's transformation matrix
    applyMatrix(
        m00, m01, m02, 0,
        m10, m11, m12, 0,
        m20, m21, m22, 0,
        0,   0,   0,   1
    );
}

// Update Euler angles from quaternion (for display purposes only)
void updateEulerAngles(float w, float x, float y, float z) {
    // Normalize quaternion
    float mag = sqrt(w*w + x*x + y*y + z*z);
    if (mag > 0.0001) {
        w /= mag;
        x /= mag;
        y /= mag;
        z /= mag;
    }
    
    // Convert quaternion to Euler angles (same as helper_3dmath.h Get_Euler function)
    euler_psi = atan2(2*x*y - 2*w*z, 2*w*w + 2*x*x - 1);      // psi (yaw)
    euler_theta = -asin(2*x*z + 2*w*y);                        // theta (pitch)
    euler_phi = atan2(2*y*z - 2*w*x, 2*w*w + 2*z*z - 1);      // phi (roll)
}

// Draw a 3D airplane model
void drawAirplane() {
    pushMatrix();
    
    // Main fuselage (elongated box along X-axis = forward)
    fill(color_airplane);
    stroke(100);
    strokeWeight(1);
    
    // Fuselage
    pushMatrix();
    translate(30, 0, 0);
    box(80, 10, 10);
    popMatrix();
    
    // Nose cone
    pushMatrix();
    translate(70, 0, 0);
    fill(255, 100, 100);
    sphere(8);
    popMatrix();
    
    // Wings (flat boxes along Y-axis)
    fill(color_airplane);
    pushMatrix();
    translate(10, 0, 0);
    box(15, 100, 3);
    popMatrix();
    
    // Tail (vertical stabilizer along Z-axis)
    pushMatrix();
    translate(-30, 0, 0);
    box(10, 3, 30);
    popMatrix();
    
    // Horizontal stabilizer
    pushMatrix();
    translate(-30, 0, 15);
    box(10, 40, 3);
    popMatrix();
    
    popMatrix();
}

// Draw world coordinate axes (fixed in world frame)
void drawWorldAxes() {
    strokeWeight(3);
    float axis_length = 150;
    
    // X-axis (Red)
    stroke(color_x_axis);
    line(0, 0, 0, axis_length, 0, 0);
    
    // Y-axis (Green)
    stroke(color_y_axis);
    line(0, 0, 0, 0, axis_length, 0);
    
    // Z-axis (Blue)
    stroke(color_z_axis);
    line(0, 0, 0, 0, 0, axis_length);
    
    // Draw axis labels
    fill(color_x_axis);
    pushMatrix();
    translate(axis_length + 10, 0, 0);
    text("X", 0, 0);
    popMatrix();
    
    fill(color_y_axis);
    pushMatrix();
    translate(0, axis_length + 10, 0);
    text("Y", 0, 0);
    popMatrix();
    
    fill(color_z_axis);
    pushMatrix();
    translate(0, 0, axis_length + 10);
    text("Z", 0, 0);
    popMatrix();
}

// Draw sensor coordinate axes (rotated with sensor)
void drawSensorAxes() {
    strokeWeight(2);
    float axis_length = 80;
    
    // X-axis (Red) - Forward
    stroke(color_x_axis, 180);
    line(0, 0, 0, axis_length, 0, 0);
    
    // Y-axis (Green) - Right
    stroke(color_y_axis, 180);
    line(0, 0, 0, 0, axis_length, 0);
    
    // Z-axis (Blue) - Down
    stroke(color_z_axis, 180);
    line(0, 0, 0, 0, 0, axis_length);
}

// Draw a grid on the ground plane
void drawGrid() {
    stroke(50);
    strokeWeight(1);
    int grid_size = 500;
    int grid_spacing = 50;
    
    pushMatrix();
    translate(0, 0, 0);
    
    for (int i = -grid_size; i <= grid_size; i += grid_spacing) {
        line(i, -grid_size, 0, i, grid_size, 0);
        line(-grid_size, i, 0, grid_size, i, 0);
    }
    
    popMatrix();
}

// Draw a vector as an arrow
void drawVector(PVector vec, float scale, color col, String label) {
    PVector v = vec.copy();
    v.mult(scale);
    
    strokeWeight(3);
    stroke(col);
    line(0, 0, 0, v.x, v.y, v.z);
    
    // Draw arrowhead
    pushMatrix();
    translate(v.x, v.y, v.z);
    
    // Point toward origin
    float len = v.mag();
    if (len > 0.001) {
        PVector dir = v.copy().normalize();
        
        // Simple arrowhead using sphere
        fill(col);
        noStroke();
        sphere(4);
    }
    
    popMatrix();
}

// Draw heads-up display
void drawHUD() {
    // Switch to 2D rendering
    hint(DISABLE_DEPTH_TEST);
    camera();
    
    fill(255);
    textAlign(LEFT, TOP);
    textSize(14);
    
    int y_offset = 10;
    int line_height = 18;
    
    // FPS and packet info
    text(String.format("FPS: %.1f", current_fps), 10, y_offset);
    y_offset += line_height;
    text(String.format("Packets: %d", packets_received), 10, y_offset);
    y_offset += line_height;
    
    // Quaternion values
    y_offset += 5;
    text("Quaternion:", 10, y_offset);
    y_offset += line_height;
    synchronized(data_lock) {
        text(String.format("  w: %7.4f  x: %7.4f", quat_w, quat_x), 10, y_offset);
        y_offset += line_height;
        text(String.format("  y: %7.4f  z: %7.4f", quat_y, quat_z), 10, y_offset);
        y_offset += line_height;
    }
    
    // Euler angles
    y_offset += 5;
    text(String.format("Euler (deg) - Yaw: %.1f  Pitch: %.1f  Roll: %.1f", 
         degrees(euler_psi), degrees(euler_theta), degrees(euler_phi)), 10, y_offset);
    y_offset += line_height;
    
    // Vector magnitudes
    y_offset += 5;
    fill(color_gravity);
    text(String.format("Gravity: [%.3f, %.3f, %.3f] mag: %.3f", 
         gravity_vec.x, gravity_vec.y, gravity_vec.z, gravity_vec.mag()), 10, y_offset);
    y_offset += line_height;
    
    fill(color_mag);
    text(String.format("Mag (µT): [%.1f, %.1f, %.1f] mag: %.1f", 
         mag_vec.x, mag_vec.y, mag_vec.z, mag_vec.mag()), 10, y_offset);
    y_offset += line_height;
    
    fill(color_accel);
    text(String.format("Accel (m/s²): [%.3f, %.3f, %.3f] mag: %.3f", 
         accel_vec.x, accel_vec.y, accel_vec.z, accel_vec.mag()), 10, y_offset);
    y_offset += line_height;
    
    // Display toggles
    y_offset += 10;
    fill(255);
    text("Display Toggles:", 10, y_offset);
    y_offset += line_height;
    text("  [G] Gravity: " + (show_gravity ? "ON" : "OFF"), 10, y_offset);
    y_offset += line_height;
    text("  [M] Magnetometer: " + (show_magnetometer ? "ON" : "OFF"), 10, y_offset);
    y_offset += line_height;
    text("  [A] Acceleration: " + (show_acceleration ? "ON" : "OFF"), 10, y_offset);
    y_offset += line_height;
    text("  [W] World Frame: " + (show_world_frame ? "ON" : "OFF"), 10, y_offset);
    y_offset += line_height;
    text("  [S] Sensor Frame: " + (show_sensor_frame ? "ON" : "OFF"), 10, y_offset);
    
    // Controls legend
    y_offset = height - 100;
    fill(200);
    textSize(12);
    text("Controls: G/M/A=Toggle vectors | W/S=Frames | R=Reset | +/-=Zoom | Arrows=Rotate", 10, y_offset);
    
    hint(ENABLE_DEPTH_TEST);
}

// Serial event handler
void serialEvent(Serial port) {
    try {
        String line = port.readStringUntil('\n');
        if (line == null) return;
        
        line = trim(line);
        if (line.length() == 0) return;
        
        // Parse the line
        String[] parts = split(line, ',');
        if (parts.length < 2) return;
        
        String type = parts[0];
        
        synchronized(data_lock) {
            if (type.equals("Q") && parts.length >= 5) {
                // Quaternion: Q,w,x,y,z (primary for rotation)
                quat_w = float(parts[1]);
                quat_x = float(parts[2]);
                quat_y = float(parts[3]);
                quat_z = float(parts[4]);
                // Update Euler angles for display only
                updateEulerAngles(quat_w, quat_x, quat_y, quat_z);
                packets_received++;
                last_packet_time = millis();
                
            } else if (type.equals("E") && parts.length >= 4) {
                // Euler angles in degrees: E,psi,theta,phi (for display/debugging)
                euler_psi = radians(float(parts[1]));
                euler_theta = radians(float(parts[2]));
                euler_phi = radians(float(parts[3]));
                // Optionally convert back to quaternion for rotation
                // (but rotation will be less smooth)
                packets_received++;
                last_packet_time = millis();
                packets_received++;
                last_packet_time = millis();
                
            } else if (type.equals("G") && parts.length >= 4) {
                // Gravity: G,x,y,z
                gravity_vec.set(float(parts[1]), float(parts[2]), float(parts[3]));
                
            } else if (type.equals("M") && parts.length >= 4) {
                // Magnetometer: M,x,y,z
                mag_vec.set(float(parts[1]), float(parts[2]), float(parts[3]));
                
            } else if (type.equals("A") && parts.length >= 4) {
                // Acceleration: A,x,y,z
                accel_vec.set(float(parts[1]), float(parts[2]), float(parts[3]));
            }
        }
        
    } catch (Exception e) {
        println("Error parsing serial data: " + e.getMessage());
    }
}

// Keyboard controls
void keyPressed() {
    if (key == 'g' || key == 'G') {
        show_gravity = !show_gravity;
        println("Gravity display: " + (show_gravity ? "ON" : "OFF"));
    }
    else if (key == 'm' || key == 'M') {
        show_magnetometer = !show_magnetometer;
        println("Magnetometer display: " + (show_magnetometer ? "ON" : "OFF"));
    }
    else if (key == 'a' || key == 'A') {
        show_acceleration = !show_acceleration;
        println("Acceleration display: " + (show_acceleration ? "ON" : "OFF"));
    }
    else if (key == 'w' || key == 'W') {
        show_world_frame = !show_world_frame;
        println("World frame display: " + (show_world_frame ? "ON" : "OFF"));
    }
    else if (key == 's' || key == 'S') {
        show_sensor_frame = !show_sensor_frame;
        println("Sensor frame display: " + (show_sensor_frame ? "ON" : "OFF"));
    }
    else if (key == 'r' || key == 'R') {
        // Reset camera view
        camera_rot_x = -30;
        camera_rot_y = 0;
        camera_rot_z = 0;
        camera_zoom = 200;
        println("Camera reset");
    }
    else if (key == 'c' || key == 'C') {
        // Clear data
        synchronized(data_lock) {
            quat_w = 1.0;
            quat_x = 0.0;
            quat_y = 0.0;
            quat_z = 0.0;
            euler_psi = 0.0;
            euler_theta = 0.0;
            euler_phi = 0.0;
            gravity_vec.set(0, 0, 1);
            mag_vec.set(1, 0, 0);
            accel_vec.set(0, 0, 0);

        }
        println("Data cleared");
    }
    else if (key == '+' || key == '=') {
        camera_zoom -= 20;
        println("Zoom: " + camera_zoom);
    }
    else if (key == '-' || key == '_') {
        camera_zoom += 20;
        println("Zoom: " + camera_zoom);
    }
    else if (keyCode == UP) {
        camera_rot_x -= 5;
    }
    else if (keyCode == DOWN) {
        camera_rot_x += 5;
    }
    else if (keyCode == LEFT) {
        camera_rot_y -= 5;
    }
    else if (keyCode == RIGHT) {
        camera_rot_y += 5;
    }
}

// Mouse controls for camera rotation
void mouseDragged() {
    float dx = mouseX - pmouseX;
    float dy = mouseY - pmouseY;
    
    camera_rot_y += dx * mouse_sensitivity;
    camera_rot_x += dy * mouse_sensitivity;
}
