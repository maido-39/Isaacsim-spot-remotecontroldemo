from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# Import after SimulationApp initialization
import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
import omni.kit.commands
import omni
import carb

# Use the non-deprecated API imports
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid

# Create World instance
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Create Cube 1 (keyboard controlled - movable) - Start closer to target
cube1 = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube1",
        name="movable_cube",
        position=np.array([0.0, 0.0, 0.5]),
        scale=np.array([0.3, 0.3, 0.3]),
        color=np.array([1.0, 0.0, 0.0]),  # Red cube
        mass=1.0
    )
)

# Create Cube 2 (fixed) - Closer for easier testing
cube2 = world.scene.add(
    FixedCuboid(
        prim_path="/World/Cube2",
        name="fixed_cube",
        position=np.array([1.0, 0.0, 0.5]),  # Closer: 1m instead of 2m
        scale=np.array([0.3, 0.3, 0.3]),
        color=np.array([0.0, 0.0, 1.0])  # Blue cube
    )
)

# Import contact sensor before world reset
from isaacsim.sensors.physics import ContactSensor, _sensor

# Reset the world to initialize all objects
world.reset()

# Get the stage and cube2 prim for sensor attachment
stage = omni.usd.get_context().get_stage()
cube2_prim = stage.GetPrimAtPath("/World/Cube2")

# Note: The ContactSensor class automatically adds Contact Report API
# Reference: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_physics_contact.html
# "Both the command and the wrapper class automatically add a Contact Report API to the parent prim"

# Create contact sensor using the wrapper class
# The sensor will be created dynamically when simulation plays
contact_sensor = ContactSensor(
    prim_path="/World/Cube2/ContactSensor",
    name="Cube2_ContactSensor",
    frequency=60,  # 60Hz update rate
    translation=np.array([0, 0, 0]),
    min_threshold=0.0,
    max_threshold=10000000,
    radius=0.5  # Detection radius within 0.5m
)

print(f"✓ Contact sensor defined at: {contact_sensor.prim_path}")
print(f"✓ Sensor will initialize when simulation plays...")

# Initialize the sensor (must be called before play)
contact_sensor.initialize()
print(f"✓ Sensor initialized")

# Get the interface for sensor reading
contact_sensor_interface = _sensor.acquire_contact_sensor_interface()
print(f"✓ Contact sensor interface acquired")

# Variables for keyboard control
cube1_velocity = np.array([0.0, 0.0, 0.0])
movement_speed = 2.0

# Create arrow for force visualization
arrow_path = "/World/ForceArrow"
xform = UsdGeom.Xform.Define(stage, arrow_path)
cone = UsdGeom.Cone.Define(stage, f"{arrow_path}/Cone")
cone.GetRadiusAttr().Set(0.05)
cone.GetHeightAttr().Set(0.3)
cone.GetAxisAttr().Set("Z")

# Color the arrow green
cone.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])

# Initially hide the arrow
xform_prim = stage.GetPrimAtPath(arrow_path)
UsdGeom.Imageable(xform_prim).MakeInvisible()

print("\n" + "="*60)
print("CONTACT SENSOR DEMO")
print("="*60)
print("Controls:")
print("  W/S - Move Cube1 forward/backward (Y-axis)")
print("  A/D - Move Cube1 left/right (X-axis)")
print("  Q/E - Move Cube1 up/down (Z-axis)")
print("  R   - Reset Cube1 position")
print("  P   - Print current cube positions")
print("\nGoal: Move the RED cube (0,0,0.5) to touch the BLUE cube (1,0,0.5)")
print("      Press 'D' to move right toward the blue cube!")
print("="*60 + "\n")

# Keyboard input handling
def on_keyboard_event(event, *args, **kwargs):
    global cube1_velocity
    
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input == carb.input.KeyboardInput.W:
            cube1_velocity[1] = movement_speed
        elif event.input == carb.input.KeyboardInput.S:
            cube1_velocity[1] = -movement_speed
        elif event.input == carb.input.KeyboardInput.A:
            cube1_velocity[0] = -movement_speed
        elif event.input == carb.input.KeyboardInput.D:
            cube1_velocity[0] = movement_speed
        elif event.input == carb.input.KeyboardInput.Q:
            cube1_velocity[2] = -movement_speed
        elif event.input == carb.input.KeyboardInput.E:
            cube1_velocity[2] = movement_speed
        elif event.input == carb.input.KeyboardInput.R:
            # Reset position
            cube1.set_world_pose(position=np.array([0.0, 0.0, 0.5]))
            cube1.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
            print("Cube1 position reset!")
        elif event.input == carb.input.KeyboardInput.P:
            # Print positions
            pos1, _ = cube1.get_world_pose()
            pos2, _ = cube2.get_world_pose()
            distance = np.linalg.norm(pos1 - pos2)
            print(f"\n📍 Cube1 (Red): [{pos1[0]:.3f}, {pos1[1]:.3f}, {pos1[2]:.3f}]")
            print(f"📍 Cube2 (Blue): [{pos2[0]:.3f}, {pos2[1]:.3f}, {pos2[2]:.3f}]")
            print(f"📏 Distance: {distance:.3f}m\n")
    
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        # Stop movement when key is released
        if event.input in [carb.input.KeyboardInput.W, carb.input.KeyboardInput.S]:
            cube1_velocity[1] = 0
        elif event.input in [carb.input.KeyboardInput.A, carb.input.KeyboardInput.D]:
            cube1_velocity[0] = 0
        elif event.input in [carb.input.KeyboardInput.Q, carb.input.KeyboardInput.E]:
            cube1_velocity[2] = 0
    
    return True

# Subscribe to keyboard events
appwindow = omni.appwindow.get_default_app_window()
input_interface = carb.input.acquire_input_interface()
keyboard = appwindow.get_keyboard()
sub_keyboard = input_interface.subscribe_to_keyboard_events(keyboard, on_keyboard_event)

# Main simulation loop
frame_count = 0
last_contact_state = False
sensor_initialized = False

print("Simulation started. Use WASD, Q/E to move the red cube.")
print("Press 'P' to check cube positions, 'D' to move right toward blue cube.\n")

# Print initial positions
pos1, _ = cube1.get_world_pose()
pos2, _ = cube2.get_world_pose()
print(f"Initial positions:")
print(f"  Red Cube:  [{pos1[0]:.3f}, {pos1[1]:.3f}, {pos1[2]:.3f}]")
print(f"  Blue Cube: [{pos2[0]:.3f}, {pos2[1]:.3f}, {pos2[2]:.3f}]")
print(f"  Distance: {np.linalg.norm(pos1 - pos2):.3f}m")
print("\nWaiting for contact sensor to initialize...\n")

def calculate_arrow_rotation(normal):
    """Calculate rotation quaternion to align arrow with normal vector"""
    # Default arrow points up (Z)
    default_dir = np.array([0, 0, 1])
    
    if np.linalg.norm(normal) < 0.001:
        return Gf.Quatf(1, 0, 0, 0)
    
    normal_normalized = normal / np.linalg.norm(normal)
    
    # Calculate rotation axis and angle
    v = np.cross(default_dir, normal_normalized)
    s = np.linalg.norm(v)
    c = np.dot(default_dir, normal_normalized)
    
    if s < 0.001:  # Vectors are parallel
        if c > 0:  # Same direction
            return Gf.Quatf(1, 0, 0, 0)
        else:  # Opposite direction
            return Gf.Quatf(0, 1, 0, 0)
    
    # Use Rodrigues' rotation formula
    v = v / s
    angle = np.arctan2(s, c)
    
    half_angle = angle / 2
    sin_half = np.sin(half_angle)
    cos_half = np.cos(half_angle)
    
    return Gf.Quatf(cos_half, v[0] * sin_half, v[1] * sin_half, v[2] * sin_half)

while simulation_app.is_running():
    world.step(render=True)
    
    if world.is_playing():
        frame_count += 1
        
        # Apply velocity to Cube1
        if np.any(cube1_velocity != 0):
            cube1.set_linear_velocity(cube1_velocity)
        else:
            # Apply damping when no key is pressed
            current_vel = cube1.get_linear_velocity()
            if np.linalg.norm(current_vel) > 0.01:
                cube1.set_linear_velocity(current_vel * 0.9)
        
        # Read contact sensor data every frame using wrapper class
        try:
            # Get current frame from sensor (returns dict with all data)
            frame_data = contact_sensor.get_current_frame()
            
            # Check if data is valid (sensor becomes valid after physics starts)
            is_valid = frame_data is not None and 'in_contact' in frame_data
            in_contact = frame_data.get('in_contact', False) if is_valid else False
            
            # Check if sensor just became valid
            if not sensor_initialized and is_valid:
                sensor_initialized = True
                print("✓ Contact sensor is now VALID and ready!\n")
            
            # Debug: Print sensor status every 60 frames (1 second)
            if frame_count % 60 == 0:
                if not is_valid:
                    print(f"[Debug] Frame {frame_count}: ⚠️ Sensor NOT valid yet (waiting for physics...)")
                else:
                    force = frame_data.get('force', 0.0)
                    print(f"[Debug] Frame {frame_count}: Sensor valid=True, in_contact={in_contact}, force={force:.2f} N")
            
            # Only process contacts if sensor is valid
            if is_valid:
                # Print contact status when it changes
                if in_contact != last_contact_state:
                    if in_contact:
                        force = frame_data.get('force', 0.0)
                        time_val = frame_data.get('time', 0.0)
                        num_contacts = frame_data.get('number_of_contacts', 0)
                        print(f"\n🔴 CONTACT DETECTED!")
                        print(f"   Force: {force:.2f} N")
                        print(f"   Time: {time_val:.3f}s")
                        print(f"   Number of contacts: {num_contacts}")
                    else:
                        print("⚪ No contact\n")
                        # Hide arrow when contact ends
                        UsdGeom.Imageable(xform_prim).MakeInvisible()
                    last_contact_state = in_contact
            
            # Visualize force with arrow
            if in_contact and is_valid:
                force_magnitude = frame_data.get('force', 0.0)
                
                # Get contact details from frame data
                contacts = frame_data.get('contacts', [])
                
                if len(contacts) > 0:
                    # Get first contact info
                    contact_info = contacts[0]
                    
                    # Extract position and normal from contact data
                    contact_position = np.array(contact_info.get('position', [0, 0, 0]))
                    contact_normal = np.array(contact_info.get('normal', [0, 0, 1]))
                    
                    # Scale arrow based on force
                    arrow_length = min(force_magnitude / 50.0, 3.0)
                    arrow_length = max(arrow_length, 0.1)  # Minimum visible size
                    
                    # Calculate rotation
                    quat = calculate_arrow_rotation(contact_normal)
                    
                    # Set arrow transform
                    xform_api = UsdGeom.Xformable(xform_prim)
                    xform_api.ClearXformOpOrder()
                    
                    translate_op = xform_api.AddTranslateOp()
                    translate_op.Set(Gf.Vec3d(contact_position[0], contact_position[1], contact_position[2]))
                    
                    orient_op = xform_api.AddOrientOp()
                    orient_op.Set(quat)
                    
                    scale_op = xform_api.AddScaleOp()
                    scale_op.Set(Gf.Vec3f(1.0, 1.0, arrow_length))
                    
                    # Make arrow visible
                    UsdGeom.Imageable(xform_prim).MakeVisible()
                    
                    # Print details with contact info
                    print(f"  └─ Contact position: [{contact_position[0]:.2f}, {contact_position[1]:.2f}, {contact_position[2]:.2f}]")
                    print(f"  └─ Contact normal: [{contact_normal[0]:.2f}, {contact_normal[1]:.2f}, {contact_normal[2]:.2f}]")
                    print(f"  └─ Arrow length: {arrow_length:.2f}m")
        except Exception as e:
            if frame_count % 60 == 0:
                print(f"[Error reading sensor]: {e}")
                import traceback
                traceback.print_exc()

# Cleanup
input_interface.unsubscribe_to_keyboard_events(keyboard, sub_keyboard)
simulation_app.close()
