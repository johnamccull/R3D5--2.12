import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

# Configuration
ROBOT_IP = '192.168.1.10'
ROBOT_PORT = 30004
CONFIG_FILE = 'path/to/your/configuration_file.xml'

# Target position: [X, Y, Z, RX, RY, RZ]
# Note: Specify the target position here. Values are in meters and radians.
target_position = [0.5, -0.2, 0.3, 0, 3.1415, 0]

# Setup RTDE communication and configuration
conf = rtde_config.ConfigFile(CONFIG_FILE)
output_names, output_types = conf.get_recipe('out')
input_names, input_types = conf.get_recipe('in')
con = rtde.RTDE(ROBOT_IP, ROBOT_PORT)
con.connect()

# Setup output and input recipes
con.send_output_setup(output_names, output_types)
con.send_input_setup(input_names, input_types)

# Start RTDE data synchronization
con.send_start()

# Create a new input packet to send to the robot
input_data = con.send_input_setup(input_names, input_types)

# Set the target position in the input packet
input_data.input_double_register_0 = target_position[0]
input_data.input_double_register_1 = target_position[1]
input_data.input_double_register_2 = target_position[2]
input_data.input_double_register_3 = target_position[3]
input_data.input_double_register_4 = target_position[4]
input_data.input_double_register_5 = target_position[5]

# Send the input packet to the robot to move the tool
con.send(input_data)

# Wait for the movement to complete
# Note: Implement appropriate waiting or checking mechanism based on your application needs

# Stop RTDE data synchronization
con.send_pause()

# Disconnect from the robot
con.disconnect()