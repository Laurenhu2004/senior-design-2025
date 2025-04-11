import subprocess
import time
import re
import os

def run_command(command, background=False):
    """Runs a command and optionally in the background."""
    if background:
        # Start the process in the background
        print(f"Running in background: {command}")
        process = subprocess.Popen(command, shell=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return process
    else:
        # Run the process and wait for it to complete
        print(f"Running: {command}")
        result = subprocess.run(command, shell=True, text=True, capture_output=True)
        if result.returncode == 0:
            print(f"Success: {command}")
            print(result.stdout)
        else:
            print(f"Error: {command}")
            print(result.stderr)

def monitor_processes(processes):
    """Monitor background processes and print their output/error."""
    for process in processes:
        # Polling to check if the process has finished
        stdout, stderr = process.communicate()
        print(f"Process completed with output: {stdout}")
        if stderr:
            print(f"Process error: {stderr}")

def extract_center_coordinates(output):
    """Extracts center_x and center_y from the output string."""
    # Use a regular expression to find the center_x and center_y values
    match = re.search(r"Center of bounding box: \((\d+), (\d+)\)", output)
    
    if match:
        # Extract the values and convert them to integers
        center_x = int(match.group(1))
        center_y = int(match.group(2))
        return center_x, center_y
    else:
        raise ValueError("Center coordinates not found in the output")

def main():
    initialize0 = run_command("roslaunch limo_cobot_moveit_config demo.launch", background=True) # turns on arm control
    process0 = run_command("rosrun mycobot_280_moveit sync_plan.py _port:=/dev/ttyACM0 _baud:=115200", background=True)
    initialize1 = run_command("roslaunch limo_bringup limo_start.launch pub_odom_tf:=false", background=True)  # starts LiDAR
    current_dir = os.getcwd()
    process1 = subprocess.Popen([
    'gnome-terminal',
    '--', 'bash', '-i', '-c', f'cd {current_dir} && source .venv/bin/activate && python text_and_obj.py'
    ])

    # Polling for the 'script_done.txt' file
    while not os.path.exists('script_done.txt'):
        print("Waiting for script to complete...")
        time.sleep(1)
    print("Script inside the terminal has finished.")

    with open('center_coordinates.txt', 'r') as f:
        line = f.readline()  # Read the first line from the file
        
        # Check if the line is not empty
        if line:
            # Use regular expression to extract the coordinates from the line
            match = re.search(r"Center of bounding box: \((\d+), (\d+)\)", line)
            
            if match:
                # Extract the center_x and center_y values as integers
                center_x = int(match.group(1))
                center_y = int(match.group(2))
                
                print(f"Read from file - Center of bounding box: ({center_x}, {center_y})")
            else:
                print("Error: Center coordinates not found in the file.")
        else:
            print("Error: The file is empty or does not contain the expected data.")

    # Set parameters for ROS
    cxpub = subprocess.call(['rosparam', 'set', '/cx', str(center_x)])
    cypub = subprocess.call(['rosparam', 'set', '/cy', str(center_y)])
    initialize2 = run_command("roslaunch limo_bringup limo_navigation_diff.launch", background=True)  # starts navigation
    time.sleep(8)
    initialize3 = run_command("roslaunch astra_camera dabai_u3.launch", background=True)  # turns on depth camera

    process2 = run_command("rosrun pak move_from_depth.py", background=True)
    time.sleep(5)
    process3 = run_command("rosrun pak depth_publisher.py")
    # process4 = run_command("rosrun pak arm_pick_pills.py")
    # time.sleep(10)

    try:
        os.remove('script_done.txt')
        print("Deleted script_done.txt.")
    except FileNotFoundError:
        print("script_done.txt not found.")

    try:
        os.remove('center_coordinates.txt')
        print("Deleted center_coordinates.txt.")
    except FileNotFoundError:
        print("center_coordinates.txt not found.")

    processes = [initialize0, initialize1, initialize2, initialize3]
    for process in processes:
        process.terminate()
        print(f"Terminated process: {process}")

    return

    # Monitor the background processes
    monitor_processes([initialize0, initialize1, initialize2, initialize3, process2, process3])

if __name__ == "__main__":
    main()
