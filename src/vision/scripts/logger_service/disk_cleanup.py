import time
import datetime
import os
import shutil
import subprocess
from pathlib import Path
from datetime import datetime


# Function to check disk usage as a percentage
def get_disk_usage_percentage(path):
    result = subprocess.run(['df', '--output=pcent', path], capture_output=True, text=True)
    usage = result.stdout.splitlines()[1].strip()
    return int(usage.rstrip('%'))


# Function to delete old log directories in vision_log until disk usage is below target
def delete_old_log_directories(log_dir, target_usage_percentage):
    # List all directories in the vision_log folder
    directories = [d for d in log_dir.iterdir() if d.is_dir()]
    
    # Sort directories by creation/modification time (oldest first)
    directories.sort(key=lambda d: d.stat().st_mtime)

    # Delete directories until disk usage is below the target
    for directory in directories[:-1]:
        if get_disk_usage_percentage(str(log_dir.parent)) <= target_usage_percentage:
            break
        print(f"Deleting directory: {directory}", flush=True)
        shutil.rmtree(directory)


# Main function to perform the cleanup
def main():
    workspace_dir = Path(os.getenv('HOME')) / 'Workspace'
    vision_log_dir = workspace_dir / 'vision_log'

    usage_upper_limit = 85
    usage_lower_limit = 25
    while True:
        # Check the current disk usage
        usage = get_disk_usage_percentage(workspace_dir)
        print(f"{datetime.now()} Current disk usage: {usage}%", flush=True)

        if usage > usage_upper_limit:
            print("Disk usage exceeded 70%. Starting cleanup...", flush=True)
            delete_old_log_directories(vision_log_dir, usage_lower_limit)
        time.sleep(1800)


if __name__ == "__main__":
    main()
