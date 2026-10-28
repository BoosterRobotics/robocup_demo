import os
import shutil
import hashlib
import time
import argparse
import re
from datetime import datetime
import uuid

def is_valid_datetime_format(date_string):
    try:
        datetime.strptime(date_string, '%Y-%m-%d-%H-%M-%S')
        return True
    except ValueError:
        return False

def calculate_md5(file_path):
    hash_md5 = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

def copy_and_verify(source_file, target_file):
    while True:
        # Copy the file
        shutil.copy2(source_file, target_file)
        
        # Calculate hashes
        source_hash = calculate_md5(source_file)
        target_hash = calculate_md5(target_file)
        
        # Verify file integrity
        if source_hash == target_hash:
            os.remove(source_file)
            print(f'Copied and removed: {source_file} to {target_file} successfully')
            break
        else:
            print(f'Error: File integrity check failed for {source_file}. Retrying...')

def move_files(source_dir, target_dir, base_time_stamp = 0):
    # Ensure the target directory exists
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    # Iterate over all files in the source directory
    for filename in os.listdir(source_dir):
        if filename.endswith('.jpg'):
            # Construct full file path
            source_file = os.path.join(source_dir, filename)
            pose_file = filename.replace('.jpg', '.yaml').replace('color_', 'pose_')
            depth_file = filename.replace('.jpg', '.png').replace('color_', 'depth_')
            source_pose_file = os.path.join(source_dir, pose_file)
            source_depth_file = os.path.join(source_dir, depth_file)

            # fix
            match = re.search(r'_(\d+\.\d+)\.jpg', filename)
            if match and os.path.exists(source_pose_file) and os.path.exists(source_depth_file):
                add_on_timestamp = float(match.group(1))
            else:
                print(f'Error: File name format is incorrect for {filename}. Removing...')
                os.remove(source_file)
                continue

            # copy color img
            new_img_filename = 'color_' + str(base_time_stamp+add_on_timestamp) + '.jpg'
            target_img_file = os.path.join(target_dir, new_img_filename)
            copy_and_verify(source_file, target_img_file)

            # copy depth img
            new_depth_filename = 'depth_' + str(base_time_stamp+add_on_timestamp) + '.png'
            target_depth_file = os.path.join(target_dir, new_depth_filename)
            copy_and_verify(source_depth_file, target_depth_file)

            # copy pose yaml
            new_pose_filename = 'pose_' + str(base_time_stamp+add_on_timestamp) + '.yaml'
            target_pose_file = os.path.join(target_dir, new_pose_filename)
            copy_and_verify(source_pose_file, target_pose_file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Move JPG files from source to target directory.')
    parser.add_argument('--source-directory', type=str, help='The source directory to scan for JPG files.', default='/home/booster/Workspace/vision_log')
    parser.add_argument('--target-directory', type=str, help='The target directory to move JPG files to.', default='/mnt/vision_data')
    parser.add_argument('--simulation', action='store_true', help='Run the script in simulation mode.', default=False)

    args = parser.parse_args()

    source_directory = args.source_directory
    target_directory = args.target_directory
    simulation = args.simulation

    # mac address
    current_device = ':'.join(['{:02x}'.format((uuid.getnode() >> elements) & 0xff) for elements in range(0,2*6,2)][::-1])

    while True:
        time.sleep(10)  # Sleep for 10 seconds before scanning again

        if not os.path.exists(source_directory):
            print(f'Warning: Source directory {source_directory} does not exist.')

        dirs = sorted(
            [d for d in os.listdir(source_directory) if is_valid_datetime_format(d)],
            key=lambda x: datetime.strptime(x, '%Y-%m-%d-%H-%M-%S')
        )
        for dir in dirs[:-1]:
            if not any(os.scandir(os.path.join(source_directory, dir))):
                os.rmdir(os.path.join(source_directory, dir))
        dirs = sorted(
            [d for d in os.listdir(source_directory) if is_valid_datetime_format(d)],
            key=lambda x: datetime.strptime(x, '%Y-%m-%d-%H-%M-%S')
        )
        for dir in dirs:
            if os.path.isdir(os.path.join(source_directory, dir)) and re.match(r'^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$', dir) is not None:
                date = dir[:10]
                # filter out earlier than 2023
                if date < '2023-01-01':
                    continue
                base_time_stamp = datetime.strptime(dir, '%Y-%m-%d-%H-%M-%S').timestamp() if simulation else 0
                move_files(os.path.join(source_directory, dir), os.path.join(target_directory, current_device, date),base_time_stamp)