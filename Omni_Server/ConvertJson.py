import json
import os
import argparse
import time
from datetime import datetime

def parse_encoder_log(file_path):
    """Parse encoder log file to JSON format matching client encoder data format"""
    data = []
    robot_id = 1
    
    with open(file_path, 'r') as f:
        # Skip header
        next(f)
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 4:  # Ensure we have time and 3 RPM values
                try:
                    # Get timestamp and RPM values
                    timestamp = int(time.time())  # Current time as unix timestamp
                    rpm1 = float(parts[1])
                    rpm2 = float(parts[2])
                    rpm3 = float(parts[3])
                    
                    # Format according to specified JSON structure
                    encoder_json = {
                        "id": robot_id,
                        "type": "encoder",
                        "data": [rpm1, rpm2, rpm3]
                    }
                    
                    data.append(encoder_json)
                except ValueError:
                    continue
    return data

def parse_log_messages(file_path):
    """Parse system log messages to JSON format matching client log data format"""
    data = []
    robot_id = 1
    
    with open(file_path, 'r') as f:
        # Skip header if exists
        first_line = f.readline()
        if not first_line.startswith("0"):  # If first line doesn't start with timestamp, it's a header
            f.seek(0)  # Reset file pointer
            next(f)  # Skip header
        else:
            f.seek(0)  # Reset file pointer
            
        for line in f:
            parts = line.strip().split(' ', 1)
            if len(parts) >= 2:
                try:
                    # Get log message
                    message = parts[1]
                    
                    # Format according to specified JSON structure
                    log_json = {
                        "id": robot_id,
                        "type": "log",
                        "message": message
                    }
                    
                    data.append(log_json)
                except ValueError:
                    continue
    return data

def parse_bno055_log(file_path):
    """Parse IMU sensor data to JSON format matching client IMU data format"""
    data = []
    robot_id = 1
    
    with open(file_path, 'r') as f:
        # Skip header
        next(f)
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 8:  # Ensure we have at least heading, pitch, roll and quaternion
                try:
                    # Extract values
                    timestamp = int(time.time())  # Current time as unix timestamp
                    heading = float(parts[1])
                    pitch = float(parts[2])
                    roll = float(parts[3])
                    w = float(parts[4])
                    x = float(parts[5])
                    y = float(parts[6])
                    z = float(parts[7])
                    
                    # Format according to specified JSON structure
                    imu_json = {
                        "id": robot_id,
                        "type": "bno055",
                        "data": {
                            "time": timestamp,
                            "euler": [heading, pitch, roll],
                            "quaternion": [w, x, y, z]
                        }
                    }
                    
                    # Add acceleration and gravity if available
                    if len(parts) >= 14 and parts[8] != "NA" and parts[9] != "NA" and parts[10] != "NA":
                        imu_json["data"]["acceleration"] = [
                            float(parts[8]), 
                            float(parts[9]), 
                            float(parts[10])
                        ]
                    
                    if len(parts) >= 14 and parts[11] != "NA" and parts[12] != "NA" and parts[13] != "NA":
                        imu_json["data"]["gravity"] = [
                            float(parts[11]), 
                            float(parts[12]), 
                            float(parts[13])
                        ]
                    
                    data.append(imu_json)
                except ValueError:
                    continue
    return data

def create_output_directory(output_dir):
    """Create output directory if it doesn't exist"""
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

def save_json_file(data, output_path):
    """Save data to JSON file"""
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)

def generate_json_strings(data):
    """Generate JSON strings as they would be sent by the client"""
    json_strings = []
    for item in data:
        json_strings.append(json.dumps(item))
    return json_strings

def main():
    parser = argparse.ArgumentParser(description='Convert Omni logs to JSON format')
    parser.add_argument('--encoder-log', default='logs/encoder_log_20250328_100755.txt', 
                       help='Path to encoder log file')
    parser.add_argument('--system-log', default='logs/log_log_20250328_100755.txt',
                       help='Path to system log file')
    parser.add_argument('--imu-log', default='logs/bno055_log_20250328_100755.txt',
                       help='Path to IMU log file')
    parser.add_argument('--output-dir', default='json_data',
                       help='Output directory for JSON files')
    parser.add_argument('--text-format', action='store_true',
                       help='Also save the JSON strings as they would be sent over the network')
    
    args = parser.parse_args()
    
    # Create output directory
    create_output_directory(args.output_dir)
    
    # Generate timestamp for file names
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Convert and save encoder data
    encoder_data = parse_encoder_log(args.encoder_log)
    encoder_json_path = os.path.join(args.output_dir, f"encoder_data_{timestamp}.json")
    save_json_file(encoder_data, encoder_json_path)
    print(f"Converted {len(encoder_data)} encoder data entries to {encoder_json_path}")
    
    # Save encoder data as text strings if requested
    if args.text_format:
        encoder_strings = generate_json_strings(encoder_data)
        encoder_txt_path = os.path.join(args.output_dir, f"encoder_strings_{timestamp}.txt")
        with open(encoder_txt_path, 'w') as f:
            for json_str in encoder_strings:
                f.write(json_str + '\n')
        print(f"Saved {len(encoder_strings)} encoder JSON strings to {encoder_txt_path}")
    
    # Convert and save log data
    log_data = parse_log_messages(args.system_log)
    log_json_path = os.path.join(args.output_dir, f"log_data_{timestamp}.json")
    save_json_file(log_data, log_json_path)
    print(f"Converted {len(log_data)} log entries to {log_json_path}")
    
    # Save log data as text strings if requested
    if args.text_format:
        log_strings = generate_json_strings(log_data)
        log_txt_path = os.path.join(args.output_dir, f"log_strings_{timestamp}.txt")
        with open(log_txt_path, 'w') as f:
            for json_str in log_strings:
                f.write(json_str + '\n')
        print(f"Saved {len(log_strings)} log JSON strings to {log_txt_path}")
    
    # Convert and save IMU data
    imu_data = parse_bno055_log(args.imu_log)
    imu_json_path = os.path.join(args.output_dir, f"imu_data_{timestamp}.json")
    save_json_file(imu_data, imu_json_path)
    print(f"Converted {len(imu_data)} IMU data entries to {imu_json_path}")
    
    # Save IMU data as text strings if requested
    if args.text_format:
        imu_strings = generate_json_strings(imu_data)
        imu_txt_path = os.path.join(args.output_dir, f"imu_strings_{timestamp}.txt")
        with open(imu_txt_path, 'w') as f:
            for json_str in imu_strings:
                f.write(json_str + '\n')
        print(f"Saved {len(imu_strings)} IMU JSON strings to {imu_txt_path}")
    
    # Create a combined file with all data types
    all_data = {
        "encoder": encoder_data,
        "log": log_data,
        "imu": imu_data
    }
    combined_json_path = os.path.join(args.output_dir, f"combined_data_{timestamp}.json")
    save_json_file(all_data, combined_json_path)
    print(f"Created combined JSON file at {combined_json_path}")

if __name__ == "__main__":
    main()