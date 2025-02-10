# Auxiliar functions for the simulator

import os
import yaml

def read_txt(self, filepath):

        self.data = []
            
        try:
            with open(filepath, 'r') as file:
                for line in file:
                    values = [float(x) if 'e' in x or '.' in x else int(x) for x in line.strip().split()]
                    self.data.append(values)

        except Exception as e:
             print(f"Failed to read the data: {e}")            


def get_time_vector(file_name):
    try:
        with open(file_name, 'r') as file:
            time_vector = []
            for line in file:
                first_value = float(line.split()[0])
                time_vector.append(first_value)
            return time_vector
        
    except FileNotFoundError:
        print(f"Error: The file '{file_name}' doesnt exist")
        return []
    
    except ValueError:
        print(f"Error: Can convert some values of '{file_name}' to float")
        return []
    except Exception as e:
        print(f"UnexpectedError: {e}")
        return []


def load_config(self, config_path):
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration File not Found: {config_path}")

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        data_path = config.get('simulator_data_path')
        if not data_path:
            raise ValueError("Missing 'simulator_data_path' in configuration file")
        
        data_path = os.path.expanduser(data_path)
        if not os.path.exists(data_path):
            raise ValueError(f"File Path not Found: {data_path}")

        self.get_logger().info(f"Using file path: {data_path}")
        return data_path


def get_column(data, column_index):
    column_data = []
    for row in data:
        if column_index < len(row):
            column_data.append(row[column_index])
    return column_data


##############################################
############### READING FUNCTIONS ############
############################################## 

def get_imu_data(self):
        read_txt(self, os.path.join(self.base_dir, "imu_data.txt"))
        self.t_imu = get_column(self.data, 0)
        self.ax = get_column(self.data, 8)
        self.ay = get_column(self.data, 9)
        self.wz = get_column(self.data, 7)
        self.imu_index = 0

        self.px = get_column(self.data, 2)
        self.py = get_column(self.data, 3)


def get_gps_data(self):
        read_txt(self, os.path.join(self.base_dir, "gps_data.txt"))
        self.t_gps = get_column(self.data, 0)
        self.lat = get_column(self.data, 5)
        self.lon = get_column(self.data, 6)
        self.vx = get_column(self.data, 9)
        self.vy = get_column(self.data, 10)
        self.vz = get_column(self.data, 11)
        self.gps_index = 0


def get_attitude_data(self):
    read_txt(self, os.path.join(self.base_dir, "attitude_data.txt"))
    self.t_attitude = get_column(self.data, 0)
    self.theta_data = get_column(self.data, 3) # rad

    self.attitude_index = 0
    self.theta = 0

