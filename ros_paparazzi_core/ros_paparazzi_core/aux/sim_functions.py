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
        if not data_path or not os.path.exists(data_path):
            raise ValueError(f"File Path not Found: {data_path}")

        self.get_logger().info(f"Using file path: {data_path}")
        return data_path


def get_column(data, column_index):
    column_data = []
    for row in data:
        if column_index < len(row):
            column_data.append(row[column_index])
    return column_data