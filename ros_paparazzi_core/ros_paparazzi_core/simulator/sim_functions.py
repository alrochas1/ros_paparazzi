import os
import yaml

def read_txt(self, filepath):

        self.data = []
            
        try:
            with open(filepath, 'r') as file:
                for line in file:
                    values = [float(x) if '.' in x else int(x) for x in line.strip().split()]
                    self.data.append(values)

        except Exception as e:
             print(f"Failed to read the data: {e}")            



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