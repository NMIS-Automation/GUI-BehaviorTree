import numpy as np
import json

def numpy_to_python(data):
    if isinstance(data, np.ndarray):
        return data.tolist()  # Convert numpy array to a standard Python list
    elif isinstance(data, np.generic):
        return data.item()  # Convert numpy scalar to a Python scalar
    elif isinstance(data, dict):
        return {key: numpy_to_python(value) for key, value in data.items()}  # Recursively convert nested dictionaries
    else:
        return data



class json_:
    def save(self,filename,data):
        data = numpy_to_python(data)
        try:
            with open(filename, "w") as write_file:
                json.dump(data, write_file)
            return True
        except:
            return False
        

    def load(self,filename):
        return json.load(open(filename,'r'))



