# Importing libraries
import requests

# Setting URL
URL = "http://10.5.5.9/gp/gpControl/command/shutter?p=1"

# Empty params
PARAMS = {}

# Sending the rquest and saving response
r = requests.get(url = URL, params = PARAMS)

# Extracting data in JSON format
data = r.json()

print(data)
