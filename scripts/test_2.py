# Importing libraries
import requests
import threading
import time

# Live-streaming


class GOPRO_LIVE_MON(object):

    def __init__(self, url, params):
        self.url = url
        self.params = params
        thread = threading.Thread(target=self.run, args=(url, params))
        thread.daemon = True
        thread.start()

    def run(self, url, pms):
        while True:

            # Sending the request and saving response
            r = requests.get(url=URL, params=pms)
            # print(r)

            time.sleep(10)


# Setting URL
URL = "http://10.5.5.9/gp/gpControl/execute?p1=gpStream&a1=proto_v2&c1=restart"

# Empty params
PARAMS = {}


# Extracting data in JSON format
# data = r.json()

# Starting monitor thread

example = GOPRO_LIVE_MON(URL, PARAMS)

try:
    while True:
        time.sleep(1)
        print("GOPRO LIVE!")

except KeyboardInterrupt:
    print("GOPRO DOWN!")

# print(data)
