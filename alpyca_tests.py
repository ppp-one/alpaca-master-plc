from alpaca.camera import *
from alpaca.telescope import *
from alpaca.dome import *
import time
import requests

remote = True
if remote:
    ip = "192.168.1.177"
else:
    ip = "localhost"

c = Camera(ip, 0)
t = Telescope(ip, 0)
d = Dome(ip, 0)


# print(c.Connected)
c.Connected = True
t.Connected = True
d.Connected = True
# print(t.Connected)
t.Park()
# print(c.CCDTemperature)

start_time = time.time()
total = 1000
for i in range(total):
    # print(t.SiderealTime)
    # print(c.CCDTemperature)
    r = requests.put(
        f"http://{ip}/api/v1/telescope/0/connected",
        data={"Connected": True, "ClientTransactionID": i, "ClientID": 999},
    )
    print(r.text)
    t.Connected = True
    # # print(c.CCDTemperature)
    # print(t.Connected)
    # t.Connected = False
    # print(t.Connected)

    # time.sleep(1)

print("--- %.2f seconds ---" % ((time.time() - start_time) / total))
