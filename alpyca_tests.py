from alpaca.camera import *
from alpaca.telescope import *
from alpaca.dome import *
import time
import requests

remote = True
if remote:
    ip = "192.168.1.10"
else:
    ip = "localhost"

# c = Camera(ip, 0)
t = Telescope(ip, 0)
d = Dome(ip, 0)


# print(c.Connected)
# c.Connected = True
# t.Connected = True
# d.Connected = True
# print(t.AtPark)
# t.Park()
# print(t.CanMoveAxis(TelescopeAxes.axisPrimary))


# print(t.Connected)
# t.Park()
# print(c.CCDTemperature)
# d.CloseShutter()

# r = requests.put(
#     f"http://{ip}/api/v1/dome/0/openshutter",
#     data={"ClientTransactionID": 123, "ClientID": 999},
# )
# print(r.text)

start_time = time.time()
total = 10
for i in range(total):
    print(d.ShutterStatus)
    # print(t.SiderealTime)
    # print(c.CCDTemperature)
    r = requests.put(
        f"http://{ip}/api/v1/telescope/0/connected",
        data={"Connected": True, "ClientTransactionID": i, "ClientID": 999},
        timeout=10,
    )
    print(r.text)
    r = requests.put(
        f"http://{ip}/api/v1/dome/0/connected",
        data={"Connected": True, "ClientTransactionID": i, "ClientID": 999},
        timeout=10,
    )
    print(r.text)

    # t.Connected = True
    # d.Connected = True
# # print(c.CCDTemperature)
# print(t.Connected)
# t.Connected = False
# print(t.Connected)

# time.sleep(1)

print("--- %.2f seconds ---" % ((time.time() - start_time) / total))
