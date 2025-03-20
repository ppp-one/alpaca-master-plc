import time
import requests
import multiprocessing

remote = True
if remote:
    ip = "192.168.1.10"
else:
    ip = "localhost"


def make_requests(ip, _):
    start_time = time.time()
    total = 10
    total_count = 0
    for i in range(total):
        try:
            r = requests.get(
                f"http://{ip}/api/v1/dome/0/shutterstatus",
                timeout=10,
            )
            print(_, "status", r.json()["Value"])

            r = requests.get(
                f"http://{ip}/api/v1/dome/0/atpark",
                timeout=10,
            )
            print(_, "atpark", r.json()["Value"])

            total_count += 1
            r = requests.get(
                f"http://{ip}/api/v1/telescope/0/altitude",
                timeout=10,
            )
            print(_, "alt", r.json()["Value"])
            total_count += 1
            r = requests.get(
                f"http://{ip}/api/v1/telescope/0/azimuth",
                timeout=10,
            )
            print(_, "az", r.json()["Value"])
            total_count += 1

            r = requests.get(
                f"http://{ip}/api/v1/telescope/0/rightascension",
                timeout=10,
            )
            print(_, "rightascension", r.json()["Value"])
            total_count += 1

            r = requests.get(
                f"http://{ip}/api/v1/telescope/0/declination",
                timeout=10,
            )
            print(_, "declination", r.json()["Value"])
            total_count += 1

            r = requests.get(
                f"http://{ip}/api/v1/telescope/0/tracking",
                timeout=10,
            )
            print(_, "tracking", r.json()["Value"])
            total_count += 1

            r = requests.get(
                f"http://{ip}/api/v1/telescope/0/slewing",
                timeout=10,
            )
            print(_, "slewing", r.json()["Value"])
            total_count += 1

            r = requests.get(
                f"http://{ip}/api/v1/telescope/0/atpark",
                timeout=10,
            )
            print(_, "atpark", r.json()["Value"])
            total_count += 1

            r = requests.put(
                f"http://{ip}/api/v1/telescope/0/connected",
                data={"Connected": True, "ClientTransactionID": i, "ClientID": 999},
                timeout=10,
            )
            total_count += 1

        except requests.exceptions.RequestException as e:
            print(e)

    print("--- %.2f seconds ---" % ((time.time() - start_time) / total_count))


if __name__ == "__main__":
    processes = []
    n_clients = 2
    for _ in range(n_clients):
        p = multiprocessing.Process(
            target=make_requests,
            args=(
                ip,
                _,
            ),
        )
        processes.append(p)
        p.start()

    for p in processes:
        p.join()
