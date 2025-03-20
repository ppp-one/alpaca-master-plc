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
                f"http://{ip}/",
                timeout=10,
            )
            print(_, r.text)
            total_count += 1

        except requests.exceptions.RequestException as e:
            print(e)

    print("--- %.2f seconds ---" % ((time.time() - start_time) / total_count))


if __name__ == "__main__":
    processes = []
    n_clients = 3
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
