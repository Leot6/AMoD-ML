
from src.utility.utility_functions import *


class DemandGenerator(object):
    def __init__(self,
                 _path_to_taxi_data: str,
                 _simulation_start_time: str,
                 _request_density: float,
                 _init_request_idx: int = 0):
        t = timer_start()
        self.system_time_ms = 0
        with open(_path_to_taxi_data, "rb") as f:
            self.all_requests = pickle.load(f)
        self.init_request_time_ms = compute_the_accumulated_seconds_from_0_clock(_simulation_start_time) * 1000
        self.init_request_idx = _init_request_idx
        while self.all_requests[self.init_request_idx].request_time_ms < self.init_request_time_ms:
            self.init_request_idx += 1
        self.current_request_count = 0
        self.request_density = _request_density
        print(f"[INFO] Demand Generator is ready. ({timer_end(t)})")

    def get_requests(self, target_system_time_ms: int) -> list[Request]:
        assert (self.system_time_ms <= target_system_time_ms)
        self.system_time_ms = target_system_time_ms
        requests = []

        new_request_idx = self.init_request_idx + int(self.current_request_count / self.request_density)
        while self.all_requests[new_request_idx].request_time_ms < self.system_time_ms + self.init_request_time_ms:
            new_request = copy.deepcopy(self.all_requests[new_request_idx])
            new_request.request_time_ms -= self.init_request_time_ms
            self.current_request_count += 1
            new_request_idx = self.init_request_idx + int(self.current_request_count / self.request_density)
            requests.append(new_request)

        return requests
