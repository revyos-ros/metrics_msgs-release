from metro_benchmark_pub import BenchmarkPub
import time


def test_timer():
    bp = BenchmarkPub(None, '')
    bp.tick('')
    time.sleep(0.1)
    elapsed = bp.tock()
    assert elapsed >= 0.1
