from metro_benchmark_msgs.msg import ComputeTime
import collections
import math
import time

BenchmarkContext = collections.namedtuple('BenchmarkContext', 'name id start')


class BenchmarkPub:
    def __init__(self, node, topic):
        if node:
            self.compute_time_pub = node.create_publisher(ComputeTime, topic, 1)
            self.logger = node.get_logger()
        else:
            self.compute_time_pub = None
            self.logger = None

        self.stack = []
        self.counter = collections.Counter()

    def tick(self, name):
        ident = f'{name}_{self.counter[name]}'
        cxt = BenchmarkContext(name, ident, time.time())
        self.counter[name] += 1
        self.stack.append(cxt)

    def tock(self, log=True):
        stop = time.time()
        cxt = self.stack.pop()
        duration_sec = stop - cxt.start
        d_nano_f, d_sec_f = math.modf(duration_sec)
        s_nano_f, s_sec_f = math.modf(stop)

        msg = ComputeTime()
        msg.header.frame_id = cxt.name
        msg.header.stamp.sec = int(s_sec_f)
        msg.header.stamp.nanosec = int(s_nano_f * 1e9)
        msg.duration.sec = int(d_sec_f)
        msg.duration.nanosec = int(d_nano_f * 1e9)
        msg.id = cxt.id
        if self.stack:
            msg.parent_id = self.stack[-1].id

        if self.compute_time_pub:
            self.compute_time_pub.publish(msg)
        if log and self.logger:
            self.logger.info(f'{cxt.name} time: {round(duration_sec, 4)}')

        return duration_sec
