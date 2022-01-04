import zmq
import pickle
import multiprocessing as mp
from typing import Callable


class ZMQPub:
    """ZMQPub is a thin wrapper of a zMQ PUB socket binding to addr.
    If hwm is absent, it leaves the socket with the default high watermark. If context is absent, it uses the global zMQ context.
    """

    def __init__(self, addr: str, hwm: int = None, context=None):
        context = context or zmq.Context.instance()
        self.socket = context.socket(zmq.PUB)
        if hwm is not None:
            self.socket.set_hwm(hwm)
        self.socket.bind(addr)

    def pub(self, topic: bytes, obj):
        """publish arbitrary picklable object to the topic."""
        self.socket.send_multipart([topic, pickle.dumps(obj)])

    def close(self):
        self.socket.close()


class LeakyZMQSub:
    """LeakyZMQSub is a thin wrapper of a zMQ SUB socket. By design, it losses older messages if the publisher sends faster than the subscriber receives.
    If hwm is absent, it leaves the socket with the default high watermark. If context is absent, it uses the global zMQ context.
    """

    def __init__(self, hwm: int = None, context=None):
        self.topics_last_message = {}
        context = context or zmq.Context.instance()
        self.socket = context.socket(zmq.SUB)
        if hwm is not None:
            self.socket.set_hwm(hwm)

    def sub(self, addr: str, topic: bytes) -> Callable:
        """connect to the addr, subscribe to the topic and return a callable which always returns the last available message, or None if nothing available.
        It should only be called once for each topic.
        """
        self.socket.connect(addr)
        self.socket.setsockopt(zmq.SUBSCRIBE, topic)

        def f():
            msg = None
            while self.socket.poll(1):
                t, m = self.socket.recv_multipart()
                if t != topic:
                    self.topics_last_message[t] = m
                else:
                    msg = m

            if not msg:
                msg = self.topics_last_message.pop(topic, None)

            return pickle.loads(msg) if msg else None

        return f

    def close(self):
        self.socket.close()


class MemPubSub:
    """A pubsub implementation based on dict. For tests."""

    def __init__(self):
        self.topics_message = {}

    def pub(self, topic: bytes, obj):
        self.topics_message[topic] = pickle.dumps(obj)

    def sub(self, addr, topic: bytes):
        def f():
            msg = self.topics_message.pop(topic, None)
            return pickle.loads(msg) if msg else None

        return f

    def close(self):
        pass


class SharedObject:
    """IPC based on SyncManager.dict.
    It maintains a downward channel, which holds message from parent process to child, and an upward channel in reverse direction.
    """

    def __init__(self):
        self.mgr = mp.Manager()
        self.dict = self.mgr.dict()

    class _Channel:
        def __init__(self, shared, name):
            self.shared, self.name = shared, name

        def read(self):
            return self.shared.dict.get(self.name, None)

        def write(self, obj):
            self.shared.dict[self.name] = obj

    def pair(self):
        return SharedObject._Channel(self, 'downward'), SharedObject._Channel(self, 'upward')
