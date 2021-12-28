from functools import partial
import socket
import threading

import ipc
import zmq
import time


def get_unused_port():
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    _, port = sock.getsockname()
    sock.close()
    return port


def test_leaky_zmq():
    addr = f"tcp://127.0.0.1:{get_unused_port()}"

    ready = threading.Event()

    def publisher():
        pub = ipc.ZMQPub(addr)
        pa = partial(pub.pub, b'A')
        pb = partial(pub.pub, b'B')
        ready.wait()
        for i in range(100):
            pa(i)
            pb(100 + i)
        pa('a')
        pb('b')
        pub.close()

    def subscriber():
        sub = ipc.LeakyZMQSub()
        suba = sub.sub(addr, b"A")
        subb = sub.sub(addr, b"B")
        # subscribing to non-existent peer should have no error
        addr2 = f"tcp://127.0.0.1:{get_unused_port()}"
        _ = sub.sub(addr2, b"A")
        _ = sub.sub(addr2, b"C")
        _ = sub.sub(addr2, b"C")
        time.sleep(0.1)  # leave some time for zmq thread to finish the subscriptions.
        ready.set()
        got = 0
        while got < 2:
            if o := suba():
                assert o == 'a', "should correctly get the last available message for the topic"
                got += 1
            if o := subb():
                assert o == 'b', "should correctly get the last available message for the topic"
                got += 1
        assert suba() is None, "should get each message only once"
        assert subb() is None, "should get each message only once"
        sub.close()

    targets = [publisher, subscriber]
    threads = []
    for t in targets:
        thread = threading.Thread(target=t)
        thread.start()
        threads.append(thread)
    for t in threads:
        t.join(2)
        assert not t.is_alive(), "threads should have terminated gracefully."
