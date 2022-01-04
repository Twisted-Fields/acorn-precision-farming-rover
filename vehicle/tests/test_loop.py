import time
from loop import Perceptor
from concurrent.futures import ThreadPoolExecutor

_ms = 1 / 1000


def test_perceptor():
    counter = 0

    def collect():
        nonlocal counter
        counter += 1
        return counter

    p = Perceptor(collect)
    assert p.last() == counter
    assert p.last(2) == [counter], "should only keep one value by default"
    v, ts = p.last(with_timestamp=True)
    assert v == counter
    assert time.time() - ts < _ms
    v, ts = p.last(2, with_timestamp=True)[0]
    assert v == counter
    assert time.time() - ts < _ms
    # even with n=0, as it runs the collect function, there's still one entry meet the criterion.
    v, ts = p.last_n_seconds(0)[0]
    assert v == counter
    assert time.time() - ts < _ms

    p = Perceptor(collect, max_frequency=10, max_history=2)
    assert p.last() == counter
    prev = counter
    assert p.last() == prev, "the collect function should be called no more than 10 times per second"
    assert p.last(2) == [prev]
    assert p.last(10) == [prev]
    assert p.last_n_seconds(0) == [], "no result since no collection happening"
    v, ts = p.last_n_seconds(_ms)[0]
    assert v == prev
    assert time.time() - ts < _ms
    time.sleep(1 / 10)
    assert p.last() == prev + 1
    assert p.last(2) == [prev, prev + 1]
    assert p.last(10) == [prev, prev + 1]
    history = p.last_n_seconds(1 / 9)
    assert len(history) == 2
    assert history[0][0] == prev
    assert history[1][0] == prev + 1


def test_perceptor_with_executor():
    counter = 0

    def collect():
        nonlocal counter
        time.sleep(1 / 10)
        counter += 1
        return counter
    pe = ThreadPoolExecutor()
    p = Perceptor(collect, max_frequency=1 / _ms, executor=pe, max_history=2)
    assert p.last() is None, "collect function should have not finished yet"
    time.sleep(1 / 9)
    assert p.last() == 1, "finished, collect again"
    time.sleep(_ms)
    assert p.last() == 1, "last one not finished yet, but collect again"
    time.sleep(1 / 9)
    assert p.last() == 3, "both finished"
