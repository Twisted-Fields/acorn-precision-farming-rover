import time
import itertools
from collections import deque
from typing import Callable
from concurrent.futures import Executor


class Perceptor:
    """Perceptor represents a perceptor in the perception-action loop. The main purpose is to bridge the mismatch of running frequency betwen the loop and the perceptor.
    For example, the loop may be designed to run 10 times per second, but the GPS module may only update once per second; Or, getting data from the GPS module may take 0.5s so it would block the loop from running. To solve either case, the GPS module can to be wrapped in a Perceptor with proper parameters.
    Depending on the use case, sometimes historical records can be quite useful too. Perceptor allows specifying how long the history need to be kept.

    fn is the collect function. It should take no parameter, but a closure can be used if necessary.
    By default, fn is called whenever last() or last_n_seconds() is called, but if max_frequency > 0, fn is called no faster than the max_frequency.
    If max_frequency is none zero and executor is not None, then fn is run within the executor, so the last() or last_n_seconds() call return immediately with previously collected data.
    Lastly, max_history controls how many some historical data to be kept.

    A deque is used internally to keep the history. The use of deque here is thread-safe based on what the author said at https://stackoverflow.com/a/58679519/1508412.
    """

    def __init__(self, fn: Callable, max_frequency: int = 0, executor: Executor = None, max_history: int = 1):
        self.fn = fn
        self.min_interval = 0
        self.executor = executor
        self.history = deque(maxlen=max_history)
        if max_frequency > 0:
            self.min_interval = 1.0 / max_frequency
            self.last_collect = 0.0

    def last(self, n: int = 1, with_timestamp=False):
        """By default, return the last collected data, which may be None if an executor is present.
        If n > 1, return a list of at most n last collected values, in chronological order.
        If with_timestamp is True, return a pair of (data, ts), or a list of pairs, depending on parameter 'n'.
        If runs the collect function when required.
        """
        if n < 1:
            raise ValueError
        self._maybe_collect()
        size = len(self.history)
        if n == 1:
            if size >= 1:
                v = self.history[size - n]
                return v if with_timestamp else v[0]
            else:
                return None
        if n > size:
            n = size
        start = size - n
        for i in range(start, size):
            s = itertools.islice(self.history, i, size)
            if with_timestamp:
                return list(s)
            else:
                return [v for (v, ts) in s]
        else:
            return []

    def last_n_seconds(self, n: float):
        """Return a list of (value, ts) pairs collected in the last n seconds before last_n_seconds() is called, in chronological order, or an empty list if none match.
        If runs the collect function when required."""
        cutoff = time.time() - n
        self._maybe_collect()
        for i, e in enumerate(self.history):
            if e and e[1] >= cutoff:
                return list(itertools.islice(self.history, i, len(self.history)))

        return []

    def _maybe_collect(self):
        if self.min_interval == 0:
            self.history.append((self.fn(), time.time()))
        elif time.time() - self.last_collect > self.min_interval:
            self.last_collect = time.time()
            if not self.executor:
                v, ts = self.fn(), time.time()
                self.history.append((v, ts))
            else:
                future = self.executor.submit(self.fn)

                def assign(future):
                    v, t = future.result(), time.time()
                    self.history.append((v, t))
                future.add_done_callback(assign)


class Actuator:
    """Actuator represents an actuator in the perception-action loop.
    """

    pass
