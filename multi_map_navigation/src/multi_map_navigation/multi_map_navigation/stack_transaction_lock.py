"""Cross-process ownership for complete navigation-stack transactions."""

import fcntl
import threading
import time


class StackTransactionLock:
    """A non-blocking, thread-reentrant flock shared by stack controllers."""

    PATH = '/tmp/multi_map_navigation_stack_transaction.lock'

    def __init__(self):
        self._state_lock = threading.Lock()
        self._owner_thread = None
        self._depth = 0
        self._file = None

    def acquire(self, timeout_sec: float = 0.0) -> bool:
        """Acquire ownership, optionally waiting a bounded amount of time."""
        thread_id = threading.get_ident()
        timeout_sec = max(0.0, float(timeout_sec))
        deadline = time.monotonic() + timeout_sec

        while True:
            with self._state_lock:
                if self._owner_thread == thread_id:
                    self._depth += 1
                    return True
                if self._owner_thread is None:
                    lock_file = open(self.PATH, 'a+', encoding='utf-8')
                    try:
                        fcntl.flock(
                            lock_file.fileno(),
                            fcntl.LOCK_EX | fcntl.LOCK_NB,
                        )
                    except BlockingIOError:
                        lock_file.close()
                    else:
                        self._file = lock_file
                        self._owner_thread = thread_id
                        self._depth = 1
                        return True

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.05, remaining))

    def release(self):
        thread_id = threading.get_ident()
        with self._state_lock:
            if self._owner_thread != thread_id or self._depth <= 0:
                raise RuntimeError('stack transaction lock released by non-owner')
            self._depth -= 1
            if self._depth:
                return
            lock_file = self._file
            self._file = None
            self._owner_thread = None
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            finally:
                lock_file.close()
