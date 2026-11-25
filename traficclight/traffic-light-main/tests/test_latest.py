import threading
import time
import unittest

from traffic_light_stream.latest import LatestSlot


class LatestSlotTest(unittest.TestCase):
    def test_replace_keeps_only_newest_and_counts_overwrite(self):
        slot = LatestSlot()
        self.assertTrue(slot.replace(b"first", generation=0))
        self.assertTrue(slot.replace(b"second", generation=0))
        item = slot.take_latest(timeout=0)
        self.assertEqual(item.value, b"second")
        self.assertEqual(item.generation, 0)
        self.assertEqual(slot.dropped_count, 1)
        self.assertIsNone(slot.take_latest(timeout=0))

    def test_generation_rejects_stale_worker_result(self):
        slot = LatestSlot()
        old_generation = slot.generation
        self.assertEqual(slot.clear(advance_generation=True), 1)
        self.assertFalse(slot.replace("late", generation=old_generation))
        self.assertTrue(slot.replace("current", generation=1))
        self.assertEqual(slot.take_latest(timeout=0).value, "current")

    def test_generation_bound_wait_does_not_cross_reconnect(self):
        slot = LatestSlot()
        previous = slot.generation
        slot.clear(advance_generation=True)
        slot.replace("new connection", generation=slot.generation)
        self.assertIsNone(slot.take_latest(timeout=0, generation=previous))
        self.assertEqual(slot.take_latest(timeout=0, generation=slot.generation).value,
                         "new connection")

    def test_clear_discards_pending_item(self):
        slot = LatestSlot()
        slot.replace("pending", generation=0)
        self.assertEqual(slot.clear(), 0)
        self.assertIsNone(slot.take_latest(timeout=0))

    def test_timeout_is_bounded(self):
        slot = LatestSlot()
        start = time.monotonic()
        self.assertIsNone(slot.take_latest(timeout=0.03))
        elapsed = time.monotonic() - start
        self.assertGreaterEqual(elapsed, 0.02)
        self.assertLess(elapsed, 0.3)

    def test_timeout_rejects_nonfinite_and_unrepresentable_numbers(self):
        slot = LatestSlot()
        for timeout in (True, float("nan"), float("inf"), 10 ** 1000):
            with self.subTest(timeout=type(timeout).__name__):
                with self.assertRaises(ValueError):
                    slot.take_latest(timeout=timeout)

    def test_close_wakes_waiter_and_rejects_publish(self):
        slot = LatestSlot()
        result = []
        waiting = threading.Event()

        def consume():
            waiting.set()
            result.append(slot.take_latest(timeout=5))

        thread = threading.Thread(target=consume)
        thread.start()
        self.assertTrue(waiting.wait(0.5))
        slot.close()
        thread.join(0.5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [None])
        self.assertFalse(slot.replace("after-close", generation=0))

    def test_concurrent_producers_remain_depth_one(self):
        slot = LatestSlot()
        barrier = threading.Barrier(9)

        def produce(number):
            barrier.wait()
            slot.replace(number, generation=0)

        threads = [threading.Thread(target=produce, args=(number,)) for number in range(8)]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join()
        item = slot.take_latest(timeout=0)
        self.assertIn(item.value, range(8))
        self.assertEqual(slot.dropped_count, 7)
        self.assertIsNone(slot.take_latest(timeout=0))


if __name__ == "__main__":
    unittest.main()

