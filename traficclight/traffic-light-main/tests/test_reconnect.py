import unittest

from traffic_light_stream.reconnect import ExponentialBackoff, connect_with_backoff


class ExponentialBackoffTest(unittest.TestCase):
    def test_growth_cap_jitter_and_reset(self):
        values = iter((0.5, 0.5, 0.5, 0.5, 0.5))
        policy = ExponentialBackoff(
            initial_s=0.1, maximum_s=0.4, multiplier=2,
            jitter_fraction=0.2, random_value=lambda: next(values),
        )
        self.assertEqual([policy.next_delay() for _ in range(4)], [0.1, 0.2, 0.4, 0.4])
        self.assertEqual(policy.attempt, 4)
        policy.reset()
        self.assertEqual(policy.next_delay(), 0.1)

    def test_jitter_is_bounded(self):
        low = ExponentialBackoff(jitter_fraction=0.2, random_value=lambda: 0).next_delay()
        high = ExponentialBackoff(jitter_fraction=0.2, random_value=lambda: 1).next_delay()
        self.assertAlmostEqual(low, 0.08)
        self.assertAlmostEqual(high, 0.12)

    def test_bad_configuration_and_random_value_fail(self):
        for values in (
            dict(initial_s=0), dict(initial_s=2, maximum_s=1),
            dict(multiplier=1), dict(jitter_fraction=1),
        ):
            with self.assertRaises(ValueError):
                ExponentialBackoff(**values)
        policy = ExponentialBackoff(random_value=lambda: float("nan"))
        with self.assertRaises(ValueError):
            policy.next_delay()

    def test_connect_retries_are_finite_and_success_resets_policy(self):
        attempts = []
        delays = []
        policy = ExponentialBackoff(jitter_fraction=0, random_value=lambda: 0.5)

        def connect():
            attempts.append(1)
            if len(attempts) < 3:
                raise ConnectionRefusedError
            return "connected"

        self.assertEqual(connect_with_backoff(
            connect, max_attempts=3, backoff=policy, sleep=delays.append
        ), "connected")
        self.assertEqual(delays, [0.1, 0.2])
        self.assertEqual(policy.attempt, 0)

    def test_connect_exhaustion_propagates_last_failure(self):
        attempts = []
        with self.assertRaises(ConnectionRefusedError):
            connect_with_backoff(
                lambda: (attempts.append(1), (_ for _ in ()).throw(ConnectionRefusedError))[0],
                max_attempts=2,
                backoff=ExponentialBackoff(jitter_fraction=0),
                sleep=lambda _: None,
            )
        self.assertEqual(len(attempts), 2)

    def test_connect_retry_respects_one_total_deadline(self):
        now = [1.0]
        attempts = []

        def fail():
            attempts.append(1)
            raise ConnectionRefusedError

        with self.assertRaises(TimeoutError):
            connect_with_backoff(
                fail,
                max_attempts=10,
                backoff=ExponentialBackoff(initial_s=0.1, jitter_fraction=0),
                sleep=lambda delay: now.__setitem__(0, now[0] + delay),
                deadline=1.25,
                clock=lambda: now[0],
            )
        self.assertEqual(len(attempts), 2)


if __name__ == "__main__":
    unittest.main()
