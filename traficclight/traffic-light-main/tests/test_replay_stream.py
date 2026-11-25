import hashlib
import socket
import threading
import time
import unittest

from traffic_light_stream import ProtocolError
from traffic_light_stream.receiver import VideoReceiver
from traffic_light_stream.sender import (
    FrameWriteTimeout, ReplayFrameBuffer, StaleSourceFrame, send_all_with_deadline,
)
from traffic_light_stream.source import ReplaySource
from traffic_light_stream.video_protocol import encode_video_frame


CAMERA_ID = "replay:front-traffic-port"
HOST_BOOT_ID = "11111111-1111-4111-8111-111111111111"
SENDER_ID = "22222222-2222-4222-8222-222222222222"


class IncrementingClock:
    def __init__(self, start=1_000_000_000, step=1_000_000):
        self.value = start - step
        self.step = step

    def __call__(self):
        self.value += self.step
        return self.value


def jpeg(number):
    return b"\xff\xd8" + number.to_bytes(4, "big") + b"\xff\xd9"


def source(frames, clock=None):
    return ReplaySource(
        frames,
        camera_id=CAMERA_ID,
        width=640,
        height=480,
        host_boot_id=HOST_BOOT_ID,
        sender_instance_id=SENDER_ID,
        clock_ns=clock or IncrementingClock(),
    )


def metadata(sequence=1, capture_ns=1_000_000_000, **updates):
    value = {
        "source_camera_id": CAMERA_ID,
        "source_host_boot_id": HOST_BOOT_ID,
        "sender_instance_id": SENDER_ID,
        "source_boot_id": "33333333-3333-4333-8333-333333333333",
        "source_frame_seq": sequence,
        "source_capture_monotonic_ns": capture_ns,
        "source_dequeue_monotonic_ns": capture_ns,
        "timestamp_basis": "replay_monotonic_generated",
        "source_mode": "replay",
        "width": 640,
        "height": 480,
        "encoding": "jpeg",
    }
    value.update(updates)
    return value


class ReplaySourceTest(unittest.TestCase):
    def test_replay_identity_is_separate_and_epoch_resets_sequence(self):
        with self.assertRaises(ProtocolError):
            ReplaySource([jpeg(1)], camera_id="front-traffic-port", width=640, height=480)
        replay = source([jpeg(1), jpeg(2)])
        first_epoch = replay.begin_epoch()
        first = replay.next_frame()
        second_epoch = replay.begin_epoch()
        second = replay.next_frame()
        self.assertNotEqual(first_epoch, second_epoch)
        self.assertEqual(first.metadata["source_frame_seq"], 1)
        self.assertEqual(second.metadata["source_frame_seq"], 1)
        self.assertEqual(second.jpeg, jpeg(2))

    def test_clock_repetition_is_made_strictly_monotonic(self):
        replay = source([jpeg(1), jpeg(2)], clock=lambda: 5)
        replay.begin_epoch()
        values = [replay.next_frame().metadata["source_capture_monotonic_ns"] for _ in range(2)]
        self.assertEqual(values, [5, 6])


class ReplayFrameBufferTest(unittest.TestCase):
    def test_30hz_producer_10hz_consumer_observes_newest_not_fifo(self):
        replay = source([jpeg(i) for i in range(1, 31)])
        buffer = ReplayFrameBuffer(replay)
        generation = buffer.begin_epoch()
        observed = []
        for batch in range(10):
            for _ in range(3):
                buffer.capture_one(generation=generation)
            captured = 1_000_000_000 + (batch * 3 + 2) * 1_000_000
            frame = buffer.take_fresh(
                generation=generation,
                max_age_s=1,
                timeout_s=0,
                clock_ns=lambda captured=captured: captured,
            )
            observed.append(frame.metadata["source_frame_seq"])
        self.assertEqual(observed, list(range(3, 31, 3)))
        self.assertEqual(buffer.slot.dropped_count, 20)

    def test_stale_frame_and_old_generation_are_rejected(self):
        buffer = ReplayFrameBuffer(source([jpeg(1), jpeg(2)]))
        old = buffer.begin_epoch()
        frame = buffer.capture_one(generation=old)
        with self.assertRaises(StaleSourceFrame):
            buffer.take_fresh(
                generation=old,
                max_age_s=0.01,
                timeout_s=0,
                clock_ns=lambda: frame.metadata["source_capture_monotonic_ns"] + 20_000_000,
            )
        buffer.abort_epoch(generation=old)
        current = buffer.begin_epoch()
        self.assertNotEqual(old, current)
        with self.assertRaises(ProtocolError):
            buffer.capture_one(generation=old)

    def test_generation_change_wakes_old_waiter(self):
        buffer = ReplayFrameBuffer(source([jpeg(1)]))
        generation = buffer.begin_epoch()
        result = []
        started = threading.Event()

        def wait_for_frame():
            started.set()
            result.append(buffer.slot.take_latest(5, generation=generation))

        worker = threading.Thread(target=wait_for_frame)
        worker.start()
        self.assertTrue(started.wait(0.5))
        buffer.abort_epoch(generation=generation)
        worker.join(0.5)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [None])


class DeadlineWriterTest(unittest.TestCase):
    def test_complete_write_preserves_socket_mode(self):
        left, right = socket.socketpair()
        try:
            left.settimeout(2.0)
            payload = b"complete"
            send_all_with_deadline(left, payload, timeout_s=0.2)
            self.assertEqual(left.gettimeout(), 2.0)
            self.assertEqual(right.recv(64), payload)
        finally:
            left.close()
            right.close()

    def test_paused_reader_hits_total_deadline_and_closes_writer(self):
        left, right = socket.socketpair()
        try:
            left.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
            started = time.monotonic()
            with self.assertRaises(FrameWriteTimeout):
                send_all_with_deadline(left, b"x" * (8 * 1024 * 1024), timeout_s=0.03)
            elapsed = time.monotonic() - started
            self.assertLess(elapsed, 0.5)
            self.assertEqual(left.fileno(), -1)
        finally:
            left.close()
            right.close()


class VideoReceiverTest(unittest.TestCase):
    def receiver(self):
        return VideoReceiver(expected_camera_id=CAMERA_ID, expected_source_mode="replay")

    def test_1000_frames_preserve_metadata_payload_association(self):
        receiver = self.receiver()
        generation = receiver.begin_connection(peer_role="video-sender")
        wire = bytearray()
        expected_hashes = {}
        for number in range(1, 1001):
            payload = jpeg(number)
            expected_hashes[number] = hashlib.sha256(payload).digest()
            wire.extend(encode_video_frame(metadata(number, number * 1000), payload))
        accepted = []
        for offset in range(0, len(wire), 137):
            accepted.extend(receiver.feed(
                wire[offset:offset + 137], generation=generation,
                receive_monotonic_ns=2_000_000_000,
            ))
        self.assertEqual(len(accepted), 1000)
        for frame in accepted:
            sequence = frame.metadata["source_frame_seq"]
            self.assertEqual(hashlib.sha256(frame.jpeg).digest(), expected_hashes[sequence])
        latest = receiver.take_latest(0)
        self.assertEqual(latest.value.metadata["source_frame_seq"], 1000)
        self.assertEqual(receiver.output.dropped_count, 999)

    def test_half_frame_eof_invalidates_generation_and_clears_output(self):
        receiver = self.receiver()
        generation = receiver.begin_connection(peer_role="video-sender")
        complete = encode_video_frame(metadata(), jpeg(1))
        receiver.feed(complete, generation=generation)
        receiver.feed(complete[:20], generation=generation)
        with self.assertRaises(ProtocolError):
            receiver.end_connection(generation=generation)
        self.assertIsNone(receiver.active_generation)
        self.assertIsNone(receiver.take_latest(0))

    def test_abort_models_rst_and_invalidates_pending_work(self):
        receiver = self.receiver()
        generation = receiver.begin_connection(peer_role="video-sender")
        receiver.feed(encode_video_frame(metadata(), jpeg(1)), generation=generation)
        next_generation = receiver.abort_connection(generation=generation)
        self.assertGreater(next_generation, generation)
        self.assertIsNone(receiver.take_latest(0))
        with self.assertRaises(ProtocolError):
            receiver.feed(b"x", generation=generation)

    def test_invalid_jpeg_duplicate_reverse_and_epoch_change_fail_closed(self):
        bad_records = (
            [encode_video_frame(metadata(), b"not-a-jpeg")],
            [encode_video_frame(metadata(2, 2000), jpeg(2)),
             encode_video_frame(metadata(2, 3000), jpeg(3))],
            [encode_video_frame(metadata(2, 2000), jpeg(2)),
             encode_video_frame(metadata(1, 3000), jpeg(1))],
            [encode_video_frame(metadata(1, 1000), jpeg(1)),
             encode_video_frame(metadata(2, 2000, source_boot_id="changed"), jpeg(2))],
        )
        for records in bad_records:
            with self.subTest(record_count=len(records)):
                receiver = self.receiver()
                generation = receiver.begin_connection(peer_role="video-sender")
                with self.assertRaises(ProtocolError):
                    receiver.feed(b"".join(records), generation=generation)
                self.assertIsNone(receiver.active_generation)
                self.assertIsNone(receiver.take_latest(0))

    def test_capture_time_must_increase_and_dequeue_cannot_precede_capture(self):
        records = (
            [encode_video_frame(metadata(1, 2000), jpeg(1)),
             encode_video_frame(metadata(2, 2000), jpeg(2))],
            [encode_video_frame(
                metadata(1, 2000, source_dequeue_monotonic_ns=1999), jpeg(1)
            )],
        )
        for pair in records:
            receiver = self.receiver()
            generation = receiver.begin_connection(peer_role="video-sender")
            with self.assertRaises(ProtocolError):
                receiver.feed(b"".join(pair), generation=generation)

    def test_one_active_source_and_tls_role_are_enforced(self):
        receiver = self.receiver()
        with self.assertRaises(ProtocolError):
            receiver.begin_connection(peer_role="controller")
        generation = receiver.begin_connection(peer_role="video-sender")
        with self.assertRaises(ProtocolError):
            receiver.begin_connection(peer_role="video-sender")
        receiver.abort_connection(generation=generation)

    def test_replay_cannot_claim_production_identity(self):
        with self.assertRaises(ProtocolError):
            VideoReceiver(
                expected_camera_id="front-traffic-port", expected_source_mode="replay"
            )
        with self.assertRaises(ProtocolError):
            VideoReceiver(expected_camera_id=CAMERA_ID, expected_source_mode="camera")


if __name__ == "__main__":
    unittest.main()
