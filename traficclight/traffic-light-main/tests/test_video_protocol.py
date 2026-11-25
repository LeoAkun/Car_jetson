import json
import random
import struct
import unittest

from traffic_light_stream import ProtocolError
from traffic_light_stream.video_protocol import (
    HEADER, MAGIC, VERSION, VideoFrameParser, encode_video_frame,
)


def metadata(**updates):
    value = {
        "source_camera_id": "front-traffic-port",
        "source_host_boot_id": "host-boot-uuid",
        "sender_instance_id": "sender-process-uuid",
        "source_boot_id": "stream-epoch-uuid",
        "source_frame_seq": 9271,
        "source_capture_monotonic_ns": 1234567890123,
        "source_dequeue_monotonic_ns": 1234567899123,
        "timestamp_basis": "v4l2_monotonic_eof",
        "source_mode": "camera",
        "width": 1920,
        "height": 1080,
        "encoding": "jpeg",
    }
    value.update(updates)
    return value


class VideoProtocolTest(unittest.TestCase):
    def setUp(self):
        self.jpeg = b"\xff\xd8fixture-jpeg\xff\xd9"
        self.encoded = encode_video_frame(metadata(), self.jpeg)

    def test_header_layout_is_network_endian_and_16_bytes(self):
        self.assertEqual(HEADER.size, 16)
        magic, version, flags, meta_length, jpeg_length = HEADER.unpack(self.encoded[:16])
        self.assertEqual((magic, version, flags), (MAGIC, VERSION, 0))
        self.assertEqual(jpeg_length, len(self.jpeg))
        self.assertGreater(meta_length, 0)

    def test_every_single_cut_decodes_the_same_frame(self):
        for cut in range(len(self.encoded) + 1):
            parser = VideoFrameParser()
            frames = parser.feed(self.encoded[:cut]) + parser.feed(self.encoded[cut:])
            self.assertEqual(len(frames), 1, cut)
            self.assertEqual(frames[0].jpeg, self.jpeg)
            self.assertEqual(dict(frames[0].metadata), metadata())

    def test_bytewise_and_random_chunks(self):
        parser = VideoFrameParser()
        frames = []
        for octet in self.encoded:
            frames += parser.feed(bytes([octet]))
        self.assertEqual(len(frames), 1)

        rng = random.Random(11)
        parser, frames, offset = VideoFrameParser(), [], 0
        while offset < len(self.encoded):
            size = rng.randint(1, 19)
            frames += parser.feed(self.encoded[offset:offset + size])
            offset += size
        self.assertEqual(len(frames), 1)

    def test_two_coalesced_frames_are_separate(self):
        second = encode_video_frame(metadata(source_frame_seq=9272), b"second")
        frames = VideoFrameParser().feed(self.encoded + second)
        self.assertEqual([frame.metadata["source_frame_seq"] for frame in frames], [9271, 9272])
        self.assertEqual([frame.jpeg for frame in frames], [self.jpeg, b"second"])

    def test_invalid_declared_lengths_fail_before_payload(self):
        for metadata_length, jpeg_length in (
                (0, 1), (4097, 1), (1, 0), (1, 4 * 1024 * 1024 + 1),
                (0xffffffff, 1), (1, 0xffffffff)):
            parser = VideoFrameParser()
            with self.assertRaises(ProtocolError):
                parser.feed(HEADER.pack(MAGIC, VERSION, 0, metadata_length, jpeg_length))

    def test_unknown_version_flags_and_magic_fail_closed(self):
        for header in (
                HEADER.pack(b"BAD!", VERSION, 0, 1, 1),
                HEADER.pack(MAGIC, 2, 0, 1, 1),
                HEADER.pack(MAGIC, VERSION, 1, 1, 1)):
            parser = VideoFrameParser()
            with self.assertRaises(ProtocolError):
                parser.feed(header)
            with self.assertRaises(ProtocolError):
                parser.feed(self.encoded)

    def test_incomplete_header_metadata_and_jpeg_fail_at_eof(self):
        meta_size = struct.unpack("!I", self.encoded[8:12])[0]
        for fragment in (self.encoded[:5], self.encoded[:16 + meta_size - 1], self.encoded[:-1]):
            parser = VideoFrameParser()
            self.assertEqual(parser.feed(fragment), [])
            with self.assertRaises(ProtocolError):
                parser.finish()

    def test_duplicate_key_nan_unknown_and_bool_integer_are_rejected(self):
        raw_values = [
            json.dumps(metadata())[:-1] + ',"width":1920}',
            json.dumps(metadata()).replace('"width": 1920', '"width": NaN'),
            json.dumps({**metadata(), "extra": 1}),
            json.dumps(metadata(source_frame_seq=True)),
        ]
        for raw in raw_values:
            payload = raw.encode()
            wire = HEADER.pack(MAGIC, VERSION, 0, len(payload), 1) + payload + b"x"
            with self.assertRaises(ProtocolError):
                VideoFrameParser().feed(wire)

    def test_encoder_rejects_bad_dimensions_mode_and_payload(self):
        for bad_metadata in (
                metadata(width=1921), metadata(height=0), metadata(source_mode="mock"),
                metadata(encoding="png")):
            with self.assertRaises(ProtocolError):
                encode_video_frame(bad_metadata, self.jpeg)
        with self.assertRaises(ProtocolError):
            encode_video_frame(metadata(), b"")
        with self.assertRaises(ProtocolError):
            encode_video_frame(metadata(), bytearray(self.jpeg))


if __name__ == "__main__":
    unittest.main()

