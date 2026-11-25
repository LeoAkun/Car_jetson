# P5/P6 Wire Protocol, Schema 2

This document freezes the CPU-only wire contract. It defines framing and validation only; it does not start listeners or authorize vehicle control. Framing is implemented in `traffic_light_stream/video_protocol.py` and `traffic_light_stream/result_protocol.py`; the separate TLS transport policy is implemented in `traffic_light_stream/tls.py` and documented in `deploy/P5_P6_TLS.md`.

## Video Stream

The sender initiates a TLS 1.3 mutually authenticated connection to the service video endpoint. TLV1 parsing starts only after CA, validity, SAN/EKU and application-role checks complete. Within the protected byte stream, each frame is:

| Offset | Size | Encoding | Value |
|---:|---:|---|---|
| 0 | 4 | bytes | ASCII `TLV1` |
| 4 | 2 | unsigned, network byte order | version `1` |
| 6 | 2 | unsigned, network byte order | flags `0` |
| 8 | 4 | unsigned, network byte order | UTF-8 metadata byte length |
| 12 | 4 | unsigned, network byte order | JPEG byte length |
| 16 | metadata length | UTF-8 JSON object | frame identity and source timestamps |
| following | JPEG length | opaque bytes | one complete JPEG payload |

Metadata is limited to 4,096 bytes. JPEG is limited to 4 MiB. Initial dimensions are at most 1,920 x 1,080. Zero lengths, lengths over the configured limit, unknown header versions/flags, invalid JSON, duplicate keys, unknown metadata keys, and EOF inside a frame close the parser. A receiver validates lengths before buffering either payload. It does not scan for a later magic value after corruption.

The metadata object has exactly these fields:

```json
{
  "source_camera_id": "front-traffic-port",
  "source_host_boot_id": "host-boot-uuid",
  "sender_instance_id": "sender-process-uuid",
  "source_boot_id": "stream-epoch-uuid",
  "source_frame_seq": 9271,
  "source_capture_monotonic_ns": 1234567890123,
  "source_dequeue_monotonic_ns": 1234567899123,
  "timestamp_basis": "v4l2_monotonic_eof",
  "source_mode": "replay",
  "width": 1920,
  "height": 1080,
  "encoding": "jpeg"
}
```

All identity and timestamp-basis strings are printable and 1-128 characters. `source_frame_seq` and both monotonic timestamps are positive unsigned 64-bit integers; booleans are not integers here. Width and height are positive integers within the limits. `source_mode` is `camera` or `replay`; replay identity must not be admitted by a real-camera allowlist. `timestamp_basis` names the verified capture timestamp source, not the application dequeue time. The example value is a fixture, not a claim about the installed camera.

`source_host_boot_id` identifies the host monotonic clock epoch. `sender_instance_id` identifies a process. `source_boot_id` identifies a stream epoch and changes after reconnect or source recovery. `source_frame_seq` strictly increases within an epoch; receiver sampling may skip frame numbers, but must not accept repeats or regressions. The source capture timestamp and sequence must advance consistently. The dequeue timestamp is diagnostic and must not replace source capture time.

## Results and Commands

The result/control connection is a UTF-8 NDJSON stream. One JSON object plus LF is one record. The complete line, including LF, is at most 65,536 bytes. Each incomplete line has one total 500 ms deadline measured from its first byte; receiving additional bytes does not restart it. Parsers accept split UTF-8, split lines, and multiple lines per read. Empty records, invalid UTF-8/JSON, duplicate keys, NaN/Infinity, unknown schema/type/fields, oversized lines, and incomplete EOF close the connection. Schema version is the exact integer `2`.

Common identity strings are printable and at most 128 characters. Integers reject booleans. Numeric values must be finite. ROI coordinates are normalized `[x0,y0,x1,y1]` with `0 <= x0 < x1 <= 1` and `0 <= y0 < y1 <= 1`.

### Frozen Fixtures

These synthetic fixtures exercise the parser. Hashes and IDs are examples, not deployment credentials or model pins.

Hello (both peers identify their application role; the TLS certificate separately authenticates the peer):

```json
{"schema_version":2,"type":"hello","role":"controller","peer_id":"vehicle-controller","peer_boot_id":"host-boot-uuid","capabilities":["schema2","source-age-check"]}
```

Status request:

```json
{"schema_version":2,"type":"status","request_id":"req-status"}
```

Arm request:

```json
{"schema_version":2,"type":"command","action":"arm","request_id":"req-001","session_id":"session-001","task_id":"shadow-test-001","map_name":"K8","waypoint_id":42,"direction":"straight","roi_norm":[0.30,0.05,0.72,0.58],"expected_model_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","expected_detector_boot_id":"detector-boot-uuid","source_camera_id":"front-traffic-port","source_host_boot_id":"host-boot-uuid","source_boot_id":"stream-epoch-uuid","arm_monotonic_ns":1234567890000}
```

Renew and disarm requests:

```json
{"schema_version":2,"type":"command","action":"renew","request_id":"req-renew","session_id":"session-001"}
{"schema_version":2,"type":"command","action":"disarm","request_id":"req-disarm","session_id":"session-001"}
```

ACK:

```json
{"schema_version":2,"type":"ack","action":"arm","request_id":"req-001","accepted":true,"reason":"","service_boot_id":"service-boot-uuid","session_id":"session-001","lease_duration_ms":1500}
```

An ACK echoes the `request_id` and action. A successful arm ACK also echoes the complete accepted binding set in optional `bindings` in the service implementation. `lease_duration_ms` is a duration, not a Jetson monotonic deadline; monotonic clocks are not compared across hosts. Repeating an identical request ID and body returns the cached ACK without repeating the effect or extending the lease. Reusing an ID with a different body is rejected. A rejected arm leaves the service unarmed.

Observation (the candidate model is never control eligible):

```json
{"schema_version":2,"type":"observation","service_boot_id":"service-boot-uuid","seq":1,"session_id":"session-001","task_id":"shadow-test-001","map_name":"K8","waypoint_id":42,"direction":"straight","model_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","roi_norm":[0.30,0.05,0.72,0.58],"source_camera_id":"front-traffic-port","source_host_boot_id":"host-boot-uuid","source_boot_id":"stream-epoch-uuid","source_frame_seq":9271,"source_capture_monotonic_ns":1234567890123,"timestamp_basis":"v4l2_monotonic_eof","source_mode":"replay","receive_monotonic_ns":2234567890123,"state":"UNKNOWN","valid":false,"confidence":0.0,"has_bbox":false,"bbox_norm":[0,0,0,0],"reason":"no_eligible_candidate","inference_ms":19.01,"control_eligible":false,"automatic_release_enabled":false,"thresholds_calibrated":false}
```

`seq` is allocated by the service for every observation, including `UNKNOWN`; it is never renumbered after a result is dropped. `source_frame_seq` may skip if the service consumes only the latest frame, but it must increase. A missing observation sequence resets client continuity. `receive_monotonic_ns` belongs to the service and is diagnostic only. The `.140` verifier compares source capture time only with `.140` monotonic time from the same `source_host_boot_id`.

Heartbeat:

```json
{"schema_version":2,"type":"heartbeat","service_boot_id":"service-boot-uuid","heartbeat_seq":1,"ready":false,"armed":false,"session_id":null,"camera_ok":false,"model_ok":true,"model_validation_status":"candidate","control_eligible":false,"automatic_release_enabled":false,"thresholds_calibrated":false,"source_camera_id":"front-traffic-port","source_host_boot_id":null,"source_boot_id":null,"receive_fps":0.0,"inference_fps":0.0,"send_fps":5.0,"source_queue_drops":0,"result_queue_drops":0,"queue_depth":0,"last_frame_age_ms":null,"reason":"video_disconnected"}
```

Heartbeat has its own sequence and cannot fill an observation gap. It is sent at 5 Hz by the later service implementation. `ready=true` requires both camera and model health; `armed=true` additionally requires a current session. A live heartbeat alone does not prove inference is healthy. While the candidate manifest remains unvalidated, all control flags stay false.

Error fixture:

```json
{"schema_version":2,"type":"error","code":"not_ready","message":"source is unavailable","request_id":"req-001"}
```

### Rejection Behavior

| Input | Parser action |
|---|---|
| Unknown schema version, message type, field, or command action | Close connection |
| Missing required field, wrong JSON type, boolean used as integer | Close connection |
| Duplicate key, invalid UTF-8, NaN/Infinity, invalid ROI or non-finite number | Close connection |
| Line/header/payload exceeds configured limit | Reject before growing the corresponding parser buffer; close connection |
| Partial record reaches its total deadline or EOF | Discard partial record and close connection |
| Valid command denied by runtime state | Later service returns `ack` with `accepted:false`; protocol syntax remains valid |

The CPU parser does not establish TLS identity, session state, source freshness, detector health, or camera authenticity. The TLS helper now establishes peer certificate and application-role identity; binding that identity to verified frames and sessions belongs to the sender/receiver/service stages.

The shared `LatestSlot` is depth one. Every publication must include the generation captured when its work began; omitting the token is an API error, and an old token is rejected. Clearing with generation advance invalidates pending and in-flight work. Closing the slot clears pending data, wakes waiters, and rejects future publications.

## Local Source Status IPC

The vehicle-side sender exposes current source identity to the later verifier over a restricted Unix stream socket. The protocol is implemented in `traffic_light_stream/source_ipc.py`. A client sends exactly `status\n`; the server checks Linux `SO_PEERCRED` against its configured UID allowlist and returns one strict UTF-8 JSON record plus LF, at most 4,096 bytes. The socket mode is `0600` or `0660`; binding refuses to replace symlinks, regular files, or sockets not owned by the running UID.

The status record contains schema/type, camera/host/sender/stream identities, latest sequence/capture time, timestamp basis, source mode, health/reason, and the `.140` monotonic generation time. A healthy record requires a nonzero frame sequence and capture time. An unhealthy record requires a reason. The verifier receives the record on the same host, checks that it is not from the future or over its configured age, rejects wrong camera/host/mode and regressions, and reports sender/stream identity changes. IPC failure or stale/unhealthy status means the source cannot be verified; a cached healthy value is not extended.
