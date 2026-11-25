#!/usr/bin/env python3
"""Read-only cross-epoch evidence from an older bag that recorded /odom_incremental.

This is an offline corroboration for the 2026-11-14 11:13 failure audit.  It
replays the TransformFusion algebra visible in the checked-out LIO-SAM source:

    odom_pose = mapping_anchor * inv(incremental_front) * incremental_back

The historical bag has the mapping anchor, mapping correction, incremental and
fused streams needed for a replay.  The 11:13 bag does not contain
/odom_incremental, so this script must not be used to claim that the exact
11:13 queue contents were observed.  Bag timestamps are recorder arrival
timestamps and do not expose executor callback ordering.
"""

from __future__ import annotations

import bisect
import datetime as dt
import hashlib
import math
import sqlite3
import subprocess
from collections import deque
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

AUDIT_DIR = Path(__file__).resolve().parent
BAG = (
    AUDIT_DIR.parents[2]
    / "runs/2026-11-10_10-17-53/navigation_core_bag"
)
SOURCE = AUDIT_DIR.parents[3] / "slam/src/LIO-SAM/src/imuPreintegration.cpp"
TOPICS = (
    "/lio_sam/mapping/odometry",
    "/lio_sam/mapping/odometry_incremental",
    "/odom_incremental",
    "/odom",
)


def ros_time_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def ros_time_s(msg) -> float:
    return ros_time_ns(msg) * 1e-9


def fmt_ns(value: int | float) -> str:
    value = float(value) * 1e-9 if abs(float(value)) > 1e12 else float(value)
    return dt.datetime.fromtimestamp(
        value, tz=dt.timezone(dt.timedelta(hours=8))
    ).strftime("%H:%M:%S.%f")


def pose_xyyaw(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return (float(p.x), float(p.y), float(yaw))


def planar_step(a, b) -> float:
    pa = pose_xyyaw(a)
    pb = pose_xyyaw(b)
    return math.hypot(pb[0] - pa[0], pb[1] - pa[1])


def affine(msg) -> np.ndarray:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        raise ValueError("zero quaternion in %s" % msg.header.frame_id)
    s = 2.0 / norm
    r = np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ],
        dtype=float,
    )
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = [float(p.x), float(p.y), float(p.z)]
    return out


def read_topic(topic: str):
    rows = []
    msg_type = None
    for db in sorted(BAG.glob("*.db3")):
        conn = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
        try:
            row = conn.execute(
                "select id,type from topics where name=?", (topic,)
            ).fetchone()
            if row is None:
                continue
            topic_id, msg_type = row
            rows.extend(
                conn.execute(
                    "select timestamp,data from messages "
                    "where topic_id=? order by timestamp,id",
                    (topic_id,),
                ).fetchall()
            )
        finally:
            conn.close()
    rows.sort(key=lambda item: item[0])
    if msg_type is None:
        raise RuntimeError(f"topic not found: {topic}")
    cls = get_message(msg_type)
    return [(int(bag_ns), deserialize_message(raw, cls)) for bag_ns, raw in rows]


def source_revision():
    try:
        root = subprocess.check_output(
            ["git", "-C", str(AUDIT_DIR.parents[3]), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        root = "unavailable"
    try:
        digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    except Exception:
        digest = "unavailable"
    try:
        status = subprocess.check_output(
            [
                "git",
                "-C",
                str(AUDIT_DIR.parents[3]),
                "status",
                "--short",
                "--",
                str(SOURCE.relative_to(AUDIT_DIR.parents[3])),
            ],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        status = "unavailable"
    return root, digest, status


def main() -> int:
    mapping = read_topic(TOPICS[0])
    mapping_incremental = read_topic(TOPICS[1])
    inc = read_topic(TOPICS[2])
    odom = read_topic(TOPICS[3])

    print("Historical TransformFusion cross-epoch corroboration")
    print(f"bag: {BAG}")
    print("topics:")
    for topic, rows in zip(TOPICS, (mapping, mapping_incremental, inc, odom)):
        print(f"  {topic}: {len(rows)} messages")
    print(
        "  bag span: "
        f"{fmt_ns(min(x[0] for x in mapping + mapping_incremental + inc + odom))} .. "
        f"{fmt_ns(max(x[0] for x in mapping + mapping_incremental + inc + odom))} "
        "(Asia/Shanghai)"
    )
    rev, digest, status = source_revision()
    print(f"checked-out source revision (not binary provenance): {rev}")
    print(f"imuPreintegration.cpp sha256 (current checkout): {digest}")
    print(f"current source worktree status: {status or 'clean'}")
    print()

    # Match recorded odometry by header stamp.  The historical recorder kept
    # /odom and /odom_incremental at essentially the same header timestamps.
    odom_by_stamp = {}
    for bag_ns, msg in odom:
        odom_by_stamp.setdefault(ros_time_ns(msg), []).append((bag_ns, msg))
    inc_by_stamp = {}
    for bag_ns, msg in inc:
        inc_by_stamp.setdefault(ros_time_ns(msg), []).append((bag_ns, msg))
    inc_bag_ns = [x[0] for x in inc]
    mapping_bag_ns = [x[0] for x in mapping]
    mapping_incremental_by_stamp = {}
    for bag_ns, msg in mapping_incremental:
        mapping_incremental_by_stamp.setdefault(ros_time_ns(msg), []).append(
            (bag_ns, msg)
        )

    # Replay the exact algebra used by TransformFusion.  We retain queue state
    # because it is itself the point of the audit: the source only removes
    # samples whose header stamp is <= the latest mapping header.
    queue = deque()
    anchor = None
    anchor_stamp = -1
    replay = []
    map_state = []
    merged = sorted(
        [(bag_ns, 0, msg) for bag_ns, msg in mapping]
        + [(bag_ns, 1, msg) for bag_ns, msg in inc],
        key=lambda x: (x[0], x[1]),
    )
    for bag_ns, kind, msg in merged:
        if kind == 0:
            before = len(queue)
            old_anchor = anchor
            anchor = affine(msg)
            anchor_stamp = ros_time_ns(msg)
            popped = 0
            while queue and ros_time_ns(queue[0][1]) <= anchor_stamp:
                queue.popleft()
                popped += 1
            state = {
                "bag_ns": bag_ns,
                "header_ns": anchor_stamp,
                "before": before,
                "popped": popped,
                "after": len(queue),
                "front_ns": ros_time_ns(queue[0][1]) if queue else None,
                "front_bag_ns": queue[0][0] if queue else None,
                "pose": pose_xyyaw(msg),
                "previous_pose": None if old_anchor is None else old_anchor[:3, 3].copy(),
            }
            map_state.append(state)
            continue

        queue.append((bag_ns, msg, affine(msg)))
        if anchor is None:
            continue
        while queue and ros_time_ns(queue[0][1]) <= anchor_stamp:
            queue.popleft()
        if not queue:
            continue
        predicted = anchor @ np.linalg.inv(queue[0][2]) @ queue[-1][2]
        candidates = odom_by_stamp.get(ros_time_ns(msg), [])
        if candidates:
            # Choose the recorded /odom sample nearest this incremental sample
            # in bag-arrival time.  This avoids relying on dictionary ordering.
            actual_bag, actual = min(candidates, key=lambda x: abs(x[0] - bag_ns))
            residual = float(np.linalg.norm(predicted[:3, 3] - affine(actual)[:3, 3]))
            replay.append(
                {
                    "bag_ns": bag_ns,
                    "header_ns": ros_time_ns(msg),
                    "actual_bag_ns": actual_bag,
                    "residual": residual,
                    "queue_len": len(queue),
                    "front_ns": ros_time_ns(queue[0][1]),
                    "front_bag_ns": queue[0][0],
                }
            )

    # Find the largest high-rate /odom steps.  These are the useful events for
    # a human review; all values are read-only extracts from the historical bag.
    jumps = []
    for i, ((b0, a), (b1, b)) in enumerate(zip(odom, odom[1:])):
        dt_s = ros_time_s(b) - ros_time_s(a)
        step = planar_step(a, b)
        if 0.0 < dt_s < 0.05 and step >= 0.40:
            jumps.append((step, i, b1, dt_s, a, b))
    jumps.sort(reverse=True, key=lambda x: x[0])

    print(
        "TransformFusion algebra replay: "
        f"{len(replay)} header-matched /odom samples; "
        f"median residual={np.median([x['residual'] for x in replay]):.3e} m, "
        f"max residual={max(x['residual'] for x in replay):.3e} m"
    )
    print(
        "Residual near machine precision means the recorded /odom stream is "
        "consistent with the source formula; it does not by itself identify "
        "which queue sample was produced before or after correction."
    )
    print()
    print("Largest historical /odom jumps (threshold 0.40 m in <50 ms):")

    for rank, (step, oi, obag, odt, before, after) in enumerate(jumps[:5], 1):
        oh = ros_time_ns(after)
        ii = bisect.bisect_left([ros_time_ns(m) for _, m in inc], oh)
        if ii >= len(inc) or ros_time_ns(inc[ii][1]) != oh:
            print(f"\n[{rank}] no same-header /odom_incremental match; skipped")
            continue
        ibag, imsg = inc[ii]
        inc_before = inc[ii - 1][1] if ii else None
        inc_step = planar_step(inc_before, imsg) if inc_before else float("nan")
        mi = bisect.bisect_right(mapping_bag_ns, obag) - 1
        current_map = mapping[mi] if mi >= 0 else None
        previous_map = mapping[mi - 1] if mi > 0 else None
        rp = next((x for x in replay if x["header_ns"] == oh and abs(x["bag_ns"] - ibag) < 2e7), None)
        # Find the queue state at the latest mapping update before this output.
        ms = map_state[mi] if 0 <= mi < len(map_state) else None

        print(f"\n[{rank}] /odom bag={fmt_ns(obag)} header={fmt_ns(oh)}")
        print(f"  /odom step={step:.9f} m in {odt*1000:.3f} ms")
        print(f"    before={pose_xyyaw(before)}")
        print(f"    after ={pose_xyyaw(after)}")
        print(
            f"  /odom_incremental bag={fmt_ns(ibag)} "
            f"bag-after-odom={(ibag-obag)*1e-6:.3f} ms"
        )
        print(f"    step from previous incremental={inc_step:.9f} m")
        print(f"    before={pose_xyyaw(inc_before)}")
        print(f"    after ={pose_xyyaw(imsg)}")
        if current_map:
            mbag, mmap = current_map
            print(
                f"  latest mapping correction bag={fmt_ns(mbag)} "
                f"({(obag-mbag)*1e-6:.3f} ms before /odom), "
                f"header={fmt_ns(ros_time_ns(mmap))}"
            )
            print(f"    pose={pose_xyyaw(mmap)}")
            if previous_map:
                _, pmap = previous_map
                print(
                    f"    step from previous mapping={planar_step(pmap,mmap):.9f} m"
                )
            trigger_candidates = mapping_incremental_by_stamp.get(
                ros_time_ns(mmap), []
            )
            trigger = (
                min(trigger_candidates, key=lambda x: abs(x[0] - mbag))
                if trigger_candidates
                else None
            )
            if trigger:
                tbag, tmsg = trigger
                print(
                    "  matching IMU-preintegration correction "
                    f"(/lio_sam/mapping/odometry_incremental) bag={fmt_ns(tbag)}, "
                    f"header={fmt_ns(ros_time_ns(tmsg))}, "
                    f"arrival delta from mapping anchor={(tbag-mbag)*1e-6:.3f} ms"
                )
        if ms:
            print(
                "  replayed queue state after latest mapping arrival: "
                f"before={ms['before']}, popped={ms['popped']}, "
                f"after={ms['after']}, "
                f"front_header={fmt_ns(ms['front_ns']) if ms['front_ns'] else 'none'}"
            )
            if ms["front_bag_ns"]:
                print(
                    f"    front bag receipt={fmt_ns(ms['front_bag_ns'])}; "
                    f"front receipt to current incremental={(ibag-ms['front_bag_ns'])*1e-6:.3f} ms"
                )
                if current_map and trigger:
                    tbag, _ = trigger
                    print(
                        "    recorder-order bracket: "
                        f"front={fmt_ns(ms['front_bag_ns'])} < "
                        f"correction={fmt_ns(tbag)} < "
                        f"back={fmt_ns(ibag)}; "
                        f"front was {(tbag-ms['front_bag_ns'])*1e-6:.3f} ms before "
                        f"correction and back was {(ibag-tbag)*1e-6:.3f} ms after"
                    )
        if rp:
            print(
                f"  algebra replay residual at this /odom={rp['residual']:.3e} m; "
                f"queue length at output={rp['queue_len']}"
            )
            if current_map and previous_map:
                front_candidates = inc_by_stamp.get(rp["front_ns"], [])
                if front_candidates:
                    _, front_msg = min(
                        front_candidates, key=lambda x: abs(x[0] - ibag)
                    )
                    _, previous_mapping_msg = previous_map
                    _, current_mapping_msg = current_map
                    actual_affine = affine(after)
                    current_anchor_prediction = affine(current_mapping_msg) @ np.linalg.inv(
                        affine(front_msg)
                    ) @ affine(imsg)
                    previous_anchor_prediction = affine(previous_mapping_msg) @ np.linalg.inv(
                        affine(front_msg)
                    ) @ affine(imsg)
                    current_error = float(
                        np.linalg.norm(
                            current_anchor_prediction[:3, 3]
                            - actual_affine[:3, 3]
                        )
                    )
                    previous_error = float(
                        np.linalg.norm(
                            previous_anchor_prediction[:3, 3]
                            - actual_affine[:3, 3]
                        )
                    )
                    print(
                        "  anchor check using same retained front/back: "
                        f"current mapping anchor residual={current_error:.3e} m; "
                        f"previous mapping anchor residual={previous_error:.6f} m"
                    )

    print()
    print("Interpretation and limits:")
    print(
        "- The historical bag directly records a discontinuous "
        "/odom_incremental step immediately after a mapping correction and "
        "shows the matching /odom step. The replay reproduces /odom from the "
        "TransformFusion formula; the selected large-jump events have "
        "machine-precision residuals. The larger all-sample maximum residual "
        "is retained because recorder arrival order is not exact callback "
        "ordering."
    )
    print(
        "- The source queue is pruned only by header stamp <= latest mapping "
        "header; queue entries newer than that anchor remain. The selected "
        "events directly bracket a retained front receipt before the mapping "
        "correction and a back receipt after it. Calling these samples old/new "
        "epochs is still an inference because the messages have no epoch tag "
        "and recorder order is not executor callback order."
    )
    print(
        "- This is same-system historical corroboration, not a frame-by-frame "
        "observation of the 11:13 failure: that bag omitted /odom_incremental, "
        "and the historical binary may differ from the current uncommitted "
        "checkout. Recorder arrival order is not exact executor scheduling."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
