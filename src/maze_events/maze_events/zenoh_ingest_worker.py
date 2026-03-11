import json
import logging
import os
from collections import Counter, defaultdict
from contextlib import closing
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, List

import psycopg2
import zenoh


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("maze_zenoh_ingest_worker")


@dataclass
class PgConfig:
    dsn: str


def get_pg_config_from_env() -> PgConfig:
    dsn = os.environ.get(
        "MAZE_EVENTS_PG_DSN",
        "host=localhost port=5432 dbname=maze_events user=maze password=maze",
    )
    return PgConfig(dsn=dsn)


def to_timestamptz(stamp: Dict[str, Any]) -> datetime:
    sec = int(stamp.get("sec", 0))
    nsec = int(stamp.get("nanosec", 0))
    return datetime.fromtimestamp(sec + nsec / 1e9, tz=timezone.utc)


def ingest_event(cur, event: Dict[str, Any]) -> None:
    """Insert detection_events + detections rows in a single transaction.

    Relies on UNIQUE constraints and PRIMARY KEY to ensure idempotency.
    """
    event_id = event["event_id"]
    run_id = event["run_id"]
    robot_id = event["robot_id"]
    sequence = int(event["sequence"])

    img = event.get("image", {})
    odom = event.get("odometry", {})
    tf = event.get("tf", {})

    stamp = to_timestamptz(img.get("stamp", {}))

    # detection_events insert
    cur.execute(
        """
        INSERT INTO detection_events (
            event_id, run_id, robot_id, sequence, stamp,
            image_frame_id, image_sha256, width, height, encoding,
            x, y, yaw, vx, vy, wz,
            tf_ok, t_base_camera,
            raw_event
        ) VALUES (
            %(event_id)s, %(run_id)s, %(robot_id)s, %(sequence)s, %(stamp)s,
            %(image_frame_id)s, %(image_sha256)s, %(width)s, %(height)s, %(encoding)s,
            %(x)s, %(y)s, %(yaw)s, %(vx)s, %(vy)s, %(wz)s,
            %(tf_ok)s, %(t_base_camera)s,
            %(raw_event)s
        )
        ON CONFLICT (event_id) DO NOTHING
        """,
        {
            "event_id": event_id,
            "run_id": run_id,
            "robot_id": robot_id,
            "sequence": sequence,
            "stamp": stamp,
            "image_frame_id": img.get("frame_id"),
            "image_sha256": img.get("sha256"),
            "width": img.get("width"),
            "height": img.get("height"),
            "encoding": img.get("encoding"),
            "x": odom.get("x"),
            "y": odom.get("y"),
            "yaw": odom.get("yaw"),
            "vx": odom.get("vx"),
            "vy": odom.get("vy"),
            "wz": odom.get("wz"),
            "tf_ok": tf.get("tf_ok", False),
            "t_base_camera": tf.get("t_base_camera"),
            "raw_event": json.dumps(event),
        },
    )

    # detections inserts
    for det in event.get("detections", []):
        bbox = det.get("bbox_xyxy", [None, None, None, None])
        cur.execute(
            """
            INSERT INTO detections (
                event_id, det_id, class_id, class_name,
                confidence, x1, y1, x2, y2
            ) VALUES (
                %(event_id)s, %(det_id)s, %(class_id)s, %(class_name)s,
                %(confidence)s, %(x1)s, %(y1)s, %(x2)s, %(y2)s
            )
            ON CONFLICT (event_id, det_id) DO NOTHING
            """,
            {
                "event_id": event_id,
                "det_id": det.get("det_id"),
                "class_id": det.get("class_id"),
                "class_name": det.get("class_name"),
                "confidence": det.get("confidence"),
                "x1": bbox[0],
                "y1": bbox[1],
                "x2": bbox[2],
                "y2": bbox[3],
            },
        )


def run_worker() -> None:
    pg_cfg = get_pg_config_from_env()

    with closing(psycopg2.connect(pg_cfg.dsn)) as conn:
        conn.autocommit = False

        zenoh_cfg = {"connect": ["tcp/127.0.0.1:7447"]}
        logger.info("Connecting to Zenoh...")
        session = zenoh.open(zenoh_cfg)
        logger.info("Connected to Zenoh.")

        key_expr = "maze/**/detections/v1/*"
        logger.info("Subscribing to %s", key_expr)

        per_class_counts: Counter = Counter()
        per_run_counts: defaultdict[str, int] = defaultdict(int)

        def listener(sample: zenoh.Sample) -> None:  # type: ignore[type-arg]
            try:
                payload = sample.payload
                if isinstance(payload, zenoh.Value) and payload.encoding == "application/json":
                    event = payload.get()
                else:
                    event = json.loads(bytes(payload).decode("utf-8"))
            except Exception as exc:  # noqa: BLE001
                logger.error("Failed to parse JSON from Zenoh: %s", exc)
                return

            try:
                with conn.cursor() as cur:
                    ingest_event(cur, event)
                conn.commit()
            except Exception as exc:  # noqa: BLE001
                conn.rollback()
                logger.error("DB ingest failed: %s", exc)
                return

            # Simple in-memory stats for run report
            run_id = event.get("run_id", "unknown")
            per_run_counts[run_id] += 1
            for det in event.get("detections", []):
                cls_name = det.get("class_name", "unknown")
                per_class_counts[cls_name] += 1

        sub = session.declare_subscriber(key_expr, listener)

        logger.info("Zenoh ingest worker running. Press Ctrl+C to stop.")
        try:
            while True:
                session.wait()
        except KeyboardInterrupt:
            logger.info("Shutting down ingest worker...")
        finally:
            sub.undeclare()
            session.close()


def main() -> None:
    run_worker()


if __name__ == "__main__":
    main()

