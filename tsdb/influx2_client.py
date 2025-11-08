import os
from typing import Dict, Optional, Any

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from dotenv import load_dotenv
from pathlib import Path

load_dotenv(Path(__file__).resolve().parent.parent / ".env")

client = None
write_api = None

def create_client():
    """Create a global InfluxDB v2 client and write API if not present."""
    global client, write_api
    if client is None:
        client = InfluxDBClient(
            url=os.getenv("INFLUXDB_URL", "http://localhost:8086"),
            token=os.getenv("INFLUXDB_TOKEN"),
            org=os.getenv("INFLUXDB_ORG", "hsm"),
        )
    if write_api is None:
        write_api = client.write_api(write_options=SYNCHRONOUS)

def get_measurements_list():
    if(client is None):
        create_client()
    res = []
    bucket = os.getenv("INFLUXDB_BUCKET", "ros")
    query = f'import "influxdata/influxdb/v1"\n' \
            f'v1.measurements(bucket: "{bucket}")'
    records = client.query_api().query_stream(query)
    lists = [{"name": r.get_value()} for r in records]
    for list_obj in lists:
        res.append(list_obj['name'])
    return res


def write_measurement(
    measurement: str,
    fields: Dict[str, Any],
    tags: Optional[Dict[str, str]] = None,
    timestamp_ns: Optional[int] = None,
) -> None:
    """
    Write a single measurement to InfluxDB in a type-agnostic way.

    - measurement: Influx measurement name (e.g. "vel_raw")
    - fields: dict of field key -> value (ints/floats/strings/bools)
    - tags: optional dict of tag key -> string value
    - timestamp_ns: optional epoch timestamp in nanoseconds; if None, server time is used
    """
    if client is None:
        create_client()

    bucket = os.getenv("INFLUXDB_BUCKET", "ros")

    p = Point(measurement)
    if tags:
        for k, v in tags.items():
            # Tags must be strings
            p = p.tag(k, str(v))
    for k, v in fields.items():
        p = p.field(k, v)

    if timestamp_ns is not None:
        p = p.time(timestamp_ns, WritePrecision.NS)

    write_api.write(bucket=bucket, record=p)
