import os
from influxdb_client import InfluxDBClient, Point, WritePrecision
from dotenv import load_dotenv
from pathlib import Path

load_dotenv(Path(__file__).resolve().parent.parent / ".env")

client = None

def create_client():
    global client
    client = InfluxDBClient(
        url=os.getenv("INFLUXDB_URL", "http://localhost:8086"),
        token=os.getenv("INFLUXDB_TOKEN"),
        org=os.getenv("INFLUXDB_ORG", "hsm")
    )

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
