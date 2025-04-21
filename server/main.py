from typing import List, Dict, Optional
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import time
import uuid
import logging
from dotenv import load_dotenv
import boto3
import os

# basic logger
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)

app = FastAPI(title="GemStack Car‑Summon API (Mock)")

app.add_middleware(
    CORSMiddleware,
    # TODO: this should check current IP address and launch on it
    allow_origins=["http://localhost", "http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Single active summon process
active_summon: Optional[Dict] = None
bounding_box: List["Coordinates"] = []


class SummonResponse(BaseModel):
    launch_status: str
    launch_id: str


class Coordinates(BaseModel):
    lat: float
    lng: float


class StreamPosition(BaseModel):
    current_position: Coordinates
    launch_status: str
    eta: str


class InspectResponse(BaseModel):
    coords: List[Coordinates]


# TODO: replace this mock with real implementation
@app.post("/api/summon", response_model=SummonResponse)
def summon(req: Coordinates):
    global active_summon

    if active_summon is not None:
        raise HTTPException(
            status_code=409,
            detail="A summon is already in progress. Please wait until the current process finishes.",
        )
    # Generate a new launch ID
    launch_id = str(uuid.uuid4())
    logging.info(
        f"received summon request to {req.lat} {req.lat}, unique launch id: {launch_id}"
    )

    # Store summon information as the single active summon
    active_summon = {
        "launch_id": launch_id,
        "target": {"lat": req.lat, "lon": req.lng},
        "current_position": {"lat": 40.0930, "lng": -88.2350},  # Mock starting position
        "start_time": time.time(),
        "status": "launched",
    }

    return SummonResponse(launch_status="launched", launch_id=launch_id)


# TODO: replace this mock with real implementation
@app.get("/api/summon")
def get_summon_status():
    global active_summon

    if active_summon is None:
        raise HTTPException(status_code=404, detail="No active summon process")

    def event_generator():
        global active_summon
        start_position = active_summon["current_position"]
        target_position = active_summon["target"]
        launch_id = active_summon["launch_id"]

        # Generate 10 position updates over 10 seconds
        for i in range(10):
            time.sleep(1)

            # Calculate progress (0 to 1)
            progress = min((i + 1) / 10, 1.0)

            # Linear interpolation between start and target
            current_lat = (
                start_position["lat"]
                + (target_position["lat"] - start_position["lat"]) * progress
            )
            current_lon = (
                start_position["lng"]
                + (target_position["lng"] - start_position["lng"]) * progress
            )

            logging.info(
                f"progress {progress}, lat {current_lat} and lon {current_lon} for launch: {launch_id}"
            )

            # Update the active summon state
            active_summon["current_position"] = {"lat": current_lat, "lon": current_lon}
            status = "arrived" if progress >= 1.0 else "navigating"
            active_summon["status"] = status
            eta = f"{10 - (i + 1)} sec" if status != "arrived" else "0 sec"

            stream_data = StreamPosition(
                current_position=Coordinates(lat=current_lat, lon=current_lon),
                launch_status=status,
                eta=eta,
            )
            yield f"data: {stream_data.json()}\n\n"

        # Ensure final state is "arrived"
        active_summon["status"] = "arrived"
        active_summon["current_position"] = target_position
        logging.info(f"car arrived for launch: {launch_id}")
        final_data = StreamPosition(
            current_position=Coordinates(
                lat=target_position["lat"], lon=target_position["lon"]
            ),
            launch_status="arrived",
            eta="0 sec",
        )
        yield f"data: {final_data.json()}\n\n"

        # Reset active summon when completed
        active_summon = None

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/inspect")
def get_bounding_box(coords: List[Coordinates]):
    global bounding_box
    # Check to see if there are 4 coordinates
    if len(coords) != 4:
        return JSONResponse(
            content="Error: Require 4 coordinates values for bounding box",
            status_code=400,
        )

    bounding_box = [coords[0], coords[2]]
    return JSONResponse(
        content="Successfully retrieved bounding box coords!",
        status_code=201,
    )


@app.get("/api/inspect", response_model=InspectResponse, status_code=200)
def send_bounding_box():
    return InspectResponse(coords=bounding_box)


def get_s3_client():
    """
    Initializes the S3 client using AWS credentials from environment variables.
    Expects:
      - AWS_ACCESS_KEY_ID
      - AWS_SECRET_ACCESS_KEY
      - AWS_DEFAULT_REGION
    Exits if any of these are missing.
    """
    # load environment variables from .env file (override local config if exists)
    load_dotenv(override=True)

    access_key = os.environ.get("AWS_ACCESS_KEY_ID")
    secret_key = os.environ.get("AWS_SECRET_ACCESS_KEY")
    region = os.environ.get("AWS_DEFAULT_REGION")

    if not access_key or not secret_key or not region:
        return JSONResponse(
            status_code=404,
            content="Error: AWS credentials not set. Please set AWS_ACCESS_KEY_ID, \
            AWS_SECRET_ACCESS_KEY, and AWS_DEFAULT_REGION environment variables (in .env).",
        )

    return boto3.client(
        "s3",
        aws_access_key_id=access_key,
        aws_secret_access_key=secret_key,
        region_name=region,
    )


def check_s3_connection(s3_client, bucket):
    """
    Verifies that we can connect to S3 and access the specified bucket.
    """
    try:
        s3_client.head_bucket(Bucket=bucket)
        print(f"Connection check: Successfully accessed bucket '{bucket}'")
    except Exception as e:
        return JSONResponse(
            status_code=400,
            content=f"Error: Could not connect to S3 bucket '{bucket}': {e}",
        )


# Endpoint to upload reconstructed files to s3
@app.post("/api/reconstruction")
def uploadToS3(folder_path, bucket, s3_prefix):
    """
    Walks through the folder and uploads each file to S3 under the given prefix.
    Files are stored under the key: s3_prefix/folder_name/<relative_file_path>.
    """
    s3_client = get_s3_client()
    check_s3_connection(s3_client, bucket)

    folder_name = os.path.basename(folder_path)
    files_uploaded = 0

    for root, _, files in os.walk(folder_path):
        for file in files:
            local_path = os.path.join(root, file)
            # determine the file's path relative to the folder being pushed.
            relative_path = os.path.relpath(local_path, folder_path)
            s3_key = os.path.join(s3_prefix, folder_name, relative_path)
            try:
                print(f"Uploading {local_path} to s3://{bucket}/{s3_key}")
                s3_client.upload_file(local_path, bucket, s3_key)
                files_uploaded += 1
            except Exception as e:
                return JSONResponse(
                    content=f"Error uploading {local_path}: {e}",
                    status_code=400,
                )

    if files_uploaded == 0:
        return JSONResponse(
            content=f"Error: No files were uploaded from folder: {folder_path}",
            status_code=400,
        )
    return JSONResponse(content="Files uploaded successfully", status_code=200)
