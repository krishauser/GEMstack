from typing import List, Optional
from fastapi import FastAPI, HTTPException, UploadFile, File, Form
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from enum import Enum
import logging
import boto3
import os
from dotenv import load_dotenv

# basic logger
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)

app = FastAPI(title="GemStack Car‑Summon & Inspect API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class Coordinates(BaseModel):
    lat: float
    lon: float


class InspectResponse(BaseModel):
    coords: List[Coordinates]


class PlannerEnum(str, Enum):
    RRT_STAR = "RRT_STAR"
    HYBRID_A_STAR = "HYBRID_A_STAR"
    PARKING = "PARKING"
    LEAVE_PARKING = "LEAVE_PARKING"
    IDLE = "IDLE"
    SUMMON_DRIVING = "SUMMON_DRIVING"
    PARALLEL_PARKING = "PARALLEL_PARKING"


class StatusUpdate(BaseModel):
    status: PlannerEnum


class StatusResponse(BaseModel):
    status: PlannerEnum


current_status: PlannerEnum = PlannerEnum.IDLE
# in-memory storage for the last summon coords
last_summon: Optional[Coordinates] = None
bounding_box: List[Coordinates] = []


@app.post("/api/summon", response_model=Coordinates)
def summon(coords: Coordinates):
    """
    Accepts a pair of lat/lon and stores them for later retrieval.
    """
    if coords.lat < -90 or coords.lat > 90 or coords.lon < -180 or coords.lon > 180:
        return JSONResponse(
            content={"error": "Invalid coordinates; lat ∈ [-90,90], lon ∈ [-180,180]"},
            status_code=400,
        )

    global last_summon
    last_summon = coords
    return coords


@app.get("/api/summon")
def get_summon():
    """
    Returns the last stored summon coordinates,
    or 404 if none have been posted yet.
    """
    if last_summon is None:
        raise HTTPException(status_code=404, detail="No summon coordinates set")
    return last_summon


@app.post("/api/status", response_model=StatusResponse)
def update_status(payload: StatusUpdate):
    """
    Set the global planner status. Returns the new status.
    """
    if payload.status not in PlannerEnum._value2member_map_:
        return JSONResponse(
            content={
                "error": (
                    f"Invalid status '{payload.status}'. "
                    f"Must be one of: {[e.value for e in PlannerEnum]}"
                )
            },
            status_code=400,
        )
    global current_status
    current_status = payload.status
    return StatusResponse(status=current_status)


@app.get("/api/status", response_model=StatusResponse)
def get_status():
    """
    Get the current global planner status.
    """
    return StatusResponse(status=current_status)


@app.post("/api/inspect")
def get_bounding_box(coords: List[Coordinates]):
    """Takes in 4 pairs of lat/lon coordinates and saves the corners for later retrieval"""
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
    """Sends the saved corner of bounding box and resets it for next inspection"""
    global bounding_box
    temp = bounding_box.copy()
    bounding_box = []
    return InspectResponse(coords=temp)


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


@app.post("/api/upload")
def uploadToS3(
    files: list[UploadFile] = File(...),
    bucket: str = Form(...),
    s3_prefix: str = Form(...),
):
    """
    Uploads files to specified s3 bucket.
    Files are stored under the key: s3_prefix/<upload-file-name>.
    """
    s3_client = get_s3_client()
    check_s3_connection(s3_client, bucket)

    files_uploaded = 0

    for file in files:
        s3_key = os.path.join(s3_prefix, file.filename)
        try:
            print(f"Uploading {file.filename} to s3://{bucket}/{s3_key}")
            s3_client.upload_fileobj(file.file, bucket, s3_key)
            files_uploaded += 1
        except Exception as e:
            return JSONResponse(
                content=f"Error uploading {file.filename}: {e}",
                status_code=400,
            )
        finally:
            file.file.close()

    if files_uploaded == 0:
        return JSONResponse(
            content=f"Error: No files were uploaded",
            status_code=400,
        )
    return JSONResponse(content="Files uploaded successfully", status_code=200)
