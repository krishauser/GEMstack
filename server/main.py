from typing import List, Dict, Optional
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import time
import uuid
import logging

# basic logger
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.StreamHandler()]
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

bounding_box: List["Coordinates"] = []

class Coordinates(BaseModel):
    lat: float
    lon: float


# in-memory storage for the last summon coords
last_summon: Optional[Coordinates] = None


class SummonResponse(BaseModel):
    launch_status: str
    launch_id: str

class Coordinates(BaseModel):
    lat: float
    lon: float

class StreamPosition(BaseModel):
    current_position: Coordinates
    launch_status: str
    eta: str


class InspectResponse(BaseModel):
    coords: List[Coordinates]


# TODO: replace this mock with real implementation
@app.post("/api/summon", response_model=SummonResponse)
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


@app.post("/api/inspect", status_code=201)
def get_bounding_box(coords: List[Coordinates]):
    global bounding_box
    # Check to see if it's a quadrilateral
    if len(coords) != 4:
        return JSONResponse(
            content={"error": "Require 4 values for bounding box coordinates"},
            status_code=400
        )

    bounding_box = coords
    return {"message": "Successfully retrieved bounding box coords!"}


@app.get("/api/inspect", response_model=InspectResponse, status_code=200)
def send_bounding_box():
    return InspectResponse(coords=bounding_box)
