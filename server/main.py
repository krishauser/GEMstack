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

# Single active summon process
active_summon: Optional[Dict] = None
bounding_box: List["Coordinates"] = []


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
def summon(req: Coordinates):
    global active_summon

    if active_summon is not None:
        raise HTTPException(
            status_code=409,
            detail="A summon is already in progress. Please wait until the current process finishes."
        )
    # Generate a new launch ID
    launch_id = str(uuid.uuid4())
    logging.info(f"received summon request to {req.lat} {req.lat}, unique launch id: {launch_id}")

    # Store summon information as the single active summon
    active_summon = {
        "launch_id": launch_id,
        "target": {"lat": req.lat, "lon": req.lon},
        "current_position": {"lat": 40.0930, "lon": -88.2350},  # Mock starting position
        "start_time": time.time(),
        "status": "launched"
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
            current_lat = start_position["lat"] + (target_position["lat"] - start_position["lat"]) * progress
            current_lon = start_position["lon"] + (target_position["lon"] - start_position["lon"]) * progress

            logging.info(f"progress {progress}, lat {current_lat} and lon {current_lon} for launch: {launch_id}")

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
            current_position=Coordinates(lat=target_position["lat"], lon=target_position["lon"]),
            launch_status="arrived",
            eta="0 sec"
        )
        yield f"data: {final_data.json()}\n\n"

        # Reset active summon when completed
        active_summon = None

    return StreamingResponse(event_generator(), media_type="text/event-stream")


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
