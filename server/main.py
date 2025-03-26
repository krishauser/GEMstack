from typing import List
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel, Field
import time
import uuid
# import jwt

SECRET_KEY = "CHANGE_ME_TO_SOMETHING_SECURE"

app = FastAPI(title="GemStack Car‑Summon API (Mock)")

### MODELS ###

class LoginRequest(BaseModel):
    username: str
    password: str

class CoordinatesRequest(BaseModel):
    lat: float = Field(..., ge=-90, le=90)
    lon: float = Field(..., ge=-180, le=180)

class CoordinatesResponse(BaseModel):
    current_position: CoordinatesRequest
    optimized_route: List[CoordinatesRequest]
    eta: str

class SummonResponse(BaseModel):
    launch_status: str
    launch_id: str

class CancelRequest(BaseModel):
    launch_id: str

class CancelResponse(BaseModel):
    launch_id: str
    status: str

class StreamPosition(BaseModel):
    current_position: CoordinatesRequest
    launch_status: str
    eta: str

### HELPERS ###

# def create_jwt(username: str) -> str:
#     payload = {"sub": username, "jti": str(uuid.uuid4())}
#     return jwt.encode(payload, SECRET_KEY, algorithm="HS256")

### ENDPOINTS ###

# @app.post("/api/login")
# def login(req: LoginRequest):
#     if req.username == "admin" and req.password == "password":
#         return {"token": create_jwt(req.username)}
#     raise HTTPException(status_code=401, detail="Invalid credentials")

@app.post("/api/coordinates", response_model=CoordinatesResponse)
def get_coordinates(req: CoordinatesRequest):
    # Mock “optimized route” as a straight line of 3 waypoints
    route = [
        CoordinatesRequest(lat=req.lat + 0.001 * i, lon=req.lon + 0.001 * i)
        for i in range(1, 4)
    ]
    return CoordinatesResponse(
        current_position=CoordinatesRequest(lat=req.lat, lon=req.lon),
        optimized_route=route,
        eta="5 min",
    )

@app.post("/api/summon", response_model=SummonResponse)
def summon(req: CoordinatesRequest):
    launch_id = str(uuid.uuid4())
    return SummonResponse(launch_status="launched", launch_id=launch_id)

@app.get("/api/stream_position/{launch_id}")
def stream_position(launch_id: str):
    def event_generator():
        lat, lon = 40.0930, -88.2350
        for i in range(5):
            time.sleep(1)
            lat += 0.0005
            lon += 0.0005
            yield f"data: {StreamPosition(current_position=CoordinatesRequest(lat=lat, lon=lon), launch_status='navigating', eta=f'{5-i} min').json()}\n\n"
        yield "data: {\"launch_status\":\"arrived\"}\n\n"
    return StreamingResponse(event_generator(), media_type="text/event-stream")

@app.post("/api/cancel", response_model=CancelResponse)
def cancel(req: CancelRequest):
    return CancelResponse(launch_id=req.launch_id, status="cancelled")
