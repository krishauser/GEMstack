from fastapi import FastAPI
from pydantic import BaseModel
from typing import List

app = FastAPI()

# Define a Point model
class Point(BaseModel):
    x: float
    y: float

# In-memory store for points
points: List[Point] = []

@app.post("/point")
def add_point(point: Point):
    points.append(point)
    return {"message": "Point added", "point": point}

@app.get("/point")
def get_points():
    return {"points": points}
