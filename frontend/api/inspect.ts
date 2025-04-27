import {LngLatLike} from "mapbox-gl";

const inspect = async (boundingBox: LngLatLike[]) => {
    await fetch('http://localhost:8000/api/inspect', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            Accept: 'application/json',
        },
        body: JSON.stringify(boundingBox),
    });
};

export {inspect}