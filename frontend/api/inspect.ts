import { LngLatLike } from "mapbox-gl";

const inspect = async (boundingBox: LngLatLike[]) => {
  await fetch(`${process.env.NEXT_PUBLIC_API_BASE_URL}/api/inspect`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      Accept: "application/json",
    },
    body: JSON.stringify(boundingBox),
  });
};

export { inspect };
