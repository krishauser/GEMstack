"use client";

import { Button } from "@/components/ui/button";
import { Locate, Plus, Minus, ArrowUpCircle } from "lucide-react";
import mapboxgl, { LngLatLike } from "mapbox-gl";
import React, { useRef, useEffect, useState } from "react";
import "mapbox-gl/dist/mapbox-gl.css";
import "@mapbox/mapbox-gl-draw/dist/mapbox-gl-draw.css";
import MapboxDraw from "@mapbox/mapbox-gl-draw";
import { inspect } from "@/api/inspect";
import { toast } from "sonner";
import { summon } from "@/api/summon";

const INITIAL_CENTER: { lng: number; lat: number } = {
  lng: -88.23556018270287,
  lat: 40.0931189521871,
};
const INITIAL_ZOOM = 18.25;
const INITIAL_PITCH = 20;

export function MapView() {
  const mapRef = useRef<mapboxgl.Map | null>(null);
  const mapContainerRef = useRef<HTMLDivElement | null>(null);

  const [userLocation, setUserLocation] = useState<{
    lng: number;
    lat: number;
  } | null>(null);
  const [center, setCenter] = useState<{ lng: number; lat: number }>(
    INITIAL_CENTER
  );
  const [boundingBox, setBoundingBox] = useState<LngLatLike[]>([]);
  const [zoom, setZoom] = useState(INITIAL_ZOOM);
  const [pitch, _] = useState(INITIAL_PITCH);

  const [isSummoning, setIsSummoning] = useState(false);

  const mapboxToken = process.env.NEXT_PUBLIC_MAPBOX_ACCESS_TOKEN;

  const handleSummon = async () => {
    toast.info(`Summoning to ${userLocation?.lat} ${userLocation?.lng}`)
    setIsSummoning(true);
    const coords = userLocation ? userLocation : center;

      const summonReq = await summon(coords.lng, coords.lat);

      if (!summonReq.ok) {
        const errorData = await summonReq.json();
        console.error("Summon error:", errorData.detail || errorData);
        setIsSummoning(false);
        return;
      }
  };

  const handleZoomIn = () => {
    mapRef.current?.zoomIn();
  };
  const handleZoomOut = () => {
    mapRef.current?.zoomOut();
  };

  useEffect(() => {
    if (mapboxToken) {
      mapboxgl.accessToken = mapboxToken;
    } else {
      console.error("Mapbox token is not defined");
    }
    mapRef.current = new mapboxgl.Map({
      container: mapContainerRef.current as HTMLElement,
      center: center,
      zoom: zoom,
      pitch: pitch,
      // style: "mapbox://styles/mapbox/satellite-v9",
      maxBounds: [
        [-88.2368, 40.0925], // Southwest coordinates
        [-88.2346, 40.0935] // Northeast coordinates
      ]
    });

    const draw = new MapboxDraw({
      displayControlsDefault: false,
      boxSelect: true,
      controls: {
        polygon: true,
        trash: true,
      },
      defaultMode: "simple_select",
    });

    mapRef.current.on("load", async () => {
      mapRef.current?.addSource("iss", {
        type: "geojson",
        data: {
          type: "FeatureCollection",
          features: [],
        },
      });

      mapRef.current?.addLayer({
        id: "iss",
        type: "symbol",
        source: "iss",
        layout: {
          "icon-image": "car",
          "icon-size": 2,
        },
      });
    });

    mapRef.current.addControl(draw);

    mapRef.current.on("draw.create", updateCoordinates);
    mapRef.current.on("draw.delete", updateCoordinates);
    mapRef.current.on("draw.update", updateCoordinates);

    function updateCoordinates() {
      const features = draw.getAll().features;

      if (features.length > 0) {
        if (features[0].geometry.type == "Polygon") {
          const res = [] as LngLatLike[];
          for (const position of features[0].geometry.coordinates[0]) {
            res.push({ lng: position[0], lat: position[1] });
          }
          res.pop();
          setBoundingBox(res);
        }
      } else {
        setBoundingBox([]);
      }
    }

    const marker = new mapboxgl.Marker({
      draggable: true
    })
        .setLngLat(INITIAL_CENTER)
        .addTo(mapRef.current);

    function onDragEnd() {
      const lngLat = marker.getLngLat();
      setUserLocation({lat: lngLat.lat, lng: lngLat.lng})
    }

    marker.on('dragend', onDragEnd);

    mapRef.current.on("move", () => {
      // get the current center coordinates and zoom level from the map
      const mapCenter = mapRef.current?.getCenter();
      const mapZoom = mapRef.current?.getZoom();

      // update state
      if (mapCenter) {
        setCenter({ lng: mapCenter.lng, lat: mapCenter.lat });
      }
      if (mapZoom) {
        setZoom(mapZoom);
      }
    });

    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
      }
    };
  }, []);

  return (
    <div className="relative w-full h-full bg-neutral-800 flex items-center justify-center">
      <div id="map-container" ref={mapContainerRef} />
      <div className="absolute top-6 left-6 bg-neutral-900/50 p-2 rounded-sm md:text-base text-xs md:max-w-none max-w-[200px]">
        Longitude: {center.lng.toFixed(4)} | Latitude: {center.lat.toFixed(4)} |
        Zoom: {zoom.toFixed(2)}
      </div>

      {/* Map controls */}
      <div className="absolute bottom-6 right-6 flex flex-col gap-2">
        <Button
          variant="secondary"
          size="icon"
          className="rounded-full bg-neutral-900 hover:bg-neutral-800"
          onClick={handleZoomIn}
        >
          <Plus
            className="h-5 w-5"
            onClick={() => setZoom(Math.min(zoom + 1, 20))}
          />
        </Button>
        <Button
          variant="secondary"
          size="icon"
          className="rounded-full bg-neutral-900 hover:bg-neutral-800"
          onClick={handleZoomOut}
        >
          <Minus
            className="h-5 w-5"
            onClick={() => setZoom(Math.max(zoom - 1, 10))}
          />
        </Button>
      </div>

      <div className="absolute bottom-6 left-6 flex gap-2">
        <Button
          variant="secondary"
          className="rounded-
                full bg-neutral-900 hover:bg-neutral-800"
          onClick={handleSummon}
          disabled={isSummoning || !userLocation}
        >
          <Locate className="h-5 w-5 mr-2" />
          {isSummoning ? "Summoning..." : "Summon My Car"}
        </Button>
        <Button
          variant="secondary"
          className="rounded-
                full bg-neutral-900 hover:bg-neutral-800"
          onClick={() => inspect(boundingBox)}
        >
          <ArrowUpCircle className="h-5 w-5 mr-2" />
          <span>Inspect Region</span>
        </Button>
      </div>
    </div>
  );
}
