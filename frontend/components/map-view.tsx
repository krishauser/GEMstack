"use client";

import { Button } from "@/components/ui/button";
import {Locate, Plus, Minus, ArrowUpCircle} from "lucide-react";
import mapboxgl, {LngLatLike} from "mapbox-gl";
import React, { useRef, useEffect, useState } from "react";
import "mapbox-gl/dist/mapbox-gl.css";
import '@mapbox/mapbox-gl-draw/dist/mapbox-gl-draw.css';
import MapboxDraw from "@mapbox/mapbox-gl-draw";
import {inspect} from "@/api/inspect";

const INITIAL_CENTER: { lng: number; lat: number } = {
  lng: -88.23556018270287,
  lat: 40.0931189521871,
};
const INITIAL_ZOOM = 18.25;
const INITIAL_PITCH = 20;

export function MapView() {
  const mapRef = useRef<mapboxgl.Map | null>(null);
  const mapContainerRef = useRef<HTMLDivElement | null>(null);

  const [center, setCenter] = useState<{ lng: number; lat: number }>(
    INITIAL_CENTER,
  );
  const [boundingBox, setBoundingBox] = useState<LngLatLike[]>([]);
  const [zoom, setZoom] = useState(INITIAL_ZOOM);
  const [pitch, _] = useState(INITIAL_PITCH);

  const mapboxToken = process.env.NEXT_PUBLIC_MAPBOX_ACCESS_TOKEN;

  const handleButtonClick = () => {
    mapRef.current?.flyTo({
      center: INITIAL_CENTER,
      zoom: INITIAL_ZOOM,
    });
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
    });

    const draw = new MapboxDraw({
      displayControlsDefault: false,
      boxSelect: true,
      controls: {
        polygon: true,
        trash: true,
      },
      defaultMode: 'draw_polygon',
    });

    mapRef.current.addControl(draw);

    mapRef.current.on('draw.create', updateCoordinates);
    mapRef.current.on('draw.delete', updateCoordinates);
    mapRef.current.on('draw.update', updateCoordinates);

    function updateCoordinates() {
      const features = draw.getAll().features;

      if (features.length > 0) {
        if (features[0].geometry.type == 'Polygon') {
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

    mapRef.current.addControl(
      new mapboxgl.GeolocateControl({
        positionOptions: {
          enableHighAccuracy: true,
        },
        trackUserLocation: true,
        showUserHeading: true,
      }),
    );

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
      <div className="absolute top-6 left-6 bg-neutral-900/50 p-2 rounded-sm">
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
          onClick={handleButtonClick}
        >
          <Locate className="h-5 w-5 mr-2" />
          Find My Car
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
