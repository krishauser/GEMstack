"use client";

import { Button } from "@/components/ui/button";
import { Locate, ArrowUpCircle, SatelliteIcon, MapIcon } from "lucide-react";
import mapboxgl, { LngLatLike } from "mapbox-gl";
import React, { useRef, useEffect, useState } from "react";
import "mapbox-gl/dist/mapbox-gl.css";
import "@mapbox/mapbox-gl-draw/dist/mapbox-gl-draw.css";
import MapboxDraw from "@mapbox/mapbox-gl-draw";
import { inspect } from "@/api/inspect";
import { toast } from "sonner";
import { summon } from "@/api/summon";
import { SummonDialog } from "@/components/summon-dialog";

// [lng, lat] tuples
const DIAGRAM_POINTS: Record<string, [number, number]> = {
  leftTop: [-88.236129, 40.092819],
  leftLeft: [-88.236168, 40.09278],
  leftBottom: [-88.236129, 40.092741],
  rightTop: [-88.235527, 40.092819],
  rightRight: [-88.235488, 40.09278],
  rightBottom: [-88.235527, 40.092741],
};

// Four lines: top–to–top, bottom–to–bottom, and two verticals
const DIAGRAM_LINES: Array<[number, number][]> = [
  [DIAGRAM_POINTS.leftTop, DIAGRAM_POINTS.rightTop], // top
  [DIAGRAM_POINTS.leftBottom, DIAGRAM_POINTS.rightBottom], // bottom
  [DIAGRAM_POINTS.leftTop, DIAGRAM_POINTS.leftBottom], // left vertical
  [DIAGRAM_POINTS.rightTop, DIAGRAM_POINTS.rightBottom], // right vertical
];

const INITIAL_CENTER: { lng: number; lat: number } = {
  lng: -88.23556018270287,
  lat: 40.0931189521871,
};
const INITIAL_ZOOM = 18.25;
const INITIAL_PITCH = 20;

// ——— Haversine: compute meters between two [lng,lat]
function haversineDistance(
  [lng1, lat1]: [number, number],
  [lng2, lat2]: [number, number],
): number {
  const toRad = (d: number) => (d * Math.PI) / 180;
  const R = 6371000;
  const dLat = toRad(lat2 - lat1);
  const dLng = toRad(lng2 - lng1);
  const a =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLng / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(a));
}

// ——— Build an ellipse polygon given separate horizontal & vertical radii
function createEllipseCoordinates(
  center: [number, number],
  radiusX: number,
  radiusY: number,
  segments = 64,
): [number, number][] {
  const coords: [number, number][] = [];
  const [lng0, lat0] = center;
  const R = 6371000;

  for (let i = 0; i <= segments; i++) {
    const θ = (i / segments) * 2 * Math.PI;
    const dx = radiusX * Math.cos(θ);
    const dy = radiusY * Math.sin(θ);

    // meter offsets → degrees
    const dLat = (dy / R) * (180 / Math.PI);
    const dLng =
      ((dx / R) * (180 / Math.PI)) / Math.cos((lat0 * Math.PI) / 180);

    coords.push([lng0 + dLng, lat0 + dLat]);
  }
  return coords;
}

// ——— Put all your diagram sources & layers (points, labels, lines, ellipses)
function addDiagramLayers(map: mapboxgl.Map) {
  // 1) points
  map.addSource("diagram-points", {
    type: "geojson",
    data: {
      type: "FeatureCollection",
      features: Object.entries(DIAGRAM_POINTS).map(([id, [lng, lat]]) => ({
        type: "Feature",
        properties: {
          id,
          label: `${lng.toFixed(6)}, ${lat.toFixed(6)}`,
        },
        geometry: { type: "Point", coordinates: [lng, lat] },
      })),
    },
  });
  map.addLayer({
    id: "diagram-points-layer",
    type: "circle",
    source: "diagram-points",
    paint: {
      "circle-radius": 6,
      "circle-color": "#e74c3c",
    },
  });
  // 1b) labels
  map.addLayer({
    id: "diagram-point-labels",
    type: "symbol",
    source: "diagram-points",
    layout: {
      "text-field": ["get", "label"],
      "text-font": ["Open Sans Semibold", "Arial Unicode MS Bold"],
      "text-size": 12,
      "text-offset": [1.2, 0],
      "text-anchor": "left",
    },
    paint: { "text-color": "#000000" },
  });

  // 2) lines
  map.addSource("diagram-lines", {
    type: "geojson",
    data: {
      type: "FeatureCollection",
      features: DIAGRAM_LINES.map((coords, i) => ({
        type: "Feature",
        properties: { lineId: i },
        geometry: { type: "LineString", coordinates: coords },
      })),
    },
  });
  map.addLayer({
    id: "diagram-lines-layer",
    type: "line",
    source: "diagram-lines",
    paint: {
      "line-color": "#e74c3c",
      "line-width": 2,
    },
  });

  // 3) ellipses
  // centers = midpoint of top/bottom verticals
  const leftCenter: [number, number] = [
    DIAGRAM_POINTS.leftTop[0],
    (DIAGRAM_POINTS.leftTop[1] + DIAGRAM_POINTS.leftBottom[1]) / 2,
  ];
  const rightCenter: [number, number] = [
    DIAGRAM_POINTS.rightTop[0],
    (DIAGRAM_POINTS.rightTop[1] + DIAGRAM_POINTS.rightBottom[1]) / 2,
  ];

  // horizontal radius = dist(center, farLeft/farRight)
  const radiusXLeft = haversineDistance(leftCenter, DIAGRAM_POINTS.leftLeft);
  const radiusXRight = haversineDistance(
    rightCenter,
    DIAGRAM_POINTS.rightRight,
  );

  // vertical radius = dist(center, top)
  const radiusYLeft = haversineDistance(leftCenter, DIAGRAM_POINTS.leftTop);
  const radiusYRight = haversineDistance(rightCenter, DIAGRAM_POINTS.rightTop);

  const leftEllipse = createEllipseCoordinates(
    leftCenter,
    radiusXLeft,
    radiusYLeft,
  );
  const rightEllipse = createEllipseCoordinates(
    rightCenter,
    radiusXRight,
    radiusYRight,
  );

  map.addSource("diagram-ellipses", {
    type: "geojson",
    data: {
      type: "FeatureCollection",
      features: [
        {
          type: "Feature",
          properties: { id: "left-ellipse" },
          geometry: {
            type: "Polygon",
            coordinates: [leftEllipse],
          },
        },
        {
          type: "Feature",
          properties: { id: "right-ellipse" },
          geometry: {
            type: "Polygon",
            coordinates: [rightEllipse],
          },
        },
      ],
    },
  });
  map.addLayer({
    id: "diagram-ellipses-layer",
    type: "fill",
    source: "diagram-ellipses",
    paint: {
      "fill-color": "rgba(231,76,60,0.1)",
      "fill-outline-color": "#e74c3c",
    },
  });
}

export function MapView() {
  const mapRef = useRef<mapboxgl.Map | null>(null);
  const mapContainerRef = useRef<HTMLDivElement | null>(null);

  const [userLocation, setUserLocation] = useState<{
    lng: number;
    lat: number;
  } | null>(null);
  const [center, setCenter] = useState<{ lng: number; lat: number }>(
    INITIAL_CENTER,
  );
  const [boundingBox, setBoundingBox] = useState<LngLatLike[]>([]);
  const [zoom, setZoom] = useState(INITIAL_ZOOM);
  const [pitch] = useState(INITIAL_PITCH);
  const [satelliteMode, setSatelliteMode] = useState(false);

  const streetStyle = "mapbox://styles/mapbox/standard";
  const satelliteStyle = "mapbox://styles/mapbox/satellite-v9";
  const mapboxToken = process.env.NEXT_PUBLIC_MAPBOX_ACCESS_TOKEN;

  const handleSummon = async () => {
    toast.info(
      `Summon to ${userLocation?.lat} ${userLocation?.lng} was placed into queue.`,
      {
        description:
          "Summoning will begin shortly (as soon as GEM will pick up the event)",
      },
    );
    const coords = userLocation ? userLocation : center;

    const summonReq = await summon(coords.lng, coords.lat);

    if (!summonReq.ok) {
      const errorData = await summonReq.json();
      console.error("Summon error:", errorData.detail || errorData);
      return;
    }
  };

  const toggleStyle = () => {
    if (!mapRef.current) return;
    const newMode = !satelliteMode;
    setSatelliteMode(newMode);
    mapRef.current.setStyle(newMode ? satelliteStyle : streetStyle);
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
        [-88.2346, 40.0935], // Northeast coordinates
      ],
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
            res.push({ lon: position[0], lat: position[1] });
          }
          res.pop();
          setBoundingBox(res);
        }
      } else {
        setBoundingBox([]);
      }
    }

    const marker = new mapboxgl.Marker({
      draggable: true,
    })
      .setLngLat(INITIAL_CENTER)
      .addTo(mapRef.current);

    function onDragEnd() {
      const lngLat = marker.getLngLat();
      setUserLocation({ lat: lngLat.lat, lng: lngLat.lng });
    }

    marker.on("dragend", onDragEnd);

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

    mapRef.current.on("style.load", () => {
      if (mapRef?.current) addDiagramLayers(mapRef.current);
    });

    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
      }
    };
  }, []);

  return (
    <div className="relative w-full h-full bg-neutral-800 flex items-center justify-center">
      <SummonDialog />
      <div id="map-container" ref={mapContainerRef} />
      <div className="absolute top-6 left-6 bg-neutral-900/50 p-2 rounded-sm md:text-base text-xs md:max-w-none max-w-[200px] space-y-1">
        {/* always show map center */}
        <div>
          Center: {center.lng.toFixed(4)}, {center.lat.toFixed(4)}
        </div>

        {/* if user has dragged the marker, show its coords */}
        {userLocation && (
          <div>
            Marker: {userLocation.lng.toFixed(4)}, {userLocation.lat.toFixed(4)}
          </div>
        )}
      </div>

      {/* Map controls */}
      <div className="absolute bottom-6 flex sm:flex-row flex-col-reverse gap-4 justify-between w-full px-6 sm:items-end">
        <div className="flex flex-col sm:flex-row gap-2">
          <Button
            variant="secondary"
            className="rounded-
                full bg-neutral-900 hover:bg-neutral-800"
            onClick={handleSummon}
            disabled={!userLocation}
          >
            <Locate className="h-5 w-5 mr-2" />
            Summon My Car
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

        <div className="flex sm:flex-col gap-2">
          <Button
            variant="secondary"
            size="icon"
            className="rounded-full bg-neutral-900 hover:bg-neutral-800"
            onClick={toggleStyle}
          >
            {satelliteMode ? (
              <MapIcon className="h-5 w-5" />
            ) : (
              <SatelliteIcon className="h-5 w-5" />
            )}
          </Button>
        </div>
      </div>
    </div>
  );
}
