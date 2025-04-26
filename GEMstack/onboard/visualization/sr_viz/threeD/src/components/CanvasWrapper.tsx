"use client";

import { Canvas } from "@react-three/fiber";
import React, { useEffect } from "react";
import { Environment } from "@react-three/drei";
import { useTimelineStore } from "@/hooks/useTimelineStore";
import Vehicle from "./Vehicle";
import Agent from "./Agent";
import Ground from "./Ground";

export default function CanvasWrapper({
    time,
    setDuration,
}: {
    time: number;
    setDuration: (duration: number) => void;
}) {
    const { vehicle, agents } = useTimelineStore();
    useEffect(() => {
        if (vehicle.length > 0) {
            setDuration(vehicle[vehicle.length - 1].time - vehicle[0].time);
        }
    }, [vehicle]);
    const startTime = vehicle.length > 0 ? vehicle[0].time : 0;
    const syncedTime = startTime + time;

    return (
        <Canvas
            shadows
            camera={{ position: [0, 5, 15], fov: 55, near: 0.1, far: 100000 }}
            style={{
                background: "#fdfdfd",
                position: "fixed",
                inset: 0,
                zIndex: 0,
            }}
        >
            <Environment preset="city" />
            <ambientLight intensity={0.3} />
            <directionalLight
                position={[10, 10, 5]}
                intensity={1.5}
                castShadow
            />
            <Vehicle timeline={vehicle} time={syncedTime} />
            {Object.entries(agents).map(([id, timeline]) => (
                <Agent key={id} id={id} timeline={timeline} time={syncedTime} />
            ))}
            <Ground />
        </Canvas>
    );
}
