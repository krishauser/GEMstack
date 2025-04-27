"use client";

import { useRef, useMemo, useEffect, useState } from "react";
import { useFrame } from "@react-three/fiber";
import { Mesh, Object3D, MeshStandardMaterial } from "three";
import { useGLTF } from "@react-three/drei";
import { FrameData } from "@/types/FrameData";
import { currentTrafficLight } from "@/config/trafficLightConfig";

interface TrafficLightProps {
  id: string;
  timeline: FrameData[];
  time: number;
}

export default function TrafficLight({ id, timeline, time }: TrafficLightProps) {
  const [mounted, setMounted] = useState(false);

  const ref = useRef<Mesh>(null);
  const { modelPath, scale, rotation, offset, bodyColor } = currentTrafficLight;
  const { scene } = useGLTF(modelPath);
  const clonedScene = useMemo(() => scene.clone(true), [scene]);

  useEffect(() => {
    setMounted(true);
  }, []);

  useEffect(() => {
    clonedScene.traverse((child) => {
      if (
        child instanceof Mesh &&
        child.material instanceof MeshStandardMaterial
      ) {
        child.material = child.material.clone();
        child.material.color.set(bodyColor);
      }
    });
  }, [clonedScene, bodyColor]);

  useFrame(() => {
    const frame = timeline.find((f) => f.time >= time);
    if (frame && ref.current) {
      ref.current.position.set(frame.x, frame.z, frame.y);
      ref.current.rotation.y = -frame.yaw;
    }
  });

  const hasSpawned = timeline.length > 0 && timeline[0].time <= time;
  if (!mounted || !hasSpawned) return null;

  return (
    <primitive
      ref={ref as React.RefObject<Object3D>}
      object={clonedScene}
      scale={scale}
      rotation={rotation}
      position={offset}
    />
  );
}
