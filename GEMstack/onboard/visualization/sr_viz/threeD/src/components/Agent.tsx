"use client";

import { useRef, useMemo, useEffect } from "react";
import { useFrame } from "@react-three/fiber";
import { Mesh, Object3D, MeshStandardMaterial } from "three";
import { useGLTF } from "@react-three/drei";
import { FrameData } from "@/types/FrameData";
import { currentAgent } from "@/config/agentConfig";

interface AgentProps {
  id: string;
  timeline: FrameData[];
  time: number;
}

export default function Agent({ id, timeline, time }: AgentProps) {
  const ref = useRef<Mesh>(null);

  const { modelPath, scale, rotation, offset, bodyColor } = currentAgent;

  const { scene } = useGLTF(modelPath);
  const clonedScene = useMemo(() => scene.clone(true), [scene]);

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
      ref.current.position.set(frame.x, 0, frame.y);
      ref.current.rotation.y = -frame.yaw;
    }
  });

  const hasSpawned = timeline.length > 0 && timeline[0].time <= time;
  if (!hasSpawned) return null;

  return (
    <primitive
      ref={ref as React.MutableRefObject<Object3D>}
      object={clonedScene}
      scale={scale}
      rotation={rotation}
      position={offset}
    />
  );
}
