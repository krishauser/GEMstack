"use client";

import { useRef, useMemo, useEffect, useState } from "react";
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
  const [mounted, setMounted] = useState(false);

  const ref = useRef<Mesh>(null);
  const { modelPath, scale, rotation, offset, bodyColor } = currentAgent;
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
    if (!ref.current || timeline.length === 0) return;

    const frame = timeline.find((f) => f.time >= time) ?? timeline.at(-1);
    if (!frame) return;

    ref.current.position.set(frame.x, 0, frame.y);
    ref.current.rotation.y = -frame.yaw;
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
