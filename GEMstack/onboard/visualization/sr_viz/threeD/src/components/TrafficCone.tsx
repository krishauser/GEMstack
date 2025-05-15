"use client";

import React, { useRef, useMemo, useEffect, useState } from "react";
import { useFrame } from "@react-three/fiber";
import {
  Mesh,
  Object3D,
  MeshStandardMaterial,
  Vector3,
  Quaternion,
  Euler,
} from "three";
import { useGLTF } from "@react-three/drei";
import { FrameData } from "@/types/FrameData";
import { currentTrafficCone } from "@/config/trafficConeConfig";
import { getInterpolatedFrame } from "@/utils/getInterpolatedFrame";

interface TrafficConeProps {
  id: string;
  timeline: FrameData[];
  time: number;
}

export default function TrafficCone({ id, timeline, time }: TrafficConeProps) {
  const [mounted, setMounted] = useState(false);
  const ref = useRef<Mesh>(null);
  const { modelPath, scale, rotation, offset, bodyColor } = currentTrafficCone;
  const { scene } = useGLTF(modelPath);
  const clonedScene = useMemo(() => scene.clone(true), [scene]);

  const targetPosition = useMemo(() => new Vector3(), []);
  const targetQuaternion = useMemo(() => new Quaternion(), []);

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
    const frame = getInterpolatedFrame(timeline, time);
    if (!frame) return;
    targetPosition.set(frame.x, frame.z, frame.y);
    ref.current.position.lerp(targetPosition, 0.2);
    targetQuaternion.setFromEuler(new Euler(0, -frame.yaw, 0));
    ref.current.quaternion.slerp(targetQuaternion, 0.2);
  });

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
