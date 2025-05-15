import React, { useRef, useMemo, useEffect } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { useCameraController } from "@/hooks/useCameraController";
import { currentVehicle } from "@/config/vehicleConfig";
import { FrameData } from "@/types/FrameData";
import { OrbitControls } from "@react-three/drei";
import URDFLoader from "urdf-loader";
import { getInterpolatedFrame } from "@/utils/getInterpolatedFrame";

interface VehicleProps {
  timeline: FrameData[];
  time: number;
}

export default function Vehicle({ timeline, time }: VehicleProps) {
  const ref = useRef<THREE.Group>(null);
  const vehicleGroup = useRef<THREE.Group>(new THREE.Group());
  const mode = useCameraController(ref);

  const targetPosition = useMemo(() => new THREE.Vector3(), []);
  const targetQuaternion = useMemo(() => new THREE.Quaternion(), []);

  const { modelPath, scale, rotation, offset, bodyColor } = currentVehicle;

  useEffect(() => {
    const loader = new URDFLoader();

    loader.load(
      modelPath,
      (robot) => {
        robot.scale.set(scale[0], scale[1], scale[2]);
        robot.rotation.set(rotation[0], rotation[1], rotation[2]);
        robot.position.set(offset[0], offset[1], offset[2]);

        robot.traverse((child) => {
          if (
            child instanceof THREE.Mesh &&
            child.material instanceof THREE.MeshStandardMaterial
          ) {
            child.material = child.material.clone();
            child.material.color.set(bodyColor);
          }
        });

        if (vehicleGroup.current) {
          vehicleGroup.current.add(robot);
        }
      },
      undefined,
      (error) => {
        console.error("URDF loading failed", error);
      }
    );
  }, [modelPath, scale, rotation, offset, bodyColor]);

  useFrame(() => {
    if (!ref.current || timeline.length === 0) return;
    const interp = getInterpolatedFrame(timeline, time);
    if (!interp) return;
    targetPosition.set(interp.x, 0, interp.y);
    ref.current.position.lerp(targetPosition, 0.2);
    targetQuaternion.setFromEuler(new THREE.Euler(0, -interp.yaw, 0));
    ref.current.quaternion.slerp(targetQuaternion, 0.2);
  });

  return (
    <>
      <group ref={ref}>
        <group ref={vehicleGroup} />
      </group>
      {mode === "free" && <OrbitControls />}
    </>
  );
}
