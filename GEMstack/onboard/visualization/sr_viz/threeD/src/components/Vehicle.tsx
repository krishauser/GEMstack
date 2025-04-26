import React, { useRef, useMemo, useEffect } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { useCameraController } from "@/hooks/useCameraController";
import { currentVehicle } from "@/config/vehicleConfig";
import { FrameData } from "@/types/FrameData";
import { OrbitControls } from "@react-three/drei";
import URDFLoader from "urdf-loader";

interface VehicleProps {
  timeline: FrameData[];
  time: number;
}

export default function Vehicle({ timeline, time }: VehicleProps) {
  const ref = useRef<THREE.Group>(null);
  const vehicleGroup = useRef<THREE.Group>(new THREE.Group()); // ← create empty group for the robot
  const mode = useCameraController(ref, timeline, time);

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
          if (child instanceof THREE.Mesh &&
              child.material instanceof THREE.MeshStandardMaterial) {
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

    const frame = timeline.find((f) => f.time >= time) ?? timeline.at(-1);
    if (!frame) return;

    targetPosition.set(frame.x, 0, frame.y);
    ref.current.position.lerp(targetPosition, 0.2);

    targetQuaternion.setFromEuler(new THREE.Euler(0, -frame.yaw, 0));
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
