"use client";

import { useEffect, useState, useRef } from "react";
import { useThree, useFrame } from "@react-three/fiber";
import * as THREE from "three";
import { cameraConfig } from "@/config/cameraConfig";

type CameraModeKey = "chase" | "top" | "side" | "free" | "first";

export function useCameraController(
  carRef: React.RefObject<THREE.Object3D | null>
): CameraModeKey {
  const { camera, gl } = useThree();
  const [mode, setMode] = useState<CameraModeKey>("chase");
  const lastMode = useRef<CameraModeKey>("chase");
  const modeRef = useRef<CameraModeKey>("chase");

  const smoothedPos = useRef(new THREE.Vector3());
  const smoothedLookAt = useRef(new THREE.Vector3());

  useEffect(() => { modeRef.current = mode; }, [mode]);

  useEffect(() => {
    const handleKey = (e: KeyboardEvent) => {
      switch (e.key) {
        case "1": setMode("first"); break;
        case "2": setMode("chase"); break;
        case "3": setMode("top"); break;
        case "4": setMode("side"); break;
        case "5": lastMode.current = modeRef.current; setMode("free"); break;
        case "0": if (lastMode.current !== "free") setMode(lastMode.current); break;
      }
    };
    window.addEventListener("keydown", handleKey);
    return () => window.removeEventListener("keydown", handleKey);
  }, []);

  useEffect(() => {
    const dom = gl.domElement;
    const activateFree = () => {
      if (modeRef.current !== "free") {
        lastMode.current = modeRef.current;
        setMode("free");
      }
    };
    const onMouseDown = (e: MouseEvent) => { if (e.button === 0) activateFree(); };
    const onWheel = () => activateFree();

    dom.addEventListener("mousedown", onMouseDown);
    dom.addEventListener("wheel", onWheel);
    return () => {
      dom.removeEventListener("mousedown", onMouseDown);
      dom.removeEventListener("wheel", onWheel);
    };
  }, [gl.domElement]);

  useFrame((_, delta) => {
    if (modeRef.current === "free" || !carRef.current) return;

    const config = cameraConfig[modeRef.current];
    if (!config) return;

    const alpha = 1 - Math.exp(-10.0 * delta);

    const carPos = new THREE.Vector3();
    carRef.current.getWorldPosition(carPos);
    const carQuat = new THREE.Quaternion();
    carRef.current.getWorldQuaternion(carQuat);

    const [px, py, pz] = config.position ?? [0, 0, 0];
    const camTargetPos = carPos.clone().add(
      new THREE.Vector3(px, py, pz).applyQuaternion(carQuat)
    );
    smoothedPos.current.lerp(camTargetPos, alpha);
    camera.position.copy(smoothedPos.current);

    const [lx, ly, lz] = config.lookAt ?? [0, 0, 0];
    const camLookAt = carPos.clone().add(
      new THREE.Vector3(lx, ly, lz).applyQuaternion(carQuat)
    );
    smoothedLookAt.current.lerp(camLookAt, alpha);
    camera.lookAt(smoothedLookAt.current);
  });

  return mode;
}