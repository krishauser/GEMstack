import { useEffect, useState, useRef } from "react";
import { useThree, useFrame } from "@react-three/fiber";
import * as THREE from "three";
import { FrameData } from "@/types/FrameData";
import { cameraConfig } from "@/config/cameraConfig";

type CameraModeKey = "chase" | "top" | "side" | "free" | "first";

export function useCameraController(
  carRef: React.RefObject<THREE.Object3D | null>,
  timeline: FrameData[] = [],
  time: number
): CameraModeKey {
  const { camera, gl } = useThree();
  const [mode, setMode] = useState<CameraModeKey>("chase");
  const lastMode = useRef<CameraModeKey>("chase");
  const modeRef = useRef<CameraModeKey>("chase");

  const smoothedPos = useRef(new THREE.Vector3());
  const smoothedLookAt = useRef(new THREE.Vector3());

  useEffect(() => {
    modeRef.current = mode;
  }, [mode]);

  // Allow switching between all modes at any time
  useEffect(() => {
    const handleKey = (e: KeyboardEvent) => {
      switch (e.key) {
        case "1":
          setMode("first");
          break;
        case "2":
          setMode("chase");
          break;
        case "3":
          setMode("top");
          break;
        case "4":
          setMode("side");
          break;
        case "5":
          lastMode.current = modeRef.current;
          setMode("free");
          break;
        case "0":
          if (lastMode.current !== "free") {
            setMode(lastMode.current);
          }
          break;
      }
    };

    window.addEventListener("keydown", handleKey);
    return () => window.removeEventListener("keydown", handleKey);
  }, []);

  // Switch to free mode on scroll or click
  useEffect(() => {
    const dom = gl.domElement;

    const activateFreeMode = () => {
      if (modeRef.current !== "free") {
        lastMode.current = modeRef.current;
        setMode("free");
      }
    };

    const onMouseDown = (e: MouseEvent) => {
      if (e.button === 0) activateFreeMode(); // Left click
    };

    const onWheel = () => {
      activateFreeMode(); // Scroll
    };

    dom.addEventListener("mousedown", onMouseDown);
    dom.addEventListener("wheel", onWheel);

    return () => {
      dom.removeEventListener("mousedown", onMouseDown);
      dom.removeEventListener("wheel", onWheel);
    };
  }, [gl.domElement]);

  useFrame(() => {
    if (modeRef.current === "free") return;
    if (!carRef.current) return;

    // Always fallback to a stable frame if timeline is empty
    const frame =
      timeline.find((f) => f.time >= time) ??
      timeline.at(-1) ??
      { x: 0, y: 0, yaw: 0 };

    const config = cameraConfig[modeRef.current];
    if (!config?.position || !config?.lookAt) return;

    const carPos = new THREE.Vector3(frame.x, 0, frame.y);
    const carQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, -frame.yaw, 0));

    const damping = config.damping ?? 0.1;

    // camera offset (relative to vehicle) and final position
    const offset = new THREE.Vector3(...config.position).applyQuaternion(carQuat);
    const camTargetPos = carPos.clone().add(offset);
    smoothedPos.current.lerp(camTargetPos, damping);
    camera.position.copy(smoothedPos.current);

    // look at offset
    const lookAtOffset = new THREE.Vector3(...config.lookAt).applyQuaternion(carQuat);
    const camLookAt = carPos.clone().add(lookAtOffset);
    smoothedLookAt.current.lerp(camLookAt, damping);
    camera.lookAt(smoothedLookAt.current);
  });

  return mode;
}
