import { useEffect, useState, useRef } from 'react';
import { Group, Vector3, Euler } from 'three';
import { useFrame } from '@react-three/fiber';

export function useVehicleControls(
  groupRef: React.RefObject<Group | null>,
  getSpeed: () => number = () => 1
) {
  const [keys, setKeys] = useState<{ [key: string]: boolean }>({});
  const targetPosition = useRef(new Vector3());
  const targetRotation = useRef(new Euler());

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) =>
      setKeys((k) => ({ ...k, [e.key.toLowerCase()]: true }));
    const handleKeyUp = (e: KeyboardEvent) =>
      setKeys((k) => ({ ...k, [e.key.toLowerCase()]: false }));

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  const state = { moving: false, direction: 0 };

  useFrame((_, delta) => {
    if (!groupRef.current) return;

    const group = groupRef.current;
    const speed = getSpeed();
    const moveDir = new Vector3();
    const currentRotation = group.rotation.clone();

    state.moving = false;

    const forward = new Vector3(1, 0, 0).applyEuler(currentRotation);

    if (keys['w']) {
      moveDir.add(forward);
      state.moving = true;
      state.direction = 1;
    }
    if (keys['s']) {
      moveDir.sub(forward);
      state.moving = true;
      state.direction = -1;
    }

    moveDir.normalize().multiplyScalar(speed * delta);
    targetPosition.current.add(moveDir);

    const steerAmount = 1.5 * delta * (keys['s'] ? -1 : 1);
    if (keys['a']) targetRotation.current.y += steerAmount;
    if (keys['d']) targetRotation.current.y -= steerAmount;

    // Smooth rotation
    group.rotation.y += (targetRotation.current.y - group.rotation.y) * 0.2;

    // Smooth position
    group.position.lerp(targetPosition.current, 0.2);
  });

  return state;
}
