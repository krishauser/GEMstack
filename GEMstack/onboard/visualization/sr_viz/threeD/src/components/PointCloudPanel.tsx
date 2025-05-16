"use client";

import React, { useEffect, useRef, useState } from "react";
import { useScrubber } from "./ScrubberContext";
import * as THREE from "three";
import { OrbitControls } from "three-stdlib";
import { Select, SelectChangeEvent, MenuItem } from "@mui/material";

function parsePointCloud2(msg: any): THREE.Points {
    const { data, point_step, is_bigendian, fields } = msg;
    const positions: number[] = [];
    const colors: number[] = [];
    const littleEndian = !is_bigendian;

    const fieldMap = Object.fromEntries(
        fields.map((f: any) => [f.name, f.offset])
    );
    const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);

    for (let i = 0; i < data.length; i += point_step) {
        const x = dv.getFloat32(i + fieldMap["x"], littleEndian);
        const y = dv.getFloat32(i + fieldMap["y"], littleEndian);
        const z = dv.getFloat32(i + fieldMap["z"], littleEndian);

        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z))
            continue;

        positions.push(x, y, z);

        if ("rgba" in fieldMap) {
            const rgba = dv.getUint32(i + fieldMap["rgba"], littleEndian);
            const r = (rgba >> 24) & 0xff;
            const g = (rgba >> 16) & 0xff;
            const b = (rgba >> 8) & 0xff;
            colors.push(r / 255, g / 255, b / 255);
        } else if ("rgb" in fieldMap) {
            const rgb = dv.getUint32(i + fieldMap["rgb"], littleEndian);
            const r = (rgb >> 16) & 0xff;
            const g = (rgb >> 8) & 0xff;
            const b = rgb & 0xff;
            colors.push(r / 255, g / 255, b / 255);
        } else {
            colors.push(1, 1, 1);
        }
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
        "position",
        new THREE.Float32BufferAttribute(positions, 3)
    );
    geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    const material = new THREE.PointsMaterial({
        size: 0.05,
        vertexColors: true,
    });

    return new THREE.Points(geometry, material);
}

function buildTransformLookup(tfMessages: any[], time: number) {
    const latestTransforms = new Map<string, any>();

    for (const msg of tfMessages) {
        const transforms = msg.data.transforms || [];
        for (const t of transforms) {
            if (t.header.stamp.sec + t.header.stamp.nsec * 1e-9 < time)
                continue;
            const key = `${t.header.frame_id}->${t.child_frame_id}`;
            if (!latestTransforms.has(key)) {
                latestTransforms.set(key, t);
            }
        }
    }

    return latestTransforms;
}

function getTransformMatrix(
    lookup: Map<string, any>,
    from: string,
    to: string
): THREE.Matrix4 | null {
    const key = `${from}->${to}`;
    const tf = lookup.get(key);
    if (!tf) return null;

    const { translation, rotation } = tf.transform;
    const position = new THREE.Vector3(
        translation.x,
        translation.y,
        translation.z
    );
    const quaternion = new THREE.Quaternion(
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w
    );

    const matrix = new THREE.Matrix4();
    matrix.makeRotationFromQuaternion(quaternion);
    matrix.setPosition(position);

    return matrix;
}

export const PointCloudPanel = ({
    messages,
    tfMessages,
    initialTopic,
}: {
    messages: Record<string, any[]>;
    tfMessages: any[];
    initialTopic?: string;
}) => {
    const mountRef = useRef<HTMLDivElement>(null);
    const { startTime, currentTime } = useScrubber();
    const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
    const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
    const sceneRef = useRef<THREE.Scene | null>(null);
    const controlsRef = useRef<OrbitControls | null>(null);
    const pointCloudRef = useRef<THREE.Points | null>(null);
    const [selectedTopic, setSelectedTopic] = useState(
        initialTopic || Object.keys(messages)[0] || ""
    );

    useEffect(() => {
        if (initialTopic && messages[initialTopic]) {
            setSelectedTopic(initialTopic);
        }
    }, [initialTopic]);

    useEffect(() => {
        if (!selectedTopic || !messages[selectedTopic]) return;
        if (!mountRef.current) return;

        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(
            75,
            mountRef.current.clientWidth / mountRef.current.clientHeight,
            0.1,
            1000
        );
        camera.position.z = 5;

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(
            mountRef.current.clientWidth,
            mountRef.current.clientHeight
        );
        mountRef.current.appendChild(renderer.domElement);

        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;

        sceneRef.current = scene;
        cameraRef.current = camera;
        rendererRef.current = renderer;
        controlsRef.current = controls;

        const animate = () => {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        };
        animate();

        return () => {
            if (rendererRef.current) {
                rendererRef.current.dispose();
                const canvas = rendererRef.current.domElement;
                canvas.remove();
            }
        };
    }, []);

    useEffect(() => {
        const scene = sceneRef.current;
        if (!scene) return;

        const msg = messages[selectedTopic].find(
            (m) => m.timestamp >= startTime + currentTime
        );
        if (!msg) return;

        if (pointCloudRef.current) {
            scene.remove(pointCloudRef.current);
            pointCloudRef.current.geometry.dispose();
            (pointCloudRef.current.material as THREE.Material).dispose();
        }

        const newCloud = parsePointCloud2(msg.data);
        pointCloudRef.current = newCloud;

        const transforms = buildTransformLookup(
            tfMessages,
            currentTime + startTime
        );
        const cloudFrame = msg.data.header.frame_id;
        const targetFrame = "base_link";

        const matrix = getTransformMatrix(transforms, cloudFrame, targetFrame);

        if (matrix) {
            newCloud.applyMatrix4(matrix);
        }
        scene.add(newCloud);
    }, [startTime, currentTime, messages, tfMessages]);

    useEffect(() => {
        if (!mountRef.current || !rendererRef.current || !cameraRef.current)
            return;

        const observer = new ResizeObserver(() => {
            const width = mountRef.current!.clientWidth;
            const height = mountRef.current!.clientHeight;

            rendererRef.current!.setSize(width, height);
            cameraRef.current!.aspect = width / height;
            cameraRef.current!.updateProjectionMatrix();
        });

        observer.observe(mountRef.current);

        return () => observer.disconnect();
    }, []);

    return (
        <div ref={mountRef} className="w-full h-full">
            <div className="absolute top-2 right-20">
                <Select
                    value={selectedTopic}
                    onChange={(e: SelectChangeEvent<string>) => {
                        setSelectedTopic(e.target.value);
                    }}
                    size="small"
                    disabled={Object.keys(messages).length === 0}
                    sx={{
                        backgroundColor: "white",
                        opacity: 0.25,
                        borderRadius: "9999px",
                        "&:hover": {
                            opacity: 0.5,
                        },
                    }}
                >
                    {Object.keys(messages).map((topic) => (
                        <MenuItem key={topic} value={topic}>
                            {topic}
                        </MenuItem>
                    ))}
                </Select>
            </div>
        </div>
    );
};
