"use client";

import React, { useEffect, useRef, useState } from "react";
import { useScrubber } from "./ScrubberContext";

export function decodeImage(msg: {
    encoding: string;
    data: Uint8Array;
    height: number;
    width: number;
    step: number;
}): ImageData {
    const { encoding, data, height, width, step } = msg;
    const rgba = new Uint8ClampedArray(width * height * 4);

    if (encoding === "bgr8") {
        for (let i = 0; i < height; i++) {
            for (let j = 0; j < width; j++) {
                const srcIdx = i * step + j * 3;
                const dstIdx = (i * width + j) * 4;
                rgba[dstIdx] = data[srcIdx + 2];
                rgba[dstIdx + 1] = data[srcIdx + 1];
                rgba[dstIdx + 2] = data[srcIdx];
                rgba[dstIdx + 3] = 255;
            }
        }
    } else if (encoding === "rgb8") {
        for (let i = 0; i < height; i++) {
            for (let j = 0; j < width; j++) {
                const srcIdx = i * step + j * 3;
                const dstIdx = (i * width + j) * 4;
                rgba[dstIdx] = data[srcIdx];
                rgba[dstIdx + 1] = data[srcIdx + 1];
                rgba[dstIdx + 2] = data[srcIdx + 2];
                rgba[dstIdx + 3] = 255;
            }
        }
    } else if (encoding === "mono8") {
        for (let i = 0; i < height; i++) {
            for (let j = 0; j < width; j++) {
                const gray = data[i * step + j];
                const dstIdx = (i * width + j) * 4;
                rgba[dstIdx] = gray;
                rgba[dstIdx + 1] = gray;
                rgba[dstIdx + 2] = gray;
                rgba[dstIdx + 3] = 255;
            }
        }
    } else if (encoding === "bayer_grbg8") {
        const get = (x: number, y: number): number =>
            x >= 0 && y >= 0 && x < width && y < height
                ? data[y * width + x]
                : 0;

        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                let r = 0,
                    g = 0,
                    b = 0;

                const val = get(x, y);

                if (y % 2 === 0) {
                    if (x % 2 === 0) {
                        g = val;
                        r = (get(x - 1, y) + get(x + 1, y)) / 2;
                        b = (get(x, y - 1) + get(x, y + 1)) / 2;
                    } else {
                        r = val;
                        g =
                            (get(x - 1, y) +
                                get(x + 1, y) +
                                get(x, y - 1) +
                                get(x, y + 1)) /
                            4;
                        b =
                            (get(x - 1, y - 1) +
                                get(x + 1, y - 1) +
                                get(x - 1, y + 1) +
                                get(x + 1, y + 1)) /
                            4;
                    }
                } else {
                    if (x % 2 === 0) {
                        b = val;
                        g =
                            (get(x - 1, y) +
                                get(x + 1, y) +
                                get(x, y - 1) +
                                get(x, y + 1)) /
                            4;
                        r =
                            (get(x - 1, y - 1) +
                                get(x + 1, y - 1) +
                                get(x - 1, y + 1) +
                                get(x + 1, y + 1)) /
                            4;
                    } else {
                        g = val;
                        r = (get(x, y - 1) + get(x, y + 1)) / 2;
                        b = (get(x - 1, y) + get(x + 1, y)) / 2;
                    }
                }

                const i = (y * width + x) * 4;
                rgba[i] = r;
                rgba[i + 1] = g;
                rgba[i + 2] = b;
                rgba[i + 3] = 255;
            }
        }
    } else {
        throw new Error(`Unsupported encoding: ${encoding}`);
    }

    return new ImageData(rgba, width, height);
}

type VideoPanelProps = {
    messages: any[];
};

export const VideoPanel: React.FC<VideoPanelProps> = ({ messages }) => {
    const { startTime, currentTime } = useScrubber();
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const containerRef = useRef<HTMLDivElement>(null);
    const imageRef = useRef<HTMLImageElement | null>(null);

    const [zoom, setZoom] = useState(1);
    const [offset, setOffset] = useState({ x: 0, y: 0 });
    const [isDragging, setIsDragging] = useState(false);
    const lastMouse = useRef({ x: 0, y: 0 });

    useEffect(() => {
        const msg = messages.find(
            (m) => m.timestamp >= startTime + currentTime
        );
        if (!msg || !msg.data) return;

        const canvas = canvasRef.current;
        const ctx = canvas?.getContext("2d", { willReadFrequently: true });
        if (!canvas || !ctx) return;

        try {
            const imageData = decodeImage(msg.data);
            imageRef.current = imageData;
            drawImage();
        } catch (e) {
            console.error("Failed to decode image:", e);
        }
    }, [messages, currentTime]);

    const drawImage = () => {
      requestAnimationFrame(() => {
        const canvas = canvasRef.current;
        const ctx = canvas?.getContext("2d", { willReadFrequently: true });
        const imageData = imageRef.current as ImageData;
        if (!canvas || !ctx || !imageData) return;
    
        const { width: cw, height: ch } = canvas;
        ctx.clearRect(0, 0, cw, ch);
    
        const imgAspect = imageData.width / imageData.height;
        const canvasAspect = cw / ch;
    
        let drawW = imageData.width * zoom;
        let drawH = imageData.height * zoom;
    
        if (imgAspect > canvasAspect) {
          drawW = cw * zoom;
          drawH = drawW / imgAspect;
        } else {
          drawH = ch * zoom;
          drawW = drawH * imgAspect;
        }
    
        const drawX = (cw - drawW) / 2 + offset.x;
        const drawY = (ch - drawH) / 2 + offset.y;
    
        const offscreen = document.createElement("canvas");
        offscreen.width = imageData.width;
        offscreen.height = imageData.height;
        offscreen.getContext("2d")?.putImageData(imageData, 0, 0);
    
        ctx.drawImage(offscreen, drawX, drawY, drawW, drawH);
      });
    };

    useEffect(() => {
        const container = containerRef.current;
        if (!container) return;

        const handleWheel = (e: WheelEvent) => {
            e.preventDefault();
            const zoomDelta = -e.deltaY * 0.001;
            setZoom((z) => Math.min(Math.max(z + zoomDelta, 0.1), 10));
        };

        container.addEventListener("wheel", handleWheel, { passive: false });
        return () => container.removeEventListener("wheel", handleWheel);
    }, []);

    useEffect(() => {
        drawImage();
    }, [zoom, offset]);

    useEffect(() => {
        const handleResize = () => {
            const container = containerRef.current;
            const canvas = canvasRef.current;
            if (container && canvas) {
                canvas.width = container.clientWidth;
                canvas.height = container.clientHeight;
                drawImage();
            }
        };

        window.addEventListener("resize", handleResize);
        handleResize();
        return () => window.removeEventListener("resize", handleResize);
    }, []);

    const handleMouseDown = (e: React.MouseEvent) => {
        setIsDragging(true);
        lastMouse.current = { x: e.clientX, y: e.clientY };
    };

    const handleMouseUp = () => {
        setIsDragging(false);
    };

    const handleMouseMove = (e: React.MouseEvent) => {
        if (!isDragging) return;
        const dx = e.clientX - lastMouse.current.x;
        const dy = e.clientY - lastMouse.current.y;
        lastMouse.current = { x: e.clientX, y: e.clientY };
        setOffset((prev) => ({ x: prev.x + dx, y: prev.y + dy }));
    };

    return (
        <div
            ref={containerRef}
            className="relative w-full h-full overflow-hidden"
            onMouseDown={handleMouseDown}
            onMouseUp={handleMouseUp}
            onMouseMove={handleMouseMove}
            onMouseLeave={handleMouseUp}
        >
            <canvas
                ref={canvasRef}
                className={`w-full h-full ${messages.length > 0 ? (isDragging ? "cursor-grabbing" : "cursor-grab") : ''}`}
            />
        </div>
    );
};
