import { useEffect, useRef, useState } from "react";

export function usePlaybackTime() {
    const [time, setTime] = useState(0);
    const [play, setPlay] = useState(true);
    const [speed, setSpeed] = useState(1);
    const [duration, setDuration] = useState(0);

    const lastFrame = useRef(performance.now());

    const reset = () => {
        lastFrame.current = performance.now();
        setTime(0);
        setDuration(0);
        setPlay(true);
    };

    const restart = () => {
        lastFrame.current = performance.now();
        setTime(0);
        setPlay(true);
    }

    const togglePlay = () => {
        setPlay((prev) => {
            if (!prev) lastFrame.current = performance.now();
            return !prev;
        });
    };

    const setPlaybackSpeed = (newSpeed: number) => {
        if (newSpeed <= 0) return;
        setSpeed(newSpeed);
    };

    const moveToTime = (targetTime: number) => {
        setTime(targetTime);
        lastFrame.current = performance.now();
    }

    useEffect(() => {
        let rafId: number;

        const loop = () => {
            rafId = requestAnimationFrame(loop);

            if (!play) return;

            const now = performance.now();
            const delta = (now - lastFrame.current) / 1000;
            lastFrame.current = now;

            setTime((t) => t + delta * speed);
        };

        rafId = requestAnimationFrame(loop);
        return () => cancelAnimationFrame(rafId);
    }, [play, speed]);

    return { time, reset, restart, play, setPlay, togglePlay, speed, setPlaybackSpeed, moveToTime, duration, setDuration };
}
