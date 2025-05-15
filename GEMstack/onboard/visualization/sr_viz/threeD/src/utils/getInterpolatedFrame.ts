import { FrameData } from "@/types/FrameData";

export function getInterpolatedFrame(frames: FrameData[], t: number): FrameData | null {
    if (frames.length === 0) return null;
    // If before first or after last, clamp
    if (t <= frames[0].time) return frames[0];
    if (t >= frames[frames.length - 1].time) return frames[frames.length - 1];

    // binary search for the first frame with time > t
    let lo = 0, hi = frames.length - 1;
    while (lo < hi) {
        const mid = Math.floor((lo + hi) / 2);
        if (frames[mid].time > t) hi = mid;
        else lo = mid + 1;
    }
    const b = frames[lo];
    const a = frames[lo - 1];

    const α = (t - a.time) / (b.time - a.time);
    return {
        time: t,
        x: a.x * (1 - α) + b.x * α,
        y: a.y * (1 - α) + b.y * α,
        z: a.z * (1 - α) + b.z * α,
        yaw: a.yaw * (1 - α) + b.yaw * α,
        pitch: a.pitch * (1 - α) + b.pitch * α,
        roll: a.roll * (1 - α) + b.roll * α,
        metadata: a.metadata, // or merge if needed
    };
}
