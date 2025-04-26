import { LogEntry } from '@/types/LogEntry';
import { FrameData } from '@/types/FrameData';
import { TimelineData } from '@/types/TimelineData';

export function buildTimeline(entries: LogEntry[]): TimelineData {
    const vehicle: FrameData[] = [];
    const agents: Record<string, FrameData[]> = {};

    for (const entry of entries) {
        const pose = entry.data?.pose;
        if (!pose || typeof pose.x !== 'number' || typeof pose.y !== 'number') continue;

        const frame: FrameData = {
            time: entry.time,
            x: pose.x,
            y: pose.y,
            z: pose.z ?? 0,
            yaw: pose.yaw ?? 0,
            pitch: pose.pitch ?? 0,
            roll: pose.roll ?? 0
        };

        if (entry.key === 'vehicle') {
            vehicle.push(frame);
        } else {
            const key = entry.key.trim();
            if (!agents[key]) agents[key] = [];
            agents[key].push(frame);
        }
    }

    return { vehicle, agents };
}
