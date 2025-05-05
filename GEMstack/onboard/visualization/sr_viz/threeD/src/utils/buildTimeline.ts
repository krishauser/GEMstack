import { LogEntry } from '@/types/LogEntry';
import { FrameData } from '@/types/FrameData';
import { TimelineData } from '@/types/TimelineData';

export function buildTimeline(entries: LogEntry[]): TimelineData {
    const vehicle: FrameData[] = [];
    const agents: Record<string, FrameData[]> = {};
    const trafficLights: Record<string, FrameData[]> = {};
    const otherVehicles: Record<string, FrameData[]> = {};

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
        } else if (entry.type === 'AgentState') {
            const key = entry.key.trim();
            if (!agents[key]) agents[key] = [];
            agents[key].push(frame);
        } else if (entry.type === 'TrafficLightState') {
            const key = entry.key.trim();
            if (!trafficLights[key]) trafficLights[key] = [];
            trafficLights[key].push(frame);
        } else if (entry.type === 'OtherVehicleState') {
            const key = entry.key.trim();
            if (!otherVehicles[key]) otherVehicles[key] = [];
            otherVehicles[key].push(frame);
        }
    }

    return { vehicle, agents, trafficLights, otherVehicles };
}
