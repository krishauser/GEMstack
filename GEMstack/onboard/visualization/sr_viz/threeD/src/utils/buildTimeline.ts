import { LogEntry } from "@/types/LogEntry";
import { FrameData } from "@/types/FrameData";
import { TimelineData } from "@/types/TimelineData";

export function buildTimeline(entries: LogEntry[]): TimelineData {
    const vehicle: FrameData[] = [];
    const pedestrians: Record<string, FrameData[]> = {};
    const trafficLights: Record<string, FrameData[]> = {};
    const trafficCones: Record<string, FrameData[]> = {};
    const otherVehicles: Record<string, FrameData[]> = {};

    for (const entry of entries) {
        const pose = entry.data?.pose;
        if (!pose || typeof pose.x !== "number" || typeof pose.y !== "number") continue;

        const frame: FrameData = {
            time: entry.time,
            x: pose.x ?? 0,
            y: pose.y ?? 0,
            z: pose.z ?? 0,
            yaw: pose.yaw ?? 0,
            pitch: pose.pitch ?? 0,
            roll: pose.roll ?? 0,
            metadata: entry.key === "vehicle" ? { ...entry.data } : undefined,
        };

        if (entry.key === "vehicle") {
            vehicle.push(frame);
        } else if (entry.type === "PedestrianState") {
            const key = entry.key.trim();
            if (!pedestrians[key]) pedestrians[key] = [];
            pedestrians[key].push(frame);
        } else if (entry.type === "TrafficLightState") {
            const key = entry.key.trim();
            if (!trafficLights[key]) trafficLights[key] = [];
            trafficLights[key].push(frame);
        } else if (entry.type === "TrafficConeState") {
            const key = entry.key.trim();
            if (!trafficCones[key]) trafficCones[key] = [];
            trafficCones[key].push(frame);
        } else if (entry.type === "OtherVehicleState") {
            const key = entry.key.trim();
            if (!otherVehicles[key]) otherVehicles[key] = [];
            otherVehicles[key].push(frame);
        }
    }

    return { vehicle, pedestrians, trafficLights, trafficCones, otherVehicles };
}
