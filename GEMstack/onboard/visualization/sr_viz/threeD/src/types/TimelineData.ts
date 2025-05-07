import { FrameData } from './FrameData';

export interface TimelineData {
    vehicle: FrameData[];
    agents: Record<string, FrameData[]>;
    trafficLights: Record<string, FrameData[]>;
    trafficCones: Record<string, FrameData[]>;
    otherVehicles: Record<string, FrameData[]>;
}
