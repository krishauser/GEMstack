import { FrameData } from './FrameData';

export interface TimelineData {
    vehicle: FrameData[];
    agents: Record<string, FrameData[]>;
}
