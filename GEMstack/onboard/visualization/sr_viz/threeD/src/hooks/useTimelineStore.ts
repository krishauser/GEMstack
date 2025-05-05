import { TimelineStore } from '@/types/TimelineStore';
import { create } from 'zustand';

export const useTimelineStore = create<TimelineStore>((set) => ({
    vehicle: [],
    agents: {},
    trafficLights: {},
    otherVehicles: {},
    setTimeline: (timeline) => set(timeline),
}));
