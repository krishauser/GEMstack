import { TimelineStore } from '@/types/TimelineStore';
import { create } from 'zustand';

export const useTimelineStore = create<TimelineStore>((set) => ({
    vehicle: [],
    pedestrians: {},
    trafficLights: {},
    trafficCones: {},
    otherVehicles: {},
    setTimeline: (timeline) => set(timeline),
}));
