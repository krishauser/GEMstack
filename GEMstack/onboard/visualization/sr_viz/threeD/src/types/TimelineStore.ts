import { TimelineData } from './TimelineData';

export interface TimelineStore extends TimelineData {
    setTimeline: (timeline: TimelineData) => void;
}
