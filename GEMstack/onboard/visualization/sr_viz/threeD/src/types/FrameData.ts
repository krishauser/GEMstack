export interface FrameData {
    time: number;
    x: number;
    y: number;
    z: number;
    yaw: number;
    pitch: number;
    roll: number;
    metadata?: Record<string, any>;
}