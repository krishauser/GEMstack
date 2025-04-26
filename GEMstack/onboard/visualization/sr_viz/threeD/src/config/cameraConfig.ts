type CameraConfig = {
    position: [number, number, number];
    lookAt: [number, number, number];
    damping?: number;
};

type CameraConfigMap = {
    [key in 'first' | 'chase' | 'top' | 'side' | 'free']: Partial<CameraConfig>;
};

const cameraConfig: CameraConfigMap = {
    first: {
        position: [0, 1.5, -0.3],     // on top of vehicle
        lookAt: [2, 1.5, 0],       // looking forward (+Z)
        damping: 0.1,
    },
    chase: {
        position: [-6, 3, 0],      // behind vehicle
        lookAt: [0, 0, 0],       // looking at the rear
        damping: 0.1,
    },
    top: {
        position: [0, 10, 0],      // overhead
        lookAt: [0, 0, 0],         // looking forward
        damping: 0.1,
    },
    side: {
        position: [0, 2, 5],       // from the right
        lookAt: [0, 0, -10],       // looking at vehicle side
        damping: 0.1,
    },
    free: {},
};

export { cameraConfig };
