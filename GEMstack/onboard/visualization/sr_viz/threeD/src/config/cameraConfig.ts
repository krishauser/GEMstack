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
        position: [0, 1.5, 0],
        lookAt: [10, 2, 0],
        damping: 0.1,
    },
    chase: {
        position: [-6, 3, 0],
        lookAt: [0, 0, 0],
        damping: 0.1,
    },
    top: {
        position: [0, 10, 0],
        lookAt: [0, 0, 0],
        damping: 0.1,
    },
    side: {
        position: [0, 2, 5],
        lookAt: [0, 0, -10],
        damping: 0.1,
    },
    free: {},
};

export { cameraConfig };
