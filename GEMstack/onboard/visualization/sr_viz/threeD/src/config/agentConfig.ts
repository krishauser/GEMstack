const agentConfig = {
    pedestrian: {
        name: "Pedestrian",
        modelPath: "/models/pedestrian.glb",
        scale: [0.2, 0.2, 0.2],
        rotation: [0, Math.PI, 0],
        offset: [0, 0, 0],
        bodyColor: "#808080",
    },
    cyclist: {
        name: "Cyclist",
        modelPath: "/models/cyclist.glb",
        scale: [1, 1, 1],
        rotation: [0, Math.PI, 0],
        offset: [0, 0.1, 0],
        bodyColor: "#808080",
    },
    vehicle: {
        name: "Vehicle",
        modelPath: "/models/vehicle.glb",
        scale: [1, 1, 1],
        rotation: [0, Math.PI, 0],
        offset: [0, 0.25, 0],
        bodyColor: "#808080",
    },
};

const currentAgent = agentConfig.pedestrian;

export { agentConfig, currentAgent };
