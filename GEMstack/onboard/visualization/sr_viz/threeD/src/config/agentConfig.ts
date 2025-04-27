const agentConfig = {
    pedestrian: {
        name: "Pedestrian",
        modelPath: "/models/pedestrian.glb",
        scale: [0.2, 0.2, 0.2],
        rotation: [0, Math.PI, 0],
        offset: [0, 0, 0],
        bodyColor: "#808080",
    }
};

const currentAgent = agentConfig.pedestrian;

export { agentConfig, currentAgent };
