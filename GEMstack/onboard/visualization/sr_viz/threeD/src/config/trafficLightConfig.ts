const trafficLightConfig = {
    trafficLight: {
        name: "Traffic Light",
        modelPath: "/models/traffic_light.glb",
        scale: [0.03, 0.03, 0.03],
        rotation: [0, 0, 0],
        offset: [0, 0, 0],
        bodyColor: "#FFD700",
    },
};

const currentTrafficLight = trafficLightConfig.trafficLight;

export { trafficLightConfig, currentTrafficLight };
