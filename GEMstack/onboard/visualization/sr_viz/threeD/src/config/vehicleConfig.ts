const vehicles = {
    gemE4: {
        name: "Gem E4",
        modelPath: "/models/model/gem_e4.urdf",
        scale: [1, 1, 1],
        rotation: [-Math.PI / 2, 0, 0],
        offset: [0, 0, 0],
        bodyColor: "#808080",
    }
};

const currentVehicle = vehicles.gemE4;

export { vehicles, currentVehicle };
