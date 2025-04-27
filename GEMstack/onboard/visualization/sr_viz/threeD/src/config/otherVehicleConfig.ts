const otherVehicleConfig = {
    car: {
        name: "Car",
        modelPath: "/models/model/gem_e2.urdf",
        scale: [1, 1, 1],
        rotation: [-Math.PI / 2, 0, 0],
        offset: [0, 0, 0],
        bodyColor: "#808080",
    },
};

const currentOtherVehicle = otherVehicleConfig.car;

export { otherVehicleConfig, currentOtherVehicle };
