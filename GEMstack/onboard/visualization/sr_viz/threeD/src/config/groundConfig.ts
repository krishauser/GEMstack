export interface GroundConfig {
    size: [number, number];
    position: [number, number, number];
    rotation: [number, number, number];
    cellSize: number;
    cellThickness: number;
    cellColor: string;
    sectionSize: number;
    sectionThickness: number;
    sectionColor: string;
    fadeDistance: number;
    fadeStrength: number;
    infiniteGrid: boolean;
}

const groundConfig: GroundConfig = {
    size: [20000, 20000],
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    cellSize: 1,
    cellThickness: 0.5,
    cellColor: "#000000",
    sectionSize: 10,
    sectionThickness: 1,
    sectionColor: "#000000",
    fadeDistance: 100,
    fadeStrength: 1,
    infiniteGrid: false,
};


export { groundConfig };
