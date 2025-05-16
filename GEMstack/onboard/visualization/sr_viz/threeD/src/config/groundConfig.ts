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
    size: [1000, 1000],
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    cellSize: 2,
    cellThickness: 0.5,
    cellColor: "#222222",
    sectionSize: 10,
    sectionThickness: 1,
    sectionColor: "#444444",
    fadeDistance: 1500,
    fadeStrength: 0,
    infiniteGrid: false,
};


export { groundConfig };