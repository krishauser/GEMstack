import * as THREE from "three";

const POLE_RADIUS = 0.1;
const POLE_HEIGHT = 4;
const POLE_COLOR = 0x555555;
const BOX_WIDTH = 0.5;
const BOX_HEIGHT = 1.5;
const BOX_DEPTH = 0.4;
const BOX_COLOR = 0x222222;
const LIGHT_RADIUS = 0.15;
const LIGHT_OFFSET = 0.2;

export default class TrafficLight {
    constructor(position, direction = 0) {
        this.group = new THREE.Group();
        this.createPole();
        this.createLightBox();
        this.createLights();
        this.group.position.set(position.x, 0, position.z);
        this.group.rotation.y = direction;
    }

    createPole() {
        const poleGeometry = new THREE.CylinderGeometry(POLE_RADIUS, POLE_RADIUS, POLE_HEIGHT, 8);
        const poleMaterial = new THREE.MeshStandardMaterial({ color: POLE_COLOR });
        this.pole = new THREE.Mesh(poleGeometry, poleMaterial);
        this.pole.position.y = POLE_HEIGHT / 2;
        this.group.add(this.pole);
    }

    createLightBox() {
        const boxGeometry = new THREE.BoxGeometry(BOX_WIDTH, BOX_HEIGHT, BOX_DEPTH);
        const boxMaterial = new THREE.MeshStandardMaterial({ color: BOX_COLOR });
        this.lightBox = new THREE.Mesh(boxGeometry, boxMaterial);
        this.lightBox.position.set(0, POLE_HEIGHT * 7 / 8, 0);
        this.group.add(this.lightBox);
    }

    createLights() {
        this.lights = {
            red: this.createLight(0xff0000, this.lightBox.position.y + 2 * LIGHT_RADIUS),
            yellow: this.createLight(0xffff00, this.lightBox.position.y),
            green: this.createLight(0x00ff00, this.lightBox.position.y - 2 * LIGHT_RADIUS),
        };
        Object.values(this.lights).forEach(light => this.group.add(light));
        this.setLightState("red"); // Default state
    }

    createLight(color, yPos) {
        const lightGeometry = new THREE.SphereGeometry(LIGHT_RADIUS);
        const lightMaterial = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0 });
        const light = new THREE.Mesh(lightGeometry, lightMaterial);
        light.position.set(0, yPos, LIGHT_OFFSET);
        return light;
    }

    setLightState(state) {
        Object.keys(this.lights).forEach(key => {
            this.lights[key].material.emissiveIntensity = key === state ? 1 : 0;
        });
    }
}
