import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
const SCALE_RATE = 1;
const MODEL_MAPS = {
    0: "single-arm",
    1: "double-arm",
};
const TEXTURE_MAPS = {
    "BaseColor": "map",
    "Metalness": "metalnessMap",
    "Roughness": "roughnessMap",
    "Normal": "normalMap",
    "Transmission": "transmissionMap",
    "Emission": "emissiveMap",
};

export default class StreetLight {
    constructor(modelPath, position = { x: 0, y: 0, z: 0 }, onLoadCallback) {
        this.position = new THREE.Vector3(position.x, position.y, position.z);

        const loader = new GLTFLoader();
        loader.load(
            modelPath,
            (gltf) => {
                this.group = gltf.scene;
                for (let i = 0; i < this.group.children.length; i++) {
                    const model = MODEL_MAPS[i];
                    const model_texture_folder_path = `/textures/${model} street light pole/`;
                    for (const [textureName, textureMap] of Object.entries(TEXTURE_MAPS)) {
                        const texturePath = model_texture_folder_path + model + '_' + textureName + ".jpg";
                        const textureLoader = new THREE.TextureLoader();
                        const texture = textureLoader.load(texturePath);
                        this.group.children[i].material[textureMap] = texture;
                    }
                    this.group.children[i].material.transparent = true;
                    this.group.children[i].material.emissive = new THREE.Color(0xffffff);
                    this.group.children[i].material.roughness = 0.5;
                }
                this.group.position.set(this.position.x, this.position.y, this.position.z);
                this.group.scale.set(SCALE_RATE, SCALE_RATE, SCALE_RATE);

                if (onLoadCallback) onLoadCallback(this);
                console.log(this.group);
            },
            (xhr) => {
                console.log((xhr.loaded / xhr.total * 100) + '% loaded');
            },
            (error) => console.error("Error loading street light model:", error)
        );
    }
}