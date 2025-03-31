<template>
    <div class="w-full h-screen relative">
        <div ref="container" class="w-full h-full"></div>
        <AxesReference
            :camera="camera"
            class="absolute top-2 left-2 w-24 h-24"
        />
    </div>
</template>

<script setup>
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import Car from "@/components/Car.js";
import TrafficLight from "@/components/TrafficLight.js";
import {
    CAR_MODEL_PATH,
    TRAFFIC_LIGHT_MODEL_PATH,
} from "../utils/constants.js";

const container = ref(null);
const camera = ref(null);
let scene, renderer, controls;

let car, trafficLight1, roadGeometry, roadMaterial;
let roadTileGroup = [];
const NUM_TILES = 5;
const TILE_LENGTH = 50;
const ROAD_SIZE = 20;
const GRID_SIZE = 3;

const keys = { forward: false, backward: false, left: false, right: false };

onMounted(() => {
    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    initScene();
    car = new Car(CAR_MODEL_PATH, { x: 0, y: 0, z: 0 }, (loadedCar) => {
        scene.add(loadedCar.group);
    });

    trafficLight1 = new TrafficLight(
        TRAFFIC_LIGHT_MODEL_PATH,
        { x: 0, y: 0, z: 5 },
        (loadedLight) => {
            scene.add(loadedLight.group);
        }
    );

    animate();
});

function handleKeyDown(event) {
    switch (event.code) {
        case "ArrowUp":
        case "KeyW":
            keys.forward = true;
            break;
        case "ArrowDown":
        case "KeyS":
            keys.backward = true;
            break;
        case "ArrowLeft":
        case "KeyA":
            keys.left = true;
            break;
        case "ArrowRight":
        case "KeyD":
            keys.right = true;
            break;
    }
}

function handleKeyUp(event) {
    switch (event.code) {
        case "ArrowUp":
        case "KeyW":
            keys.forward = false;
            break;
        case "ArrowDown":
        case "KeyS":
            keys.backward = false;
            break;
        case "ArrowLeft":
        case "KeyA":
            keys.left = false;
            break;
        case "ArrowRight":
        case "KeyD":
            keys.right = false;
            break;
    }
}

function initScene() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xa0a0a0);

    camera.value = new THREE.PerspectiveCamera(
        75,
        window.innerWidth / window.innerHeight,
        0.1,
        1000
    );
    camera.value.position.set(0, 5, 10);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    container.value.appendChild(renderer.domElement);

    controls = new OrbitControls(camera.value, renderer.domElement);
    controls.enableDamping = true;

    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 5);
    scene.add(light);
    scene.add(new THREE.HemisphereLight(0xffffbb, 0x080820, 1));

    loadRoad();

    window.addEventListener("resize", onWindowResize, false);
}
function loadRoad() {
    const textureLoader = new THREE.TextureLoader();

    const diffuseTexture = textureLoader.load(
        "/textures/ground_grey_diff_1k.jpg"
    );
    const normalTexture = textureLoader.load(
        "/textures/ground_grey_nor_gl_1k.jpg"
    );
    const roughnessTexture = textureLoader.load(
        "/textures/ground_grey_rough_1k.jpg"
    );

    roadMaterial = new THREE.MeshStandardMaterial({
        map: diffuseTexture,
        normalMap: normalTexture,
        roughnessMap: roughnessTexture,
        roughness: 0.7,
    });
    roadMaterial.map.wrapS = THREE.RepeatWrapping;
    roadMaterial.map.wrapT = THREE.RepeatWrapping;
    roadMaterial.map.repeat.set(2, 2);

    roadGeometry = new THREE.PlaneGeometry(50, 50);
    const roadMesh = new THREE.Mesh(roadGeometry, roadMaterial);
    roadMesh.rotation.x = -Math.PI / 2;

    roadMesh.position.set(0, 0, 0);

    scene.add(roadMesh);
    roadTileGroup.push(roadMesh);
}

function updateRoadTiles() {}

function animate() {
    requestAnimationFrame(animate);
    if (car) {
        car.update(keys);
    }
    // updateRoadTiles();
    controls.update();
    renderer.render(scene, camera.value);
}

function onWindowResize() {
    camera.value.aspect = window.innerWidth / window.innerHeight;
    camera.value.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}
</script>

<style>
body {
    margin: 0;
    overflow: hidden;
}
</style>
