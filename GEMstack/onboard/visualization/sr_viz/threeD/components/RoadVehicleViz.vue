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
import { Car, CAR_WIDTH, CAR_HEIGHT, CAR_LENGTH } from "@/components/Car.js";
import TrafficLight from "@/components/TrafficLight.js";
import Human from "@/components/Human.js";
import {
    CAR_MODEL_PATH,
    TRAFFIC_LIGHT_MODEL_PATH,
} from "../utils/constants.js";

const container = ref(null);
const camera = ref(null);
const cameraMode = ref("free");
let scene, renderer, controls;

let car, roadGeometry, roadMaterial;
let lastTime = performance.now();
let cars = [];
let humans = [];
let trafficLights = [];
let carIdx = 0;

const keys = { forward: false, backward: false, left: false, right: false };

onMounted(() => {
    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    window.addEventListener("keypress", handleKeyPress);

    initScene();
    car = createCar(0, 0, 0, Math.PI / 2);
    for (let i = 0; i < 4; i++) {
        const offset = (i + 1) * 10;
        createCar(
            offset,
            0,
            (i % 2 === 0 ? -1 : 1) * (Math.random() * 5),
            Math.PI / 2
        );
    }
    createTrafficLight(0, 0, 5);
    createHuman(-3, 0, 4);
    createHuman(2, 0, 6);
    animate();
});

function createCar(x, y, z, heading) {
    const newCar = new Car(CAR_MODEL_PATH, { x: x, y: y, z: z }, heading, loadCallBack);
    cars.push(newCar);
    return newCar;
}

function createTrafficLight(x, y, z) {
    const newTrafficLight = new TrafficLight(
        TRAFFIC_LIGHT_MODEL_PATH,
        { x: x, y: y, z: z },
        loadCallBack
    );
    trafficLights.push(newTrafficLight);
    return newTrafficLight;
}

function createHuman(x, y, z) {
    const color = Math.random() * 0xffffff;
    const newHuman = new Human(color, { x: x, y: y, z: z });
    scene.add(newHuman.group);
    humans.push(newHuman);
    return newHuman;
}

function loadCallBack(object) {
    scene.add(object.group);
}

function handleKeyPress(event) {
    switch (event.key) {
        case "c":
        case "C":
            if (cameraMode.value === "free") {
                cameraMode.value = "follow";
            } else {
                cameraMode.value = "free";
            }
            break;
        case "q":
            carIdx = (carIdx - 1 + cars.length) % cars.length;
            car = cars[carIdx];
            break;
        case "e":
            carIdx = (carIdx + 1) % cars.length;
            car = cars[carIdx];
            break;
        default:
            break;
    }
}

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
        roughness: 1,
    });
    roadMaterial.map.wrapS = THREE.RepeatWrapping;
    roadMaterial.map.wrapT = THREE.RepeatWrapping;
    roadMaterial.map.repeat.set(100, 100);

    roadGeometry = new THREE.PlaneGeometry(1000, 1000);
    const roadMesh = new THREE.Mesh(roadGeometry, roadMaterial);
    roadMesh.rotation.x = -Math.PI / 2;

    roadMesh.position.set(0, 0, 0);

    scene.add(roadMesh);
    
    createRoadLines();
}

function createRoadLines() {
    const lineMaterialSolid = new THREE.LineBasicMaterial({ color: 0xffff00 });
    const lineMaterialDashed = new THREE.LineDashedMaterial({
        color: 0xffffff,
        dashSize: 5,
        gapSize: 5,
    });

    const roadWidth = 20;
    
    const centerLineGeometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(-500, 0.1, 0),
        new THREE.Vector3(500, 0.1, 0),
    ]);
    const centerLine = new THREE.Line(centerLineGeometry, lineMaterialDashed);
    centerLine.computeLineDistances();
    scene.add(centerLine);

    const leftLineGeometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(-500, 0.1, roadWidth / 2),
        new THREE.Vector3(500, 0.1, roadWidth / 2),
    ]);
    const leftLine = new THREE.Line(leftLineGeometry, lineMaterialSolid);
    scene.add(leftLine);

    const rightLineGeometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(-500, 0.1, -roadWidth / 2),
        new THREE.Vector3(500, 0.1, -roadWidth / 2),
    ]);
    const rightLine = new THREE.Line(rightLineGeometry, lineMaterialSolid);
    scene.add(rightLine);
}

function updateCamera() {
    if (!car || !car.group) return;
    if (cameraMode.value === "free") return;

    const carPosition = car.group.position;
    const carDirection = new THREE.Vector3();
    car.group.getWorldDirection(carDirection);

    camera.value.position.set(
        carPosition.x - carDirection.x * 2 * CAR_LENGTH,
        carPosition.y + 3 * CAR_HEIGHT,
        carPosition.z - carDirection.z * 2 * CAR_LENGTH
    );

    controls.target.set(
        carPosition.x + carDirection.x * 10 * CAR_LENGTH,
        carPosition.y + CAR_HEIGHT / 2,
        carPosition.z + carDirection.z * 10 * CAR_LENGTH
    );
}

function animate() {
    requestAnimationFrame(animate);
    const currentTime = performance.now();
    const dt = (currentTime - lastTime) / 1000;
    lastTime = currentTime;

    if (car) {
        car.update(keys, dt);
    }
    // for (const h of humans) {
    //     h.walk();
    // }
    updateCamera();
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
