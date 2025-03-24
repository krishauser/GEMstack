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
import Car from "@/components/Car.js";
import Human from "@/components/Human.js";
import TrafficLight from "@/components/TrafficLight.js";

const container = ref(null);
const camera = ref(null);
let scene, renderer, controls;

onMounted(() => {
    initScene();
    animate();
});

function addTrafficLines() {
    const lineMaterial = new THREE.MeshStandardMaterial({ color: 0xffffff });

    for (let i = -24; i < 25; i += 4) {
        const lineGeometry = new THREE.PlaneGeometry(0.2, 2);
        const line = new THREE.Mesh(lineGeometry, lineMaterial);
        line.rotation.x = -Math.PI / 2;
        line.position.set(0, 0.01, i);
        scene.add(line);
    }

    const sideLineMaterial = new THREE.MeshStandardMaterial({
        color: 0xffffff,
    });

    const leftLineGeometry = new THREE.PlaneGeometry(0.2, 50);
    const rightLineGeometry = new THREE.PlaneGeometry(0.2, 50);

    const leftLine = new THREE.Mesh(leftLineGeometry, sideLineMaterial);
    leftLine.rotation.x = -Math.PI / 2;
    leftLine.position.set(-4.5, 0.01, 0);

    const rightLine = new THREE.Mesh(rightLineGeometry, sideLineMaterial);
    rightLine.rotation.x = -Math.PI / 2;
    rightLine.position.set(4.5, 0.01, 0);

    scene.add(leftLine, rightLine);
}

function createTrafficLight(position) {
    const lightGroup = new THREE.Group();
    const poleGeometry = new THREE.CylinderGeometry(0.1, 0.1, 3);
    const poleMaterial = new THREE.MeshStandardMaterial({ color: 0x555555 });
    const pole = new THREE.Mesh(poleGeometry, poleMaterial);
    pole.position.y = 1.5;
    lightGroup.add(pole);

    const lightGeometry = new THREE.SphereGeometry(0.2);
    const redLightMaterial = new THREE.MeshStandardMaterial({
        color: 0xff0000,
    });
    const redLight = new THREE.Mesh(lightGeometry, redLightMaterial);
    redLight.position.set(0, 2.7, 0);
    lightGroup.add(redLight);

    lightGroup.position.set(position.x, 0, position.z);
    scene.add(lightGroup);
}

function createPedestrian(position) {
    const pedestrianGeometry = new THREE.SphereGeometry(0.3);
    const pedestrianMaterial = new THREE.MeshStandardMaterial({
        color: 0x00ff00,
    });
    const pedestrian = new THREE.Mesh(pedestrianGeometry, pedestrianMaterial);
    pedestrian.position.set(position.x, 0.3, position.z);
    scene.add(pedestrian);
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
    const light2 = new THREE.HemisphereLight(0xffffbb, 0x080820, 1);
    scene.add(light2);

    const roadGeometry = new THREE.PlaneGeometry(10, 50);
    const roadMaterial = new THREE.MeshStandardMaterial({ color: 0x333333 });
    const road = new THREE.Mesh(roadGeometry, roadMaterial);
    road.rotation.x = -Math.PI / 2;
    scene.add(road);
    addTrafficLines();

    const car1 = new Car(0xff0000, { x: -2, z: 5 });
    const car2 = new Car(0x0000ff, { x: 2, z: -5 }, Math.PI);
    scene.add(car1.group);
    scene.add(car2.group);

    const trafficLight1 = new TrafficLight({ x: 0, z: 10 }, Math.PI);
    const trafficLight2 = new TrafficLight({ x: 0, z: -10 });
    scene.add(trafficLight1.group, trafficLight2.group);

    const human1 = new Human(0xff0000, { x: -3, y: 0, z: 1 });
    const human2 = new Human(0x0000ff, { x: 3, y: 0, z: -1 });
    scene.add(human1.group, human2.group);

    window.addEventListener("resize", onWindowResize, false);
}

function animate() {
    requestAnimationFrame(animate);

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
