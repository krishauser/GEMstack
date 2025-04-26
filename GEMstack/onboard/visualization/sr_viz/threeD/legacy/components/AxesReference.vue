<template>
    <div ref="axesContainer"></div>
</template>

<script setup>
import * as THREE from "three";

const props = defineProps({
    camera: Object,
});

const axesContainer = ref(null);
let scene, axesCamera, axesRenderer, axesHelper;
let xLabel, yLabel, zLabel;

onMounted(() => {
    initAxes();
    animate();
    axesContainer.value.addEventListener("contextmenu", handleContextMenu);
});

function handleContextMenu(event) {
    event.preventDefault();
}

function initAxes() {
    scene = new THREE.Scene();

    axesCamera = new THREE.OrthographicCamera(-50, 50, 50, -50, 1, 500);
    axesCamera.position.set(5, 5, 5);
    axesCamera.lookAt(0, 0, 0);

    axesRenderer = new THREE.WebGLRenderer({ alpha: true });
    axesRenderer.setSize(400, 400);
    axesContainer.value.appendChild(axesRenderer.domElement);

    axesHelper = new THREE.AxesHelper(20);
    scene.add(axesHelper);

    xLabel = createLabel("X", 0xff0000, [22, 0, 0]);
    yLabel = createLabel("Y", 0x00ff00, [0, 22, 0]);
    zLabel = createLabel("Z", 0x0000ff, [0, 0, 22]);

    scene.add(xLabel, yLabel, zLabel);
}

function createLabel(text, color, position) {
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");
    canvas.width = 128;
    canvas.height = 128;

    ctx.fillStyle = "#" + color.toString(16).padStart(6, "0");
    ctx.font = "48px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);

    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({
        map: texture,
        transparent: true,
    });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(10, 10, 1);
    sprite.position.set(...position);

    return sprite;
}

function animate() {
    requestAnimationFrame(animate);
    if (props.camera) {
        axesCamera.position.copy(props.camera.position);
        axesCamera.lookAt(
            props.camera.position
                .clone()
                .add(props.camera.getWorldDirection(new THREE.Vector3()))
        );
    }
    axesRenderer.render(scene, axesCamera);
}

onBeforeUnmount(() => {
    axesRenderer.dispose();
    axesContainer.value.removeEventListener("contextmenu", handleContextMenu);
});
</script>
