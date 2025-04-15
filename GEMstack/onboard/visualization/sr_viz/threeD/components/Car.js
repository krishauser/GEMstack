import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import  URDFLoader  from 'urdf-loader';
const CAR_X = 19651.6005859375;
const CAR_Y = 14238.344896316528;
const CAR_Z = 47565.92169100046;
const SCALE_RATE = 0.0002;
export const CAR_WIDTH = CAR_X * SCALE_RATE;
export const CAR_HEIGHT = CAR_Y * SCALE_RATE;
export const CAR_LENGTH = CAR_Z * SCALE_RATE;

export class Car {
    constructor(modelPath, position = { x: 0, y: 0, z: 0 }, heading = Math.PI / 2, onLoadCallback) {
        this.position = new THREE.Vector3(position.x, position.y, position.z);
        this.heading = heading;
        this.velocity = 0;
        this.wheelBase = CAR_LENGTH * 5 / 8;
        this.maxSpeed = 20;
        this.acceleration = 0.5;
        this.friction = 0.98;
        this.steerAngle = 0;
        this.maxSteerAngle = Math.PI / 6;

        // const loader = new GLTFLoader();
        // loader.load(
        //     modelPath,
        //     (gltf) => {
        //         this.group = gltf.scene;
        //         this.group.position.set(this.position.x, this.position.y, this.position.z);
        //         this.group.scale.set(SCALE_RATE, SCALE_RATE, SCALE_RATE);
        //         this.group.rotation.y = this.heading;

        //         if (onLoadCallback) onLoadCallback(this);
        //     },
        //     undefined,
        //     (error) => console.error("Error loading car model:", error)
        // );
        const loader = new URDFLoader();
        loader.load(
            modelPath,
            (car) => {
                this.group = car;
                this.group.position.set(this.position.x, this.position.y, this.position.z);
                // this.group.scale.set(SCALE_RATE, SCALE_RATE, SCALE_RATE);
                this.group.rotation.x = -Math.PI / 2;
                this.group.rotation.z = -this.heading;

                if (onLoadCallback) onLoadCallback(this);
            },
            undefined,
            (error) => console.error("Error loading car model:", error)
        );
    }

    update(keys, dt) {
        if (!this.group) return;

        if (keys.forward) this.velocity = Math.min(this.velocity + this.acceleration, this.maxSpeed);
        if (keys.backward) this.velocity = Math.max(this.velocity - this.acceleration, -this.maxSpeed);
        this.velocity *= this.friction;

        if (keys.right) this.steerAngle = Math.max(this.steerAngle - 0.02, -this.maxSteerAngle);
        if (keys.left) this.steerAngle = Math.min(this.steerAngle + 0.02, this.maxSteerAngle);
        if (!keys.left && !keys.right) this.steerAngle *= 0.9;

        const frontWheel = this.position.clone().add(new THREE.Vector3(
            Math.sin(this.heading) * (this.wheelBase / 2) - Math.cos(this.heading) * (CAR_WIDTH / 2),
            0,
            Math.cos(this.heading) * (this.wheelBase / 2) + Math.sin(this.heading) * (CAR_WIDTH / 2)
        ));
        const backWheel = this.position.clone().add(new THREE.Vector3(
            -Math.sin(this.heading) * (this.wheelBase / 2) - Math.cos(this.heading) * (CAR_WIDTH / 2),
            0,
            -Math.cos(this.heading) * (this.wheelBase / 2) + Math.sin(this.heading) * (CAR_WIDTH / 2)
        ));

        backWheel.add(new THREE.Vector3(
            Math.sin(this.heading) * this.velocity * dt,
            0,
            Math.cos(this.heading) * this.velocity * dt
        ));
        frontWheel.add(new THREE.Vector3(
            Math.sin(this.heading + this.steerAngle) * this.velocity * dt,
            0,
            Math.cos(this.heading + this.steerAngle) * this.velocity * dt
        ));

        this.position = frontWheel.clone().add(backWheel).multiplyScalar(0.5).add(new THREE.Vector3(
            Math.cos(this.heading) * (CAR_WIDTH / 2),
            0,
            -Math.sin(this.heading) * (CAR_WIDTH / 2)
        ));
        this.heading = Math.atan2(frontWheel.x - backWheel.x, frontWheel.z - backWheel.z);

        this.group.position.copy(this.position);
        this.group.rotation.z = -this.heading;
    }
    updateFromLog(logData) {
        if (!this.group) return;
        this.velocity = logData.velocity;
        this.acceleration = logData.acceleration;
        this.steerAngle = logData.steering_wheel_angle;
        this.position.set(logData.position[1], logData.position[2], logData.position[0]);
        this.heading = logData.rotation[1];
        this.group.position.copy(this.position);
        this.group.rotation.y = this.heading;
    }


    dispose() {
        if (!this.group) return;

        this.group.traverse((child) => {
            if (child.isMesh) {
                child.geometry.dispose();

                if (child.material) {
                    if (Array.isArray(child.material)) {
                        child.material.forEach((material) => {
                            material.map?.dispose();
                            material.normalMap?.dispose();
                            material.roughnessMap?.dispose();
                            material.dispose();
                        });
                    } else {
                        child.material.map?.dispose();
                        child.material.normalMap?.dispose();
                        child.material.roughnessMap?.dispose();
                        child.material.dispose();
                    }
                }
            }
        });

        this.group.removeFromParent();
        this.group = null;
    }
}

