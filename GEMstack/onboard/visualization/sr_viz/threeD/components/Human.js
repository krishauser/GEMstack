import * as THREE from "three";
import { CAR_HEIGHT } from "./Car.js";
import { HUMAN_BODY_RATIOS, HUMAN_TO_CAR_HEIGHT_RATIO } from "../utils/constants.js";

const HUMAN_HEIGHT = CAR_HEIGHT * HUMAN_TO_CAR_HEIGHT_RATIO;
const HUMAN_BODY_LENGTH = HUMAN_HEIGHT * HUMAN_BODY_RATIOS.body;
const HUMAN_HEAD_RADIUS = HUMAN_HEIGHT * HUMAN_BODY_RATIOS.head;
const HUMAN_HEAD_COLOR = 0xffcc99;
const HUMAN_ARM_LENGTH = HUMAN_HEIGHT * HUMAN_BODY_RATIOS.arm;
const HUMAN_LEG_LENGTH = HUMAN_HEIGHT * HUMAN_BODY_RATIOS.leg;
const HUMAN_LEG_COLOR = 0x555555;

export default class Human {
    constructor(color = 0x00aaff, position = { x: 0, y: 0, z: 0 }) {
        this.group = new THREE.Group();

        const bodyGeometry = new THREE.BoxGeometry(HUMAN_BODY_LENGTH / 1.2, HUMAN_BODY_LENGTH, HUMAN_BODY_LENGTH / 1.6);
        const bodyMaterial = new THREE.MeshStandardMaterial({ color });
        this.body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        this.body.position.y = HUMAN_LEG_LENGTH + (HUMAN_BODY_LENGTH / 2);
        this.group.add(this.body);

        const headGeometry = new THREE.SphereGeometry(HUMAN_HEAD_RADIUS);
        const headMaterial = new THREE.MeshStandardMaterial({ color: HUMAN_HEAD_COLOR });
        this.head = new THREE.Mesh(headGeometry, headMaterial);
        this.head.position.y = HUMAN_LEG_LENGTH + HUMAN_BODY_LENGTH + HUMAN_HEAD_RADIUS;
        this.group.add(this.head);

        const armGeometry = new THREE.CylinderGeometry(HUMAN_ARM_LENGTH / 6, HUMAN_ARM_LENGTH / 6, HUMAN_ARM_LENGTH);
        const armMaterial = new THREE.MeshStandardMaterial({ color });
        this.leftArm = new THREE.Mesh(armGeometry, armMaterial);
        this.rightArm = new THREE.Mesh(armGeometry, armMaterial);
        const armX = (HUMAN_BODY_LENGTH / 1.2) / 2 + HUMAN_ARM_LENGTH / 6;
        const armY = HUMAN_LEG_LENGTH + HUMAN_BODY_LENGTH - HUMAN_ARM_LENGTH / 1.7;
        this.leftArm.position.set(armX, armY, 0);
        this.rightArm.position.set(-armX, armY, 0);
        this.group.add(this.leftArm, this.rightArm);

        const legGeometry = new THREE.CylinderGeometry(HUMAN_LEG_LENGTH / 6, HUMAN_LEG_LENGTH / 6, HUMAN_LEG_LENGTH);
        const legMaterial = new THREE.MeshStandardMaterial({ color: HUMAN_LEG_COLOR });
        this.leftLeg = new THREE.Mesh(legGeometry, legMaterial);
        this.rightLeg = new THREE.Mesh(legGeometry, legMaterial);
        this.leftLeg.position.set(-0.2, HUMAN_LEG_LENGTH / 2, 0);
        this.rightLeg.position.set(0.2, HUMAN_LEG_LENGTH / 2, 0);
        this.group.add(this.leftLeg, this.rightLeg);

        this.group.position.set(position.x, position.y, position.z);

        this.walkingSpeed = 0.03;
        this.walkingPhase = 0;
    }

    walk() {
        this.walkingPhase += this.walkingSpeed;

        const swingAmount = Math.PI / 6;

        this.leftArm.rotation.x = Math.sin(this.walkingPhase) * swingAmount;
        this.rightArm.rotation.x = -Math.sin(this.walkingPhase) * swingAmount;

        this.leftLeg.rotation.x = -Math.sin(this.walkingPhase) * swingAmount;
        this.rightLeg.rotation.x = Math.sin(this.walkingPhase) * swingAmount;
    }
}
