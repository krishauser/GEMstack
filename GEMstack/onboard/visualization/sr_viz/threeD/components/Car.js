import * as THREE from 'three';

const TIRE_RADIUS = 0.4;
const TIRE_WIDTH = 0.5;
const BODY_X = 2;
const BODY_Y = 1;
const BODY_Z = 4;
const UPPER_BODY_COLOR = 0xffffff;
const UPPER_BODY_X = 1.6;
const UPPER_BODY_Y = 0.8;
const UPPER_BODY_Z = 2.2;
const UPPER_BODY_Z_OFFSET = -0.2;
const WINDOWS_COLOR = 0xaaaaaa;
const WINDOWS_THICKNESS = 0.05;
const SIDE_WINDOWS_X = WINDOWS_THICKNESS;
const WINDOWS_Y = 0.6;
const FRONT_SIDE_WINDOWS_Z = UPPER_BODY_Z / 11 * 5;
const BACK_SIDE_WINDOWS_Z = UPPER_BODY_Z / 11 * 3;
const FRONT_BACK_WINDOWS_X = 1.3;
const FRONT_BACK_WINDOWS_Z = WINDOWS_THICKNESS;
export const CAR_HEIGHT = TIRE_RADIUS + BODY_Y + UPPER_BODY_Y;

export default class Car {
  constructor(color = 0xff0000, position = { x: 0, z: 0 }, direction = 0) {
    this.group = new THREE.Group();
    this.color = color;
    this.createBody(color);
    this.createWindows();
    this.createWheels();
    // this.createDoors();
    this.group.position.set(position.x, 0, position.z);
    this.group.rotation.y = direction;
  }

  createBody(color) {
    const bodyGeometry = new THREE.BoxGeometry(BODY_X, BODY_Y, BODY_Z);
    const bodyMaterial = new THREE.MeshStandardMaterial({ color });
    this.body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    this.body.position.y = TIRE_RADIUS + (BODY_Y / 2);
    const upperBodyGeometry = new THREE.BoxGeometry(UPPER_BODY_X, UPPER_BODY_Y, UPPER_BODY_Z);
    const upperBodyMaterial = new THREE.MeshStandardMaterial({ color: UPPER_BODY_COLOR });
    this.upperbody = new THREE.Mesh(upperBodyGeometry, upperBodyMaterial);
    this.upperbody.position.set(0, TIRE_RADIUS + BODY_Y + UPPER_BODY_Y / 2, UPPER_BODY_Z_OFFSET);
    this.group.add(this.body);
    this.group.add(this.upperbody);
  }

  createWindows() {
    const FrontSideWindowGeometry = new THREE.BoxGeometry(SIDE_WINDOWS_X, WINDOWS_Y, FRONT_SIDE_WINDOWS_Z);
    const windowMaterial = new THREE.MeshStandardMaterial({ color: WINDOWS_COLOR, transparent: true, opacity: 0.5 });
    this.frontDriverSideWindows = new THREE.Mesh(FrontSideWindowGeometry, windowMaterial);
    this.frontDriverSideWindows.position.set((UPPER_BODY_X + SIDE_WINDOWS_X) / 2, TIRE_RADIUS + BODY_Y + UPPER_BODY_Y / 2, UPPER_BODY_Z_OFFSET + (UPPER_BODY_Z / 2 - UPPER_BODY_Z / 11 - FRONT_SIDE_WINDOWS_Z / 2));
    this.group.add(this.frontDriverSideWindows);
    this.frontPassengerSideWindows = new THREE.Mesh(FrontSideWindowGeometry, windowMaterial);
    this.frontPassengerSideWindows.position.set(-(UPPER_BODY_X + SIDE_WINDOWS_X) / 2, TIRE_RADIUS + BODY_Y + UPPER_BODY_Y / 2, UPPER_BODY_Z_OFFSET + (UPPER_BODY_Z / 2 - UPPER_BODY_Z / 11 - FRONT_SIDE_WINDOWS_Z / 2));
    this.group.add(this.frontPassengerSideWindows);
    const BackSideWindowGeometry = new THREE.BoxGeometry(SIDE_WINDOWS_X, WINDOWS_Y, BACK_SIDE_WINDOWS_Z);
    this.backDriverSideWindows = new THREE.Mesh(BackSideWindowGeometry, windowMaterial);
    this.backDriverSideWindows.position.set((UPPER_BODY_X + SIDE_WINDOWS_X) / 2, TIRE_RADIUS + BODY_Y + UPPER_BODY_Y / 2, UPPER_BODY_Z_OFFSET - (UPPER_BODY_Z / 2 - UPPER_BODY_Z / 11 - BACK_SIDE_WINDOWS_Z / 2));
    this.group.add(this.backDriverSideWindows);
    this.backPassengerSideWindows = new THREE.Mesh(BackSideWindowGeometry, windowMaterial);
    this.backPassengerSideWindows.position.set(-(UPPER_BODY_X + SIDE_WINDOWS_X) / 2, TIRE_RADIUS + BODY_Y + UPPER_BODY_Y / 2, UPPER_BODY_Z_OFFSET - (UPPER_BODY_Z / 2 - UPPER_BODY_Z / 11 - BACK_SIDE_WINDOWS_Z / 2));
    this.group.add(this.backPassengerSideWindows);
    const FrontWindowGeometry = new THREE.BoxGeometry(FRONT_BACK_WINDOWS_X, WINDOWS_Y, FRONT_BACK_WINDOWS_Z);
    this.frontWindow = new THREE.Mesh(FrontWindowGeometry, windowMaterial);
    this.frontWindow.position.set(0, TIRE_RADIUS + BODY_Y + UPPER_BODY_Y / 2, UPPER_BODY_Z_OFFSET + (UPPER_BODY_Z + FRONT_BACK_WINDOWS_Z) / 2);
    this.group.add(this.frontWindow);
    const BackWindowGeometry = new THREE.BoxGeometry(FRONT_BACK_WINDOWS_X, WINDOWS_Y, FRONT_BACK_WINDOWS_Z);
    this.backWindow = new THREE.Mesh(BackWindowGeometry, windowMaterial);
    this.backWindow.position.set(0, TIRE_RADIUS + BODY_Y + UPPER_BODY_Y / 2, UPPER_BODY_Z_OFFSET - (UPPER_BODY_Z + FRONT_BACK_WINDOWS_Z) / 2);
    this.group.add(this.backWindow);
  }

  createWheels() {
    const wheelGeometry = new THREE.CylinderGeometry(TIRE_RADIUS, TIRE_RADIUS, TIRE_WIDTH, 16);
    const wheelMaterial = new THREE.MeshStandardMaterial({ color: 0x000000 });

    const createWheel = (x, z) => {
      const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
      wheel.rotation.z = Math.PI / 2;
      wheel.position.set(x, TIRE_RADIUS, z);
      this.group.add(wheel);
    };
    const wheelOffset_X = (BODY_X - TIRE_WIDTH / 2) / 2;
    const wheelOffset_Z = (BODY_Z - 3 * TIRE_WIDTH) / 2;
    createWheel(wheelOffset_X, wheelOffset_Z);
    createWheel(wheelOffset_X, -wheelOffset_Z);
    createWheel(-wheelOffset_X, wheelOffset_Z);
    createWheel(-wheelOffset_X, -wheelOffset_Z);
  }

  createDoors() {
    const doorGeometry = new THREE.BoxGeometry(0.02, 0.8, 1.5);
    const doorMaterial = new THREE.MeshStandardMaterial({ color: this.color });

    const leftDoor = new THREE.Mesh(doorGeometry, doorMaterial);
    leftDoor.position.set(-1, 0.5, 0);
    this.group.add(leftDoor);

    const rightDoor = new THREE.Mesh(doorGeometry, doorMaterial);
    rightDoor.position.set(1, 0.5, 0);
    this.group.add(rightDoor);
  }
}
