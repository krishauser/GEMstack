"use client"

import {Canvas, useThree} from "@react-three/fiber"
import { OrbitControls, Environment } from "@react-three/drei"
import { useLoader } from "@react-three/fiber"
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js"

const urdfParts = [
    { name: "base_link", url: "/car/base_link.STL", position: [0, 0, 0], rotation: [0,0,0], color: "#e0e0e0" },
    { name: "chair_link", url: "/car/chair_link.STL", position: [0.001,0,-0.02], rotation:[0,0,0], color:"#888888" },
    { name: "door_link", url: "/car/door_link.STL", position: [0.001,0,-0.015], rotation:[0,0,0], color:"#e0e0e0" },
    { name:"top_rack_link", url:"/car/top_rack_link.STL", position:[-0.10172,0.6575,1.3921], rotation:[4.71,0,Math.PI], color:"#000" },
    // { name:"front_rack_link", url:"/car/front_rack_link.STL", position:[1.352,0,-0.30594], rotation:[Math.PI/2,0,Math.PI/2], color:"#000" },
    // { name:"rear_rack_link", url:"/car/rear_rack_link.STL", position:[-1.302,0,-0.27168], rotation:[Math.PI/2,0,-Math.PI/2], color:"#000" },
    { name:"front_right_head_light_link", url:"/car/front_right_head_light_link.STL", position:[0.755,-0.5,0.58706], rotation:[0,0,0], color:"#fff" },
    { name:"front_left_head_light_link", url:"/car/front_left_head_light_link.STL", position:[0.755,0.5,0.58706], rotation:[0,0,0], color:"#fff" },
    { name:"front_right_turn_light_link", url:"/car/front_right_turn_light_link.STL", position:[0.765,-0.345,0.58706], rotation:[0,0,0], color:"#ffa500" },
    { name:"front_left_turn_light_link", url:"/car/front_left_turn_light_link.STL", position:[0.765,0.345,0.58706], rotation:[0,0,0], color:"#ffa500" },
    { name:"rear_right_light_link", url:"/car/rear_right_light_link.STL", position:[-1.195,-0.32,0.025063], rotation:[0,0,0], color:"#f00" },
    { name:"rear_left_light_link", url:"/car/rear_left_light_link.STL", position:[-1.195,0.32,0.025063], rotation:[0,0,0], color:"#f00" },
    { name:"rear_left_stop_light_link", url:"/car/rear_left_stop_light_link.STL", position:[-1.195,0.38,0.24506], rotation:[0,0,Math.PI], color:"#f00" },
    { name:"rear_right_stop_light_link", url:"/car/rear_right_stop_light_link.STL", position:[-1.195,-0.38,0.24506], rotation:[0,0,Math.PI], color:"#f00" },
    { name:"right_blue_outer_link", url:"/car/right_blue_outer_link.STL", position:[0.002,-0.6745,0.23435], rotation:[0,0,0], color:"#00f" },
    { name:"right_I_link", url:"/car/right_I_link.STL", position:[0.002,-0.6745,0.23435], rotation:[0,0,0], color:"#ffa500" },
    { name:"left_blue_outer_link", url:"/car/left_blue_outer_link.STL", position:[0.002,0.6645,0.23435], rotation:[0,0,0], color:"#00f" },
    { name:"left_I_link", url:"/car/left_I_link.STL", position:[0.002,0.6645,0.23435], rotation:[0,0,0], color:"#ffa500" },
    { name:"right_antenna_link", url:"/car/right_antenna_link.STL", position:[-0.23472,-0.6125,1.5071], rotation:[0,0,0], color:"#888" },
    { name:"left_antenna_link", url:"/car/left_antenna_link.STL", position:[-0.23472,0.6125,15071], rotation:[0,0,0], color:"#888" },
    { name:"rear_left_emergency_button_link", url:"/car/rear_left_emergency_button_link.STL", position:[-0.72012,0.6645,0.83815], rotation:[0,0,0], color:"#f00" },
    { name:"rear_right_emergency_button_link", url:"/car/rear_right_emergency_button_link.STL", position:[-0.72012,-0.6645,0.83815], rotation:[0,0.57871,Math.PI], color:"#f00" },
    { name:"front_left_emergency_button_link", url:"/car/front_left_emergency_button_link.STL", position:[1.1497,0.6645,0.20492], rotation:[0,0,0], color:"#f00" },
    { name:"front_right_emergency_button_link", url:"/car/front_right_emergency_button_link.STL", position:[1.1497,-0.6645,0.20492], rotation:[0,0.57871,Math.PI], color:"#f00" },
    // { name:"front_camera_link", url:"/car/front_camera_link.STL", position:[0.16,-0.11,1.1063], rotation:[Math.PI/2,0,-Math.PI], color:"#000" },
    // { name:"rear_light_bar_link", url:"/car/rear_light_bar_link.STL", position:[-0.64921,0,0.76944], rotation:[-1.9138,0,Math.PI/2], color:"#f00" },
    // Wheels
    { name:"front_left_wheel_link", url:"/car/front_left_wheel_link.STL", position:[0.88,0.6,-0.151], rotation:[0,0,0], color:"#121212" },
    { name:"front_right_wheel_link", url:"/car/front_right_wheel_link.STL", position:[0.88,-0.6,-0.151], rotation:[0,0,0], color:"#121212" },
    { name:"rear_left_wheel_link", url:"/car/rear_left_wheel_link.STL", position:[-0.87,0.6,-0.151], rotation:[0,0,0], color:"#121212" },
    { name:"rear_right_wheel_link", url:"/car/rear_right_wheel_link.STL", position:[-0.87,-0.6,-0.151], rotation:[0,0,0], color:"#121212" },
]

interface PartProps {
    url: string
    position?: [number, number, number]
    rotation?: [number, number, number]
    scale?: number
    color?: string
}

function Part({ url, position = [0, 0, 0], rotation = [0, 0, 0], scale = 1, color = "#888" }: PartProps) {
    const geometry = useLoader(STLLoader, url)
    return (
        <mesh geometry={geometry} position={position} rotation={rotation} scale={scale} castShadow receiveShadow>
            <meshStandardMaterial color={color} metalness={0.6} roughness={0.4} />
        </mesh>
    )
}

function ResponsiveGroup({ children }: { children: React.ReactNode }) {
    const { size } = useThree()
    const scaleBase = 1.9
    const scaleMin = 1.3
    const maxWidth = 1024
    const t = Math.min(size.width, maxWidth) / maxWidth
    const scale = scaleMin + (scaleBase - scaleMin) * t

    return (
        <group position={[0, -0.7, 0]} rotation={[-Math.PI / 2, 0, 0]} scale={scale}>
            {children}
        </group>
    )
}


export function CarModel() {
    return (
        <Canvas shadows>
            <ambientLight intensity={0.5}/>
            <spotLight position={[10, 10, 10]} angle={0.15} penumbra={1} castShadow/>

            <ResponsiveGroup>
                {urdfParts.map(({name, url, position, rotation, color}) => (
                    <Part
                        key={name}
                        url={url}
                        position={position as [number, number, number]}
                        rotation={rotation as [number, number, number]}
                        color={color}
                    />
                ))}
            </ResponsiveGroup>
                <Environment preset="city"/>
                <OrbitControls
                    enableZoom={false}
                />
        </Canvas>
)
}