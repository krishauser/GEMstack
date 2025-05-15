"use client";

import React from "react";
import { groundConfig } from "@/config/groundConfig";
import { Grid } from "@react-three/drei";

export default function Ground() {
  return (
    <Grid
      args={groundConfig.size}
      position={groundConfig.position}
      rotation={groundConfig.rotation}
      cellSize={groundConfig.cellSize}
      cellThickness={groundConfig.cellThickness}
      cellColor={groundConfig.cellColor}
      sectionSize={groundConfig.sectionSize}
      sectionThickness={groundConfig.sectionThickness}
      sectionColor={groundConfig.sectionColor}
      fadeDistance={groundConfig.fadeDistance}
      fadeStrength={groundConfig.fadeStrength}
      infiniteGrid={groundConfig.infiniteGrid}
    />
  );
}
