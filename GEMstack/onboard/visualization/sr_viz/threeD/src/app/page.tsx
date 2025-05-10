"use client";

import { useEffect, useState } from "react";
import ControlPanel from "@/components/ControlPanel";
import CanvasWrapper from "@/components/CanvasWrapper";
import Scrubber from "@/components/Scrubber";
import { VehicleInfoPanel } from "@/components/VehicleInfoPanel";
import { usePlaybackTime } from "@/hooks/usePlaybackTime";
import { useRouter } from "next/navigation";
import Button from "@mui/material/Button";
import PageviewIcon from "@mui/icons-material/Pageview";

export default function HomePage() {
  const router = useRouter();
  const {
    time,
    reset,
    restart,
    play,
    setPlay,
    togglePlay,
    speed,
    setPlaybackSpeed,
    moveToTime,
    duration,
    setDuration,
  } = usePlaybackTime();

  const [searchParams, setSearchParams] = useState<{
    folder?: string;
    file?: string;
  }>({});

  useEffect(() => {
    if (typeof window !== "undefined") {
      const params = new URLSearchParams(window.location.search);
      const folder = params.get("folder") || undefined;
      const file = params.get("file") || undefined;
      setSearchParams({ folder, file });
    }
  }, []);

  const handleRedirect = () => {
    setPlay(false);
    router.replace("/rosbagViewer");
  };

  return (
    <main className="relative w-screen h-screen bg-white">
      <ControlPanel
        reset={reset}
        folder={searchParams.folder}
        file={searchParams.file}
      />
      <CanvasWrapper time={time} setDuration={setDuration} />
      <Scrubber
        time={time}
        play={play}
        togglePlay={togglePlay}
        restart={restart}
        setPlaybackSpeed={setPlaybackSpeed}
        moveToTime={moveToTime}
        duration={duration}
      />
      <VehicleInfoPanel time={time} />
    </main>
  );
}
