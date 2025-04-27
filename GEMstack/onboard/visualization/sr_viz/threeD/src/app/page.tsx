"use client";

import { useSearchParams } from "next/navigation";
import ControlPanel from "@/components/ControlPanel";
import CanvasWrapper from "@/components/CanvasWrapper";
import Scrubber from "@/components/Scrubber";
import { usePlaybackTime } from "@/hooks/usePlaybackTime";

export default function HomePage() {
  const {
    time,
    reset,
    restart,
    play,
    togglePlay,
    speed,
    setPlaybackSpeed,
    moveToTime,
    duration,
    setDuration,
  } = usePlaybackTime();
  const searchParams = useSearchParams();
  const folder = searchParams.get("folder") || undefined;
  const file = searchParams.get("file") || undefined;
  return (
    <main className="relative w-screen h-screen bg-white">
      <ControlPanel reset={reset} folder={folder} file={file} />
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
    </main>
  );
}
