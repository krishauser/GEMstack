"use client";

import { Suspense } from "react";
import dynamic from "next/dynamic";
import { useSearchParams } from "next/navigation";
import ControlPanel from "@/components/ControlPanel";
import Scrubber from "@/components/Scrubber";
import { usePlaybackTime } from "@/hooks/usePlaybackTime";
import { useState, useEffect } from "react";

const CanvasWrapper = dynamic(() => import("@/components/CanvasWrapper"), {
  ssr: false,
});

function InnerHomePage() {
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

  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return null;
  }

  return (
    <>
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
    </>
  );
}

export default function HomePage() {
  return (
    <main className="relative w-screen h-screen bg-white">
      <Suspense fallback={<div>Loading Canvas...</div>}>
        <InnerHomePage />
      </Suspense>
    </main>
  );
}
