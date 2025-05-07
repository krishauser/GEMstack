"use client";
import React, { useState } from "react";
import { useTimelineStore } from "@/hooks/useTimelineStore";
import { RxCross2 } from "react-icons/rx";

export function VehicleInfoPanel({ time }: { time: number }) {
  const [isOpen, setIsOpen] = useState(false);
  const frames = useTimelineStore((s) => s.vehicle);

  const toggleOpen = (e?: React.MouseEvent) => {
    if (e) e.preventDefault();
    setIsOpen((open) => !open);
  };

  if (frames.length === 0) return null;

  const startTimestamp = frames[0].time;
  const currentTimestamp = startTimestamp + time;
  const current = frames.filter((f) => f.time <= currentTimestamp).pop();

  return (
    <React.Fragment>
      <div
        className={`fixed top-0 right-0 w-64 bg-black/80 text-white shadow-lg transform transition-transform duration-500 ease-in-out z-40 p-4 ${
          isOpen ? "translate-x-0" : "translate-x-full"
        }`}
        onContextMenu={toggleOpen}
      >
        {isOpen && (
          <>
            <div className="flex justify-between items-center mb-2">
              <h2 className="text-lg font-semibold">Current Vehicle State</h2>
              <button
                onClick={toggleOpen}
                className="text-white p-1 rounded hover:bg-white/10"
                title="Close Panel"
              >
                <RxCross2 className="w-4 h-4" />
              </button>
            </div>
            {current ? (
              <ul className="space-y-1 text-sm">
                <li>
                  <span className="font-medium">Timestamp:</span> {current.time}
                </li>
                <li>
                  <span className="font-medium">X:</span> {current.x}
                </li>
                <li>
                  <span className="font-medium">Y:</span> {current.y}
                </li>
                <li>
                  <span className="font-medium">Z:</span> {current.z}
                </li>
                <li>
                  <span className="font-medium">Yaw:</span> {current.yaw}
                </li>
                <li>
                  <span className="font-medium">Pitch:</span> {current.pitch}
                </li>
                <li>
                  <span className="font-medium">Roll:</span> {current.roll}
                </li>
              </ul>
            ) : (
              <p className="text-sm italic">No vehicle data at this time</p>
            )}
          </>
        )}
      </div>
      {!isOpen && (
        <button
          onClick={toggleOpen}
          className="fixed top-[15%] right-0 z-50 bg-black/80 text-white w-3 h-8 flex items-center justify-center rounded-l border-r border-white/20"
          title="Open Vehicle Panel"
          onContextMenu={(e) => e.preventDefault()}
        >
          <span className="text-xs">⟨</span>
        </button>
      )}
    </React.Fragment>
  );
}
