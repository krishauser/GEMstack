"use client";

import React, { useState } from "react";
import { useTimelineStore } from "@/hooks/useTimelineStore";
import { RxCross2 } from "react-icons/rx";

export function VehicleInfoPanel({ time }: { time: number }) {
  const [isOpen, setIsOpen] = useState(true);
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
    <>
      <div
        className={`fixed top-0 right-0 w-76 max-h-screen bg-black/80 text-white shadow-lg transform transition-transform duration-300 ease-in-out z-40 p-4 overflow-y-auto text-sm ${
          isOpen ? "translate-x-0" : "translate-x-full"
        }`}
        onContextMenu={toggleOpen}
      >
        <div className="flex justify-between items-center mb-2">
          <h2 className="text-base font-semibold whitespace-nowrap">Vehicle State</h2>
          <button
            onClick={toggleOpen}
            className="text-white p-1 rounded hover:bg-white/10"
            title="Close Panel"
          >
            <RxCross2 className="w-4 h-4" />
          </button>
        </div>

        {/* Scrollable content */}
        <div className="overflow-x-auto pr-2">
          {current ? (
            <ul className="space-y-1 divide-y divide-white/10 min-w-[16rem]">
              {[
                ["Timestamp", current.time],
                ["X", current.x],
                ["Y", current.y],
                ["Z", current.z],
                ["Yaw", current.yaw],
                ["Pitch", current.pitch],
                ["Roll", current.roll],
              ].map(([label, value]) => (
                <li key={label} className="flex justify-between py-0.5 gap-4">
                  <span className="text-white/80 whitespace-nowrap">{label}:</span>
                  <span className="font-mono text-white text-right">{value}</span>
                </li>
              ))}

              {current.metadata &&
                Object.entries(current.metadata).map(([key, value]) => {
                  if (typeof value === "object") return null;
                  return (
                    <li key={key} className="flex justify-between py-0.5 gap-4">
                      <span className="text-white/70 whitespace-nowrap">
                        {key.replace(/_/g, " ")}:
                      </span>
                      <span className="font-mono text-white text-right">
                        {String(value)}
                      </span>
                    </li>
                  );
                })}
            </ul>
          ) : (
            <p className="text-sm italic text-white/70">
              No vehicle data at this time
            </p>
          )}
        </div>
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
    </>
  );
}
