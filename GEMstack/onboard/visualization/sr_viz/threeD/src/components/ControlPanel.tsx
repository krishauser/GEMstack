"use client";

import React, { useEffect, useState } from "react";
import { TiUpload } from "react-icons/ti";
import { RxCross2 } from "react-icons/rx";
import { buildTimeline } from "@/utils/buildTimeline";
import { parseLogFile } from "@/utils/parseLogFile";
import { useTimelineStore } from "@/hooks/useTimelineStore";
import { TimelineData } from "@/types/TimelineData";

export default function ControlPanel({
  reset,
  folder,
  file,
}: {
  reset: () => void;
  folder?: string;
  file?: string;
}) {
  const [isOpen, setIsOpen] = useState(false);
  const [fileName, setFileName] = useState<string | null>(null);
  const setTimeline = useTimelineStore((state) => state.setTimeline);

  const handleFileUpload = async (
    event: React.ChangeEvent<HTMLInputElement>
  ) => {
    const file = event.target.files?.[0];
    if (!file) return;

    setFileName(file.name);

    try {
      const entries = await parseLogFile(file);
      const timeline: TimelineData = buildTimeline(entries);
      setTimeline(timeline);
      reset();
      console.log("timeline loaded:", timeline);
    } catch (err) {
      console.error("Failed to parse log file:", err);
    }
  };
  const handleContextMenu = (event: React.MouseEvent) => {
    event.preventDefault();
    if (isOpen) {
      setIsOpen(false);
    } else {
      setIsOpen(true);
    }
  }

  useEffect(() => {
    if (!folder || !file) return;

    const fetchLog = async () => {
      const url = `http://localhost:5000/raw_logs/${encodeURIComponent(
        folder
      )}/${encodeURIComponent(file)}`;

      try {
        const res = await fetch(url);
        if (!res.ok) {
          throw new Error(
            `Failed to fetch ${file} from ${url} (status ${res.status})`
          );
        }

        const text = await res.text();
        const fakeFile = new File([text], file, { type: "text/plain" });

        const entries = await parseLogFile(fakeFile);
        const timeline = buildTimeline(entries);

        setTimeline(timeline);
        reset();
        setFileName(file);

        console.log("[✔] Timeline loaded from Flask API:", timeline);
      } catch (err) {
        console.error("[✘] Failed to load remote log file:", err);
      }
    };

    fetchLog();
  }, [folder, file]);

  return (
    <>
      <div
        className={`fixed top-0 left-0 h-full w-64 bg-black/80 text-white shadow-lg transform transition-transform duration-500 ease-in-out z-40 ${
          isOpen ? "translate-x-0" : "-translate-x-full"
        }`}
        onContextMenu={handleContextMenu}
      >
        {isOpen && (
          <>
            <div className="flex items-center justify-between px-4 py-3 border-b border-white/20">
              <h2 className="text-lg font-semibold">Control Panel</h2>
              <button
                onClick={() => setIsOpen(false)}
                className="text-white p-1 rounded hover:bg-white/10"
                title="Close Panel"
              >
                <RxCross2 className="w-4 h-4" />
              </button>
            </div>

            <div className="p-4">
              <label className="inline-flex items-center gap-2 cursor-pointer text-gray-300 text-sm px-4 py-2 border border-gray-500 rounded hover:bg-white/10 transition">
                <TiUpload className="w-4 h-4" />
                <span>Choose file</span>
                <input
                  type="file"
                  accept=".json,.txt,.log"
                  className="hidden"
                  onChange={handleFileUpload}
                />
              </label>
              <p className="mt-2 text-xs text-gray-400 truncate">
                {fileName ? `${fileName}` : "No file loaded"}
              </p>
            </div>

            <div className="px-4 text-xs text-gray-400 border-t border-white/10 pt-4">
              <p>🚧 Feature toggles coming soon...</p>
            </div>
          </>
        )}
      </div>

      {!isOpen && (
        <button
          onClick={() => setIsOpen(true)}
          className="fixed top-1/2 left-0 -translate-y-1/2 z-50 bg-black/80 text-white w-3 h-8 flex items-center justify-center rounded-r hover:bg-black border-l border-white/20"
          title="Open Panel"
          onContextMenu={(e) => e.preventDefault()}
        >
          <span className="text-xs">⟩</span>
        </button>
      )}
    </>
  );
}
