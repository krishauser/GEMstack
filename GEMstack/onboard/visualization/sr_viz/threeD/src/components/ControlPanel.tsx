"use client";

import React, { useRef, useState, useEffect } from "react";
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
  const fileInputRef = useRef<HTMLInputElement>(null);
  const prevKeyRef = useRef<string | null>(null);
  const setTimeline = useTimelineStore((state) => state.setTimeline);

  const handleFileUpload = async (
    event: React.ChangeEvent<HTMLInputElement>
  ) => {
    const uploaded = event.target.files?.[0];
    if (!uploaded) return;
    setFileName(uploaded.name);
    const entries = await parseLogFile(uploaded);
    const timeline: TimelineData = buildTimeline(entries);
    setTimeline(timeline);
    reset();
    fileInputRef.current!.value = "";
  };

  useEffect(() => {
    if (!folder || !file) return;
    const key = `${folder}/${file}`;
    if (prevKeyRef.current === key) return;
    prevKeyRef.current = key;
    (async () => {
      const url = `http://localhost:5000/raw_logs/${encodeURIComponent(
        folder
      )}/${encodeURIComponent(file)}`;
      const res = await fetch(url);
      const text = await res.text();
      const fakeFile = new File([text], file, { type: "text/plain" });
      const entries = await parseLogFile(fakeFile);
      const timeline = buildTimeline(entries);
      setTimeline(timeline);
      reset();
      setFileName(file);
    })();
  }, [folder, file, reset, setTimeline]);

  return (
    <>
      <div
        className={`fixed top-0 left-0 h-full w-48 max-w-[90vw] bg-black/80 text-white shadow-lg transform transition-transform duration-300 ease-in-out z-40 overflow-y-auto flex flex-col ${
          isOpen ? "translate-x-0" : "-translate-x-full"
        }`}
        onContextMenu={(e) => {
          e.preventDefault();
          setIsOpen((o) => !o);
        }}
      >
        {isOpen && (
          <>
            <div className="flex items-center justify-between px-4 py-3 border-b border-white/20">
              <h2 className="text-base font-semibold whitespace-nowrap">
                Control Panel
              </h2>
              <button
                onClick={() => setIsOpen(false)}
                className="text-white p-1 rounded hover:bg-white/10"
              >
                <RxCross2 className="w-4 h-4" />
              </button>
            </div>
            <div className="p-4">
              <label className="inline-flex items-center gap-2 cursor-pointer text-gray-300 text-sm px-4 py-2 border border-gray-500 rounded hover:bg-white/10 transition">
                <TiUpload className="w-4 h-4" />
                <span>Choose file</span>
                <input
                  ref={fileInputRef}
                  type="file"
                  accept=".json,.txt,.log"
                  className="hidden"
                  onChange={handleFileUpload}
                />
              </label>
              <p className="mt-2 text-xs text-gray-400 truncate">
                {fileName ?? "No file loaded"}
              </p>
            </div>
          </>
        )}
      </div>
      {!isOpen && (
        <button
          onClick={() => setIsOpen(true)}
          className="fixed top-1/2 left-0 -translate-y-1/2 z-50 bg-black/80 text-white w-3 h-8 flex items-center justify-center rounded-r hover:bg-black border-l border-white/20"
        >
          <span className="text-xs">⟩</span>
        </button>
      )}
    </>
  );
}
