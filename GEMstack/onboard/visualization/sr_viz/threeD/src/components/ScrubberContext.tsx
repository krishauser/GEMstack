"use client";

import React, { createContext, useContext, useState } from "react";

const ScrubberContext = createContext<any>(null);

export const ScrubberProvider = ({
    children,
}: {
    children: React.ReactNode;
}) => {
    const [startTime, setStartTime] = useState<number>(0);
    const [currentTime, setCurrentTime] = useState<number>(0);
    const [isPlaying, setIsPlaying] = useState(false);

    return (
        <ScrubberContext.Provider
            value={{
                startTime,
                setStartTime,
                currentTime,
                setCurrentTime,
                isPlaying,
                setIsPlaying,
            }}
        >
            {children}
        </ScrubberContext.Provider>
    );
};

export const useScrubber = () => useContext(ScrubberContext);
