"use client";

import React, { useEffect, useState } from "react";
import { useScrubber } from "./ScrubberContext";
import { IconButton, Slider, Menu, MenuItem } from "@mui/material";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PauseIcon from "@mui/icons-material/Pause";
import SpeedIcon from "@mui/icons-material/Speed";
import RefreshIcon from "@mui/icons-material/Refresh";

export const Scrubber2 = ({
    duration,
    startTime,
    loading,
}: {
    duration: number;
    startTime: number;
    loading: boolean;
}) => {
    const {
        setStartTime,
        currentTime,
        setCurrentTime,
        isPlaying,
        setIsPlaying,
    } = useScrubber();
    const frameRate = 30;
    const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
    const [selectedSpeed, setSelectedSpeed] = useState<number>(1);
    const speedOptions = [0.5, 1, 1.5, 2, 3];
    const open = Boolean(anchorEl);
    const [isDragging, setIsDragging] = useState<boolean>(false);

    const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
        setAnchorEl(event.currentTarget);
    };
    const handleClose = () => {
        setAnchorEl(null);
    };
    const handleMenuItemClick = (speed: number) => {
        setSelectedSpeed(speed);
        handleClose();
    };
    const handleSliderChange = (_: Event, newValue: number) => {
        setCurrentTime(newValue);
    };
    const handleSliderChangeCommitted = () => {
        setIsDragging(false);
    };
    const togglePlay = () => {
        setIsPlaying((prev: boolean) => !prev);
    };
    const restart = () => {
        setCurrentTime(0);
    };
    const formatDuration = (time: number) => {
        const minutes = Math.floor(time / 60);
        const seconds = Math.floor(time % 60);
        return `${minutes < 10 ? "0" : ""}${minutes}:${
            seconds < 10 ? "0" : ""
        }${seconds}`;
    };
    const handleContextMenu = (event: React.MouseEvent) => {
        event.preventDefault();
    };
    useEffect(() => {
        if (!isPlaying) return;
        const interval = setInterval(() => {
            setCurrentTime((prev: number) =>
                Math.min(prev + (selectedSpeed * frameRate) / 1000, duration)
            );
        }, 1000 / frameRate);
        return () => clearInterval(interval);
    }, [isPlaying, duration, selectedSpeed]);

    useEffect(() => {
        setStartTime(startTime);
        setCurrentTime(0);
    }, [startTime]);

    useEffect(() => {
        if (currentTime >= duration) {
            setIsPlaying(false);
        }
    }, [currentTime, duration]);

    useEffect(() => {
        if (loading) {
            setIsPlaying(false);
        }
    }, [loading]);

    return (
        <div
            className="px-4 py-2 fixed bottom-4 left-1/6 w-2/3 bg-black/80 text-white shadow-md rounded-full flex items-center justify-between"
            onContextMenu={handleContextMenu}
        >
            <IconButton
                size="small"
                onClick={togglePlay}
                sx={{ color: "white", mr: 1 }}
                loading={loading}
            >
                {isPlaying ? (
                    <PauseIcon fontSize="small" />
                ) : (
                    <PlayArrowIcon fontSize="small" />
                )}
            </IconButton>
            <div className="w-full flex items-center mx-2 text-sm">
                <span className="min-w-[36px] text-center">
                    {currentTime < duration
                        ? formatDuration(currentTime)
                        : formatDuration(duration)}
                </span>
                <Slider
                    key={isDragging ? undefined : Math.floor(currentTime * 100)}
                    min={0}
                    max={duration}
                    step={0.02}
                    className="mx-3"
                    disabled={duration === 0}
                    onChange={handleSliderChange}
                    onMouseDown={() => setIsDragging(true)}
                    onChangeCommitted={handleSliderChangeCommitted}
                    value={currentTime}
                />
                <span className="min-w-[36px] text-center">{formatDuration(duration)}</span>
            </div>
            <div className="flex items-center">
                <IconButton
                    size="small"
                    onClick={handleClick}
                    sx={{ color: "white" }}
                >
                    <SpeedIcon fontSize="small" />
                </IconButton>
                <Menu
                    anchorEl={anchorEl}
                    open={open}
                    onClose={handleClose}
                    anchorOrigin={{
                        vertical: "top",
                        horizontal: "center",
                    }}
                    transformOrigin={{
                        vertical: "bottom",
                        horizontal: "center",
                    }}
                >
                    {speedOptions.map((speed) => (
                        <MenuItem
                            key={speed}
                            selected={selectedSpeed === speed}
                            onClick={() => handleMenuItemClick(speed)}
                        >
                            {speed}x
                        </MenuItem>
                    ))}
                </Menu>
                <IconButton
                    size="small"
                    sx={{ color: "white", ml: 1 }}
                >
                    <RefreshIcon fontSize="small" onClick={restart} />
                </IconButton>
            </div>
        </div>
    );
};
