"use client";
import React, { useState, useRef } from "react";
import { IconButton, Slider, Menu, MenuItem } from "@mui/material";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PauseIcon from "@mui/icons-material/Pause";
import SpeedIcon from "@mui/icons-material/Speed";
import RefreshIcon from "@mui/icons-material/Refresh";

export default function Scrubber({
    time,
    play,
    togglePlay,
    restart,
    setPlaybackSpeed,
    moveToTime,
    duration,
}: {
    time: number;
    play: boolean;
    togglePlay: () => void;
    restart: () => void;
    setPlaybackSpeed: (speed: number) => void;
    moveToTime: (time: number) => void;
    duration: number;
}) {
    const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
    const [selectedSpeed, setSelectedSpeed] = useState<number>(1);
    const speedOptions = [0.5, 1, 1.5, 2, 3];
    const open = Boolean(anchorEl);
    const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
        setAnchorEl(event.currentTarget);
    };
    const handleClose = () => {
        setAnchorEl(null);
    };
    const handleMenuItemClick = (speed: number) => {
        setSelectedSpeed(speed);
        setPlaybackSpeed(speed);
        handleClose();
    };
    const handleSliderChange = (_: Event, newValue: number | number[]) => {
        if (typeof newValue === "number") {
            moveToTime(newValue);
        }
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
    return (
        <div
            className="px-5 fixed bottom-5 left-1/6 h-20 w-2/3 bg-black/40 text-white shadow-lg rounded-full flex justify-center items-center"
            onContextMenu={handleContextMenu}
        >
            <IconButton
                size="large"
                onClick={togglePlay}
                sx={{ marginRight: "20px", color: "white" }}
            >
                {play ? (
                    <PauseIcon fontSize="inherit" />
                ) : (
                    <PlayArrowIcon fontSize="inherit" />
                )}
            </IconButton>
            <div className="w-2/3 flex items-center select-none text-xl">
                <div>
                    {time < duration
                        ? formatDuration(time)
                        : formatDuration(duration)}
                </div>
                <Slider
                    min={0}
                    max={duration}
                    step={0.000001}
                    className="mx-8"
                    disabled={duration === 0}
                    onChange={handleSliderChange}
                    value={time}
                />
                <div>{formatDuration(duration)}</div>
            </div>
            <div className="ml-5 flex flex-nowrap">
                <IconButton
                    size="large"
                    onClick={handleClick}
                    sx={{ color: "white" }}
                >
                    <SpeedIcon fontSize="inherit" />
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
                    size="large"
                    sx={{ marginLeft: "20px", color: "white" }}
                >
                    <RefreshIcon fontSize="inherit" onClick={restart} />
                </IconButton>
            </div>
        </div>
    );
}
