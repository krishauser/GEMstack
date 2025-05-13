"use client";

import React, { useState } from "react";
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
  const [isDragging, setIsDragging] = useState<boolean>(false);
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
  const handleSliderChange = (_: Event, newValue: number) => {
    moveToTime(newValue);
  };
  const handleSliderChangeCommitted = () => {
    setIsDragging(false);
  };
  const formatDuration = (time: number) => {
    const minutes = Math.floor(time / 60);
    const seconds = Math.floor(time % 60);
    return `${minutes < 10 ? "0" : ""}${minutes}:${seconds < 10 ? "0" : ""}${seconds}`;
  };
  const handleContextMenu = (event: React.MouseEvent) => {
    event.preventDefault();
  };

  return (
    <div
      className="px-4 py-2 fixed bottom-4 left-1/6 w-2/3 bg-black/80 text-white shadow-md rounded-full flex items-center justify-between"
      onContextMenu={handleContextMenu}
    >
      <IconButton
        size="small"
        onClick={togglePlay}
        sx={{ color: "white", mr: 1 }}
      >
        {play ? <PauseIcon fontSize="small" /> : <PlayArrowIcon fontSize="small" />}
      </IconButton>

      <div className="w-full flex items-center mx-2 text-sm">
        <span className="min-w-[36px] text-center">
          {formatDuration(Math.min(time, duration))}
        </span>
        <Slider
          key={isDragging ? undefined : Math.floor(time * 100)}
          min={0}
          max={duration}
          step={0.02}
          className="mx-3"
          disabled={duration === 0}
          value={time}
          onChange={handleSliderChange}
          onMouseDown={() => setIsDragging(true)}
          onChangeCommitted={handleSliderChangeCommitted}
        />
        <span className="min-w-[36px] text-center">{formatDuration(duration)}</span>
      </div>

      <div className="flex items-center">
        <IconButton size="small" onClick={handleClick} sx={{ color: "white" }}>
          <SpeedIcon fontSize="small" />
        </IconButton>
        <Menu
          anchorEl={anchorEl}
          open={open}
          onClose={handleClose}
          anchorOrigin={{ vertical: "top", horizontal: "center" }}
          transformOrigin={{ vertical: "bottom", horizontal: "center" }}
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
          onClick={restart}
          sx={{ color: "white", ml: 1 }}
        >
          <RefreshIcon fontSize="small" />
        </IconButton>
      </div>
    </div>
  );
}