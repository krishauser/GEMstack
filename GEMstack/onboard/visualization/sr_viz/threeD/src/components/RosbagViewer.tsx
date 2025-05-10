"use client";

import React, { useState } from "react";
import { Bag } from "@foxglove/rosbag";
import Button from "@mui/material/Button";
import CloudUploadIcon from "@mui/icons-material/CloudUpload";
import { styled } from "@mui/material/styles";
import { ScrubberProvider } from "./ScrubberContext";
import { Scrubber2 } from "@/components/Scrubber2";
import { PanelManager } from "./PanelManager";

const VisuallyHiddenInput = styled("input")({
    clip: "rect(0 0 0 0)",
    clipPath: "inset(50%)",
    height: 1,
    overflow: "hidden",
    position: "absolute",
    bottom: 0,
    left: 0,
    whiteSpace: "nowrap",
    width: 1,
});

function fileToReader(file: File) {
    return {
        size: () => file.size,
        read: async (offset: number, length: number) => {
            const blob = file.slice(offset, offset + length);
            const arrayBuffer = await blob.arrayBuffer();
            return new Uint8Array(arrayBuffer);
        },
    };
}

export default function RosbagViewer() {
    const [topics, setTopics] = useState<string[]>([]);
    const [types, setTypes] = useState<string[]>([]);
    const [loading, setLoading] = useState(false);
    const [messageMap, setMessageMap] = useState<Record<string, any[]>>({
        video: [],
        pointcloud: [],
    });

    const [duration, setDuration] = useState(0);
    const [startTime, setStartTime] = useState(0);

    const handleFileUpload = async (
        event: React.ChangeEvent<HTMLInputElement>
    ) => {
        const file = event.target.files?.[0];
        if (!file) return;
        
        setMessageMap({
            video: [],
            pointcloud: [],
            tf: [],
        });
        setLoading(true);
        const reader = fileToReader(file);
        const bag = new Bag(reader);
        await bag.open();
        console.log("Bag opened:", bag);
        const topicNames = Array.from(bag.connections.values()).map(
            (conn) => conn.topic
        );
        const topicTypes = Array.from(bag.connections.values()).map(
            (conn) => conn.type
        );
        setTopics(topicNames);
        setTypes(topicTypes);
        console.log("Topics:", topicNames, topicTypes);

        const videoMessages: any[] = [];
        const pointcloudMessages: any[] = [];
        const tfMessages: any[] = [];
        const velodyneScanMessages: any[] = [];
        for await (const msg of bag.messageIterator()) {
            const entry = {
                timestamp: msg.timestamp.sec + msg.timestamp.nsec * 1e-9,
                topic: msg.topic,
                data: msg.message,
            };
            const type = topicTypes[topicNames.indexOf(msg.topic)];
            if (type.includes("Image")) videoMessages.push(entry);
            if (type.includes("PointCloud2")) pointcloudMessages.push(entry);
            if (type.includes("TFMessage")) tfMessages.push(entry);
            if (type.includes("VelodyneScan")) velodyneScanMessages.push(entry);
        }
        setLoading(false);
        console.log("Messages parsed:", videoMessages, pointcloudMessages, tfMessages, velodyneScanMessages);
        setMessageMap({
            video: videoMessages,
            pointcloud: pointcloudMessages,
            tf: tfMessages,
        });
        if (videoMessages.length > 0) {
            setDuration(
                Math.max(
                    videoMessages[videoMessages.length - 1].timestamp -
                        videoMessages[0].timestamp,
                    0
                )
            );
        }
        if (pointcloudMessages.length > 0) {
            setDuration((prev) =>
                Math.max(
                    prev,
                    pointcloudMessages[pointcloudMessages.length - 1]
                        .timestamp - pointcloudMessages[0].timestamp,
                    0
                )
            );
        }
        const videoStart = videoMessages[0]?.timestamp || 0;
        const pointcloudStart = pointcloudMessages[0]?.timestamp || 0;
        const start = (videoStart > 0 && pointcloudStart > 0) ? Math.min(videoStart, pointcloudStart) : Math.max(videoStart, pointcloudStart);
        setStartTime(start);
    };

    return (
        <div className="relative h-full w-full">
            <div className="flex items-center fixed bottom-5 right-0 z-50 group w-[200px] h-[48px]">
                <div className="w-fit transform translate-x-6/7 transition-transform duration-300 group-hover:translate-x-0">
                    <Button
                    component="label"
                    role={undefined}
                    variant="contained"
                    tabIndex={-1}
                    startIcon={<CloudUploadIcon />}
                    sx={{
                        // backgroundColor: "#2196f3",
                        "&:hover": {
                        backgroundColor: "#2196f3",
                        },
                        borderRadius: "9999px",
                    }}
                    >
                    Upload ROS Bag
                    <VisuallyHiddenInput
                        type="file"
                        onChange={handleFileUpload}
                        accept=".bag"
                    />
                    </Button>
                </div>
            </div>

            <ScrubberProvider>
                <PanelManager messageMap={messageMap} />
                <Scrubber2 duration={duration} startTime={startTime} loading={loading} />
            </ScrubberProvider>
        </div>
    );
}
