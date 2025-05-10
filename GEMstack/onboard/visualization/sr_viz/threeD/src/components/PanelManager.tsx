"use client";

import React, { useState } from "react";
import { VideoPanel } from "./VideoPanel";
import { PointCloudPanel } from "./PointCloudPanel";
import { Button, Menu, MenuItem, ListItemIcon, ListItemText } from "@mui/material";
import MenuIcon from "@mui/icons-material/Menu";
import { PanelGroup, Panel, PanelResizeHandle } from "react-resizable-panels";
import HorizontalSplitIcon from '@mui/icons-material/HorizontalSplit';
import VerticalSplitIcon from '@mui/icons-material/VerticalSplit';
import ImageIcon from '@mui/icons-material/Image';
import HubIcon from '@mui/icons-material/Hub';
import CloseIcon from '@mui/icons-material/Close';

type PanelNode =
    | {
          id: number;
          type: "video" | "pointcloud" | "text";
      }
    | {
          id: number;
          split: "horizontal" | "vertical";
          children: PanelNode[];
      };

let panelIdCounter = 2;

function createPanel(type: "video" | "pointcloud" | "text"): PanelNode {
    return { id: panelIdCounter++, type };
}

export const PanelManager = ({
    messageMap,
}: {
    messageMap: Record<string, any[]>;
}) => {
    const [rootPanel, setRootPanel] = useState<PanelNode>(createPanel("video"));

    const renderPanelNode = (node: PanelNode): JSX.Element => {
        if ("split" in node) {
            return (
                <Panel>
                    <PanelGroup key={node.id} direction={node.split}>
                        {node.children.map((child, idx) => (
                            <React.Fragment key={child.id}>
                                {renderPanelNode(child)}
                                {idx < node.children.length - 1 && (
                                    <PanelResizeHandle
                                        className={`${
                                            node.split === "horizontal"
                                                ? "w-1"
                                                : "h-1"
                                        } bg-gray-300 hover:bg-gray-500`}
                                    />
                                )}
                            </React.Fragment>
                        ))}
                    </PanelGroup>
                </Panel>
            );
        }

        return (
            <Panel key={node.id} defaultSize={50} className="border relative">
                <PanelMenu
                    onSplit={(direction) => {
                        const newSplit: PanelNode = {
                            id: panelIdCounter++,
                            split: direction,
                            children: [node, createPanel("video")],
                        };
                        replacePanel(
                            rootPanel,
                            node.id,
                            newSplit,
                            setRootPanel
                        );
                    }}
                    onClose={() => closePanel(rootPanel, node.id, setRootPanel)}
                    onChangeType={(newType) =>
                        changePanelType(
                            rootPanel,
                            node.id,
                            newType,
                            setRootPanel
                        )
                    }
                    rootPanel={rootPanel}
                />
                {node.type === "video" && (
                    <VideoPanel messages={messageMap["video"]} />
                )}
                {node.type === "pointcloud" && (
                    <PointCloudPanel
                        messages={messageMap["pointcloud"]}
                        tfMessages={messageMap["tf"]}
                    />
                )}
            </Panel>
        );
    };

    return (
        <div className="fixed top-0 left-0 w-full h-full">
            {/* <div className="fixed top-5 left-55 flex gap-5 mb-2 z-10">
        <Button onClick={() => console.log(rootPanel)}>Print</Button>
      </div> */}
            <PanelGroup direction="horizontal">
                {renderPanelNode(rootPanel)}
            </PanelGroup>
        </div>
    );
};

function replacePanel(
    root: PanelNode,
    targetId: number,
    replacement: PanelNode,
    setRoot: (r: PanelNode) => void
) {
    function helper(node: PanelNode): PanelNode {
        if (node.id === targetId) return replacement;
        if ("split" in node) {
            return {
                ...node,
                children: node.children.map(helper),
            };
        }
        return node;
    }
    const newRoot = helper(root);
    setRoot(newRoot);
}

function closePanel(
    root: PanelNode,
    targetId: number,
    setRoot: (r: PanelNode) => void
) {
    function helper(node: PanelNode): PanelNode | null {
        if (node.id === targetId) return null;
        if ("split" in node) {
            const newChildren = node.children
                .map(helper)
                .filter((child) => child !== null) as PanelNode[];
            return { ...node, children: newChildren };
        }
        return node;
    }
    const newRoot = helper(root);
    setRoot(newRoot);
}

function changePanelType(
    root: PanelNode,
    targetId: number,
    newType: "video" | "pointcloud" | "text",
    setRoot: (r: PanelNode) => void
) {
    function helper(node: PanelNode): PanelNode {
        if (node.id === targetId) {
            return { ...node, type: newType };
        }
        if ("split" in node) {
            return {
                ...node,
                children: node.children.map(helper),
            };
        }
        return node;
    }
    const newRoot = helper(root);
    setRoot(newRoot);
}

function getNumLeaves(node: PanelNode): number {
    if ("split" in node) {
        return node.children.reduce((sum, child) => sum + getNumLeaves(child), 0);
    }
    return 1;
}

const PanelMenu = ({
    onSplit,
    onClose,
    onChangeType,
    rootPanel,
}: {
    onSplit: (dir: "horizontal" | "vertical") => void;
    onClose: () => void;
    onChangeType: (newType: "video" | "pointcloud") => void;
    rootPanel: PanelNode;
}) => {
    const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
    const open = Boolean(anchorEl);
    const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
        setAnchorEl(event.currentTarget);
    };
    const handleClose = () => setAnchorEl(null);

    return (
        <div className="absolute top-2 right-2 z-10">
            <Button variant="outlined" size="small" onClick={handleClick}>
                <MenuIcon />
            </Button>
            <Menu anchorEl={anchorEl} open={open} onClose={handleClose}>
                <MenuItem
                    onClick={() => {
                        onSplit("horizontal");
                        handleClose();
                    }}
                >
                    <ListItemIcon>
                        <HorizontalSplitIcon fontSize="small" />
                    </ListItemIcon>
                    <ListItemText>
                        Split Horizontally
                    </ListItemText>
                </MenuItem>
                <MenuItem
                    onClick={() => {
                        onSplit("vertical");
                        handleClose();
                    }}
                >
                    <ListItemIcon>
                        <VerticalSplitIcon fontSize="small" />
                    </ListItemIcon>
                    <ListItemText>
                        Split Vertically
                    </ListItemText>
                </MenuItem>
                <MenuItem
                    onClick={() => {
                        onChangeType("video");
                        handleClose();
                    }}
                >
                    <ListItemIcon>
                        <ImageIcon fontSize="small" />
                    </ListItemIcon>
                    <ListItemText>
                        Switch to Video Panel
                    </ListItemText>
                </MenuItem>
                <MenuItem
                    onClick={() => {
                        onChangeType("pointcloud");
                        handleClose();
                    }}
                >
                    <ListItemIcon>
                        <HubIcon fontSize="small" />
                    </ListItemIcon>
                    <ListItemText>
                        Switch to PointCloud Panel
                    </ListItemText>
                </MenuItem>
                {getNumLeaves(rootPanel) > 1 && (
                    <MenuItem
                        onClick={() => {
                            onClose();
                            handleClose();
                        }}
                    >
                        <ListItemIcon>
                            <CloseIcon fontSize="small" />
                        </ListItemIcon>
                        <ListItemText>
                            Close Panel
                        </ListItemText>
                    </MenuItem>
                )}
            </Menu>
        </div>
    );
};
