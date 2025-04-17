"use client"

import { useState } from "react"
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { motion } from "framer-motion"
import { ChevronDown, ChevronUp, User, Calendar, Hash, MapPin } from "lucide-react"

// Mock user data
const MOCK_USER = {
    name: "CS 588 Student",
    email: "cs588@illinois.edu",
    avatar: "/placeholder-user.jpg",
}

// Mock car data
const CAR_DATA = {
    model: "GEM e2 Vehicle",
    licensePlate: "UIUC-CS",
    location: "High Bay",
    controls: "steering, braking, acceleration, turning lights",
    rosAccess: "left & right blinkers, forward & reverse gear selection",
}

export function OwnerInfo() {
    const [expanded, setExpanded] = useState(false)

    return (
        <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
            className="w-full bg-neutral-900 rounded-xl p-5 shadow-lg mb-4"
        >
            <div className="flex items-center justify-between">
                <div className="flex items-center gap-3">
                    <Avatar className="h-10 w-10 border border-neutral-700">
                        <AvatarImage src={MOCK_USER.avatar || "/placeholder-user.jpg"} alt={MOCK_USER.name} />
                        <AvatarFallback>
                            <User className="h-5 w-5" />
                        </AvatarFallback>
                    </Avatar>
                    <div>
                        <h3 className="font-medium">{MOCK_USER.name}</h3>
                        <p className="text-xs text-neutral-400">Owner</p>
                    </div>
                </div>
                <Badge variant="outline" className="bg-neutral-800">
                    UIUC
                </Badge>
            </div>

            <Button
                variant="ghost"
                size="sm"
                onClick={() => setExpanded(!expanded)}
                className="mt-2 w-full justify-between text-neutral-400 hover:text-white"
            >
                {expanded ? "Hide details" : "View details"}
                {expanded ? <ChevronUp className="h-4 w-4" /> : <ChevronDown className="h-4 w-4" />}
            </Button>

            {expanded && (
                <motion.div
                    initial={{ opacity: 0, height: 0 }}
                    animate={{ opacity: 1, height: "auto" }}
                    exit={{ opacity: 0, height: 0 }}
                    transition={{ duration: 0.3 }}
                    className="mt-4 space-y-3 text-sm"
                >
                    <div className="grid grid-cols-2 gap-3">
                        <div className="flex items-center gap-2 text-neutral-400">
                            <Calendar className="h-4 w-4"/>
                            <span>Model:</span>
                        </div>
                        <div className="font-medium">{CAR_DATA.model}</div>

                        <div className="flex items-center gap-2 text-neutral-400">
                            <Hash className="h-4 w-4"/>
                            <span>License:</span>
                        </div>
                        <div className="font-medium">{CAR_DATA.licensePlate}</div>

                        <div className="flex items-center gap-2 text-neutral-400">
                            <MapPin className="h-4 w-4"/>
                            <span>Location:</span>
                        </div>
                        <div className="font-medium">{CAR_DATA.location}</div>

                        <div className="flex items-center gap-2 text-neutral-400">
                            <Calendar className="h-4 w-4"/>
                            <span>Controls:</span>
                        </div>
                        <div className="font-medium">{CAR_DATA.controls}</div>

                        <div className="flex items-center gap-2 text-neutral-400">
                            <Hash className="h-4 w-4"/>
                            <span>ROS Access:</span>
                        </div>
                        <div className="font-medium">{CAR_DATA.rosAccess}</div>
                    </div>
                </motion.div>
            )}
        </motion.div>
    )
}

