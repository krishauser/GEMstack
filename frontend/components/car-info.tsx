"use client"

import { useState, useEffect } from "react"
import { Battery, Thermometer, Lock, Unlock, Fan, Zap, Wifi } from "lucide-react"
import { Button } from "@/components/ui/button"
import { Progress } from "@/components/ui/progress"
import { cn } from "@/lib/utils"
import { motion, AnimatePresence } from "framer-motion"
import { toast } from "sonner"

export function CarInfo() {
    const [batteryLevel, setBatteryLevel] = useState(80)
    const [temperature, setTemperature] = useState(72)
    const [isLocked, setIsLocked] = useState(true)
    const [isAcOn, setIsAcOn] = useState(false)
    const [isCharging, setIsCharging] = useState(false)
    const [isConnected, setIsConnected] = useState(true)

    // Simulate battery drain or charge
    useEffect(() => {
        const interval = setInterval(() => {
            if (isCharging && batteryLevel < 100) {
                setBatteryLevel((prev) => Math.min(prev + 1, 100))
            } else if (!isCharging && Math.random() > 0.7) {
                setBatteryLevel((prev) => Math.max(prev - 1, 10))
            }
        }, 5000)

        return () => clearInterval(interval)
    }, [isCharging, batteryLevel])

    const getBatteryColor = () => {
        if (batteryLevel > 50) return "bg-green-500"
        if (batteryLevel > 20) return "bg-yellow-500"
        return "bg-red-500"
    }

    const toggleLock = () => {
        setIsLocked(!isLocked)
        if (isLocked) {
            toast.success("Vehicle unlocked", {
                description: "Your car is now unlocked",
            })
        } else {
            toast.success("Vehicle locked", {
                description: "Your car is now locked",
            })
        }
    }

    const toggleAc = () => {
        setIsAcOn(!isAcOn)
        if (isAcOn) {
            toast.success("Climate off", {
                description: "Climate control turned off",
            })
        } else {
            toast.success("Climate on", {
                description: "Climate control turned on",
            })
        }
    }

    const toggleCharging = () => {
        setIsCharging(!isCharging)
        if (isCharging) {
            toast.success("Charging stopped", {
                description: "Your car is no longer charging",
            })
        } else {
            toast.success("Charging started", {
                description: "Your car is now charging",
            })
        }
    }

    return (
        <div className="w-full bg-neutral-900 rounded-xl p-5 shadow-lg">
            <h2 className="text-xl font-medium mb-4">Status</h2>

            <div className="space-y-6">
                <div>
                    <div className="flex items-center justify-between mb-2">
                        <div className="flex items-center">
                            <Battery className={cn("mr-2 h-5 w-5 transition-colors", isCharging && "text-green-500")} />
                            <span>Battery</span>
                        </div>
                        <div className="flex items-center gap-2">
                            <AnimatePresence>
                                {isCharging && (
                                    <motion.div
                                        initial={{ opacity: 0, scale: 0 }}
                                        animate={{ opacity: 1, scale: 1 }}
                                        exit={{ opacity: 0, scale: 0 }}
                                        className="text-green-500 text-xs"
                                    >
                                        Charging
                                    </motion.div>
                                )}
                            </AnimatePresence>
                            <span className="font-medium">{batteryLevel}%</span>
                            <Button
                                variant="outline"
                                size="icon"
                                className={cn("rounded-full h-8 w-8", isCharging ? "bg-green-500/20 text-green-400" : "")}
                                onClick={toggleCharging}
                            >
                                <Zap className="h-4 w-4" />
                            </Button>
                        </div>
                    </div>
                    <Progress value={batteryLevel} className="h-2 bg-neutral-800">
                        <motion.div
                            className={cn("h-full", getBatteryColor())}
                            style={{ width: `${batteryLevel}%` }}
                            initial={{ width: 0 }}
                            animate={{ width: `${batteryLevel}%` }}
                            transition={{ duration: 1 }}
                        />
                    </Progress>
                </div>

                <div className="flex items-center justify-between">
                    <div className="flex items-center">
                        <Thermometer className={cn("mr-2 h-5 w-5", isAcOn && "text-blue-400")} />
                        <span>Interior</span>
                    </div>
                    <div className="flex items-center gap-2">
                        <motion.span
                            className="font-medium"
                            key={temperature}
                            initial={{ opacity: 0, y: -10 }}
                            animate={{ opacity: 1, y: 0 }}
                            transition={{ duration: 0.3 }}
                        >
                            {temperature}°F
                        </motion.span>
                        <Button
                            variant="outline"
                            size="icon"
                            className={cn("rounded-full h-8 w-8", isAcOn ? "bg-blue-500/20 text-blue-400" : "")}
                            onClick={toggleAc}
                        >
                            <Fan className="h-4 w-4" />
                        </Button>
                    </div>
                </div>

                <div className="flex items-center justify-between">
                    <div className="flex items-center">
                        {isLocked ? <Lock className="mr-2 h-5 w-5" /> : <Unlock className="mr-2 h-5 w-5 text-green-400" />}
                        <span>Vehicle</span>
                    </div>
                    <Button
                        variant="outline"
                        size="sm"
                        className={cn("rounded-full", !isLocked && "bg-green-500/20 text-green-400")}
                        onClick={toggleLock}
                    >
                        {isLocked ? "Unlock" : "Lock"}
                    </Button>
                </div>

                <div className="flex items-center justify-between">
                    <div className="flex items-center">
                        <Wifi className={cn("mr-2 h-5 w-5", isConnected ? "text-green-400" : "text-neutral-500")} />
                        <span>Connection</span>
                    </div>
                    <span
                        className={cn(
                            "text-xs px-2 py-1 rounded-full",
                            isConnected ? "bg-green-500/20 text-green-400" : "bg-neutral-800 text-neutral-400",
                        )}
                    >
            {isConnected ? "Online" : "Offline"}
          </span>
                </div>
            </div>
        </div>
    )
}

