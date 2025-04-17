"use client"

import { Info } from "lucide-react"
import { Button } from "@/components/ui/button"
import { motion } from "framer-motion"

interface AppHeaderProps {
    title: string
}

export function AppHeader({ title }: AppHeaderProps) {
    return (
        <motion.header
            className="w-full px-4 py-4 flex justify-between items-center border-b border-neutral-800"
            initial={{ opacity: 0, y: -20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
        >
            <h1 className="text-xl font-medium">{title}</h1>
            <div className="flex items-center gap-2">
                <Button variant="ghost" size="icon" className="rounded-full">
                    <Info className="h-5 w-5" />
                </Button>
            </div>
        </motion.header>
    )
}

