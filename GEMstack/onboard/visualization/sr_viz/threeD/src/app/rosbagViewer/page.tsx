"use client";

import RosbagViewer from "@/components/RosbagViewer";
import { useRouter } from "next/navigation";
import Button from "@mui/material/Button";
import HomeIcon from "@mui/icons-material/Home";

export default function RosbagViewerPage() {
    const router = useRouter();
    const handleRedirect = () => {
        router.replace("/");
    };
    return (
        <div className="w-screen h-screen">
            <div className="flex items-center fixed bottom-5 left-0 z-50 group w-[240px] h-[48px]">
                <div className="w-fit transform -translate-x-4/5 transition-transform duration-300 group-hover:translate-x-[20px]">
                    <Button
                        variant="contained"
                        endIcon={<HomeIcon />}
                        onClick={handleRedirect}
                        className="rounded-full bg-slate-400 text-white px-4 py-2 hover:bg-slate-700"
                        sx={{
                            color: "white",
                            backgroundColor: "black",
                            "&:hover": { backgroundColor: "gray" },
                            borderRadius: "9999px",
                        }}
                    >
                        Go to Visualizer
                    </Button>
                </div>
            </div>
            <RosbagViewer />
        </div>
    );
}
