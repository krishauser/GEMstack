import { AppHeader } from "@/components/app-header"
import { MapView } from "@/components/map-view"

export default function MapPage() {
    return (
        <main className="flex min-h-screen flex-col items-center bg-neutral-950 text-neutral-100">
            <AppHeader title="Location" />
            <div className="w-full h-[calc(100vh-8rem)]">
                <MapView />
            </div>
        </main>
    )
}

