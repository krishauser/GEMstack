export interface LogEntry<T = any> {
    key: string;        // Top-level key (e.g., "vehicle", "agents", etc.)
    type: string;       // Inner type (e.g., "VehicleState")
    time: number;       // Timestamp of the event
    data: T;            // Full data object (includes pose, velocity, etc.)
}

