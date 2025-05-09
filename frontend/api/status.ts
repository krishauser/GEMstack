export enum PlannerEnum {
    RRT_STAR         = "RRT_STAR",
    HYBRID_A_STAR    = "HYBRID_A_STAR",
    PARKING          = "PARKING",
    LEAVE_PARKING    = "LEAVE_PARKING",
    IDLE             = "IDLE",
    SUMMON_DRIVING   = "SUMMON_DRIVING",
    PARALLEL_PARKING = "PARALLEL_PARKING",
}

interface StatusResponse {
    status: PlannerEnum;
}


const getCarStatus = async (): Promise<PlannerEnum | null>  => {
    const res = await fetch("http://localhost:8000/api/status", {
        method: "GET",
        headers: {
            "Content-Type": "application/json",
            Accept: "application/json",
        }
    });


    if (!res.ok) {
        console.error("Failed to fetch status:", res.statusText);
        return null;
    }

    const body: StatusResponse = await res.json();
    return body.status;
};

export { getCarStatus };
