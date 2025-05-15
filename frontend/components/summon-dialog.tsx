"use client";

import { useState, useEffect, useRef } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { Dialog, DialogContent, DialogTitle } from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { Car, CheckCircle, Loader2 } from "lucide-react";
import { Progress } from "@/components/ui/progress";
import { getCarStatus, PlannerEnum } from "@/api/status";

export const SummonDialog = () => {
  const [carStatus, setCarStatus] = useState<PlannerEnum | null>(null);
  const [showDialog, setShowDialog] = useState(false);
  const [progress, setProgress] = useState(0);
  const [showSuccess, setShowSuccess] = useState(false);

  const prevStatus = useRef<PlannerEnum | null>(null);

  useEffect(() => {
    const fetchStatus = async () => {
      const status = await getCarStatus();

      const prev = prevStatus.current;
      // detect transition non-idle -> idle
      if (prev && prev !== PlannerEnum.IDLE && status === PlannerEnum.IDLE) {
        setShowSuccess(true);
        setTimeout(() => {
          setShowDialog(false);
          setShowSuccess(false);
        }, 5000);
      }
      // detect idle -> non-idle
      if (
        (prev === PlannerEnum.IDLE || prev === null) &&
        status &&
        status !== PlannerEnum.IDLE
      ) {
        setShowDialog(true);
        setProgress(0);
      }
      prevStatus.current = status;
      setCarStatus(status);
    };

    const updateProgress = () => {
      if (
        carStatus === PlannerEnum.RRT_STAR ||
        carStatus === PlannerEnum.HYBRID_A_STAR
      ) {
        setProgress((prev) => Math.min(prev + 2, 100));
      } else if (
        carStatus === PlannerEnum.SUMMON_DRIVING ||
        carStatus === PlannerEnum.LEAVE_PARKING
      ) {
        setProgress((prev) => Math.min(prev + 0.5, 95));
      } else if (
        carStatus === PlannerEnum.PARKING ||
        carStatus === PlannerEnum.PARALLEL_PARKING
      ) {
        setProgress((prev) => Math.min(prev + 1, 98));
      } else if (carStatus === PlannerEnum.IDLE) {
        setProgress(100);
      } else {
        setProgress(0);
      }
    };

    fetchStatus();
    const statusInterval = window.setInterval(fetchStatus, 3000);
    const progressInterval = window.setInterval(updateProgress, 200);

    return () => {
      window.clearInterval(statusInterval);
      window.clearInterval(progressInterval);
    };
  }, [carStatus]);

  const getStatusMessage = () => {
    switch (carStatus) {
      case PlannerEnum.RRT_STAR:
        return "Planning optimal route...";
      case PlannerEnum.HYBRID_A_STAR:
        return "Calculating path...";
      case PlannerEnum.PARKING:
        return "Parking your car";
      case PlannerEnum.LEAVE_PARKING:
        return "Exiting parking space";
      case PlannerEnum.SUMMON_DRIVING:
        return "Your car is on the way";
      case PlannerEnum.PARALLEL_PARKING:
        return "Parallel parking in progress";
      case PlannerEnum.IDLE:
        return "Ready";
      default:
        return "Connecting to your car...";
    }
  };

  const getStatusIcon = () => {
    if (
      carStatus === PlannerEnum.RRT_STAR ||
      carStatus === PlannerEnum.HYBRID_A_STAR
    ) {
      return <Loader2 className="h-10 w-10 text-blue-400 animate-spin" />;
    } else if (
      carStatus === PlannerEnum.SUMMON_DRIVING ||
      carStatus === PlannerEnum.LEAVE_PARKING
    ) {
      return <Car className="h-10 w-10 text-blue-400" />;
    } else if (
      carStatus === PlannerEnum.PARKING ||
      carStatus === PlannerEnum.PARALLEL_PARKING
    ) {
      return <Car className="h-10 w-10 text-yellow-400" />;
    } else if (carStatus === PlannerEnum.IDLE) {
      return <CheckCircle className="h-10 w-10 text-green-400" />;
    } else {
      return <Loader2 className="h-10 w-10 animate-spin" />;
    }
  };

  return (
    <Dialog
      open={showDialog}
      onOpenChange={(open) => {
        if (carStatus === PlannerEnum.IDLE || showSuccess) setShowDialog(open);
      }}
    >
      <DialogContent className="bg-neutral-900 border-neutral-800 p-0 overflow-hidden max-w-md w-full rounded-xl">
        <DialogTitle className="bg-neutral-800 text-white p-4 text-lg">
          Vehicle Status
        </DialogTitle>
        <div className="p-6">
          <AnimatePresence mode="wait">
            {showSuccess ? (
              <motion.div
                key="success"
                initial={{ opacity: 0, scale: 0.8 }}
                animate={{ opacity: 1, scale: 1 }}
                exit={{ opacity: 0, scale: 0.8 }}
                className="flex flex-col items-center justify-center text-center space-y-4"
              >
                <CheckCircle className="h-16 w-16 text-green-400" />
                <h2 className="text-2xl font-medium">Your car has arrived</h2>
                <p className="text-neutral-400">
                  Your GEM is ready and waiting for you
                </p>
                <Button
                  onClick={() => {
                    setShowDialog(false);
                    setShowSuccess(false);
                  }}
                  className="mt-4"
                >
                  Close
                </Button>
              </motion.div>
            ) : (
              <motion.div
                key="status"
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
                exit={{ opacity: 0 }}
                className="flex flex-col items-center justify-center text-center space-y-6"
              >
                {getStatusIcon()}
                <div className="space-y-2 w-full">
                  <h2 className="text-xl font-medium">{getStatusMessage()}</h2>
                  {carStatus && carStatus !== PlannerEnum.IDLE && (
                    <>
                      <Progress value={progress} className="h-2 bg-neutral-800">
                        <div
                          className={`h-full ${carStatus === PlannerEnum.PARKING || carStatus === PlannerEnum.PARALLEL_PARKING ? "bg-yellow-500" : "bg-blue-500"}`}
                          style={{ width: `${progress}%` }}
                        />
                      </Progress>
                      <p className="text-xs text-neutral-400 mt-2">
                        {carStatus === PlannerEnum.RRT_STAR ||
                        carStatus === PlannerEnum.HYBRID_A_STAR
                          ? "Planning optimal route"
                          : carStatus === PlannerEnum.PARKING ||
                              carStatus === PlannerEnum.PARALLEL_PARKING
                            ? "Finding the perfect spot"
                            : "Estimated arrival time: soon"}
                      </p>
                    </>
                  )}
                </div>
                {carStatus === PlannerEnum.IDLE && (
                  <Button onClick={() => setShowDialog(false)}>Close</Button>
                )}
              </motion.div>
            )}
          </AnimatePresence>
        </div>
      </DialogContent>
    </Dialog>
  );
};
