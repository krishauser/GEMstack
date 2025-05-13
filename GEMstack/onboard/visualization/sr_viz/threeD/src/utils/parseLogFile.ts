import { LogEntry } from '../types/LogEntry';
import { streamLogLines } from './streamLogLines';

export async function parseLogFile(file: File): Promise<LogEntry[]> {
  const entries: LogEntry[] = [];

  for await (const line of streamLogLines(file)) {
    try {
      const parsed = JSON.parse(line);
      const time = parsed.time;

      for (const [key, value] of Object.entries(parsed)) {
        if (key === 'time' || typeof value !== 'object' || value === null) continue;

        // Main vehicle (frame 2)
        if (
          key === 'vehicle' &&
          'type' in value &&
          'data' in value &&
          (value as any).data?.pose?.frame === 2
        ) {
          entries.push({
            key,
            type: (value as any).type,
            time,
            data: (value as any).data,
          });
          continue;
        }

        // Unified agents (pedestrians, lights, cones, others) — frame 0 only
        if (key === 'agents') {
          for (const [agentId, agentValue] of Object.entries(value)) {
            if (
              typeof agentValue === 'object' &&
              agentValue !== null &&
              'data' in agentValue &&
              (agentValue as any).data?.pose?.frame === 0
            ) {
              const inferredType = getTypeFromKey(agentId);
              entries.push({
                key: agentId,
                type: inferredType,
                time,
                data: (agentValue as any).data,
              });
            }
          }
        }
      }
    } catch (err) {
      console.warn('Failed to parse log line:', err);
    }
  }

  return entries;
}

function getTypeFromKey(key: string): string {
  const lowerKey = key.toLowerCase();
  if (lowerKey.startsWith('pedestrian')) return 'PedestrianState';
  if (lowerKey.startsWith('vehicle')) return 'OtherVehicleState';
  if (lowerKey.startsWith('light')) return 'TrafficLightState';
  if (lowerKey.startsWith('cone')) return 'TrafficConeState';
  return 'UnknownState';
}
