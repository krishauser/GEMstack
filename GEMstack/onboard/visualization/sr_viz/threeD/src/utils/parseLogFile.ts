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

        if (key === 'vehicle' && 'type' in value && 'data' in value && (value as any).data?.pose?.frame === 3) {
          entries.push({
            key,
            type: (value as any).type,
            time,
            data: (value as any).data,
          });
          continue;
        }

        if (key === 'agents' || key === 'traffic_lights' || key === 'other_vehicles') {
          for (const [itemId, itemValue] of Object.entries(value)) {
            if (typeof itemValue === 'object' && itemValue !== null && 'data' in itemValue) {
              const frame = (itemValue as any).data?.pose?.frame;

              if (key === 'agents' && frame !== 1) {
                continue;
              }

              entries.push({
                key: itemId,
                type: (itemValue as any).type ?? guessTypeFromKey(key),
                time,
                data: (itemValue as any).data,
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

function guessTypeFromKey(key: string): string {
  if (key === 'agents') return 'AgentState';
  if (key === 'traffic_lights') return 'TrafficLightState';
  if (key === 'other_vehicles') return 'OtherVehicleState';
  return 'Unknown';
}
