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

        if (
          key === 'vehicle' &&
          'type' in value &&
          'data' in value &&
          (value as any).data?.pose?.frame === 3
        ) {
          entries.push({
            key,
            type: (value as any).type,
            time,
            data: (value as any).data,
          });
          continue;
        }

        if (key === 'agents') {
          for (const [agentId, agentValue] of Object.entries(value)) {
            if (
              typeof agentValue === 'object' &&
              agentValue !== null &&
              'data' in agentValue &&
              (agentValue as any).data?.pose?.frame === 1
            ) {
              entries.push({
                key: agentId,
                type: (agentValue as any).type ?? 'AgentState',
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
