export async function* streamLogLines(file: File): AsyncGenerator<string> {
    const reader = file.stream().getReader();
    const decoder = new TextDecoder('utf-8');
    let partial = '';

    while (true) {
        const { value, done } = await reader.read();
        if (done) break;

        partial += decoder.decode(value, { stream: true });
        const lines = partial.split('\n');
        partial = lines.pop() || '';

        for (const line of lines) {
            if (line.trim()) yield line;
        }
    }

    if (partial.trim()) yield partial;
}
