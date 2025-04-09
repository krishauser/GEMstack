export default class LogReader {
    constructor(logData, scene, createFuncs) {
        this.framerate = logData.framerate || 60;
        this.totalFrames = logData.totalFrames || 0;
        this.frames = logData.data || [];
        this.scene = scene;
        this.currentFrame = 0;
        this.playing = true;
        this.createFuncs = createFuncs;
        this.models = {}
        this.car = null;
        for (const type of Object.keys(createFuncs)) {
            this.models[type] = new Map();
        }
    }

    update(dt) {
        if (!this.playing) return;

        const frameData = this.frames[this.currentFrame];
        if (!frameData) return;

        frameData.objects.forEach(obj => {
            const id = obj.id;
            const type = obj.type;
            const position = obj.position;
            if (this.models[type].has(id)) {
                const model = this.models[type].get(id);
                if ((type === 'vehicle' && obj.frame === 0) || type !== 'vehicle') {
                    model.updateFromLog(obj);
                }
            } else {
                const createFunc = this.createFuncs[type];
                if (createFunc) {
                    const orientation = obj.orientation || obj.misc.orientation;
                    const model = createFunc(position[1], position[2], position[0], orientation);
                    if (type === 'vehicle' && id === 0) {
                        this.car = model;
                    }
                    this.models[type].set(id, model);
                }
            }
        });

        this.currentFrame++;
        if (this.currentFrame >= this.frames.length) {
            this.playing = false;
        }
    }

    reset() {
        this.currentFrame = 0;
        this.playing = true;
    }

    pause() {
        this.playing = false;
    }

    play() {
        this.playing = true;
    }
}
