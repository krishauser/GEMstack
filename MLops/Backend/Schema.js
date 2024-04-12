const mongoose = require('mongoose');

const ModelsSchema = new mongoose.Schema({
    ModelName: {
        type: String,
        required: true
    },
    Path: {
       type: String,
       required: true
    },
    Description: {
       type: String,
    },
    DateTime: {
        type: Date,
        default: Date.now,
        required: true
    }
});
module.exports = mongoose.model("Models", ModelsSchema, "Models");

const DataSchema = new mongoose.Schema({
    DataName: {
        type: String,
        required: true
    },
    Path: {
       type: String,
       required: true
    },
    Description: {
       type: String,
    },
    DateTime: {
        type: Date,
        default: Date.now,
        required: true
    },
    Topics: {
        type: [String],
        default: []
    },
    Source: {
        type: String,
        default: ""
    }
});
module.exports = mongoose.model("Data", DataSchema, "Datasets");
