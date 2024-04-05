const mongoose = require('mongoose');
mongoose.connect("mongodb://localhost:27017/GEM_DB"); // Add the connection address here to either the local or network server point of MongoDB

const Models = new mongoose.Schema({
    ModelName: {
        type: String,
        required: true
    },
    Path: {
       type: String,
       required: true
    },
    DateTime: {
        type: Date,
        default: Date.now
    },
    Description: {
       type: String,
    }
})
module.exports = mongoose.model("Models", Models, "Models");

const Data = new mongoose.Schema({
    DataName: {
        type: String,
        required: true
    },
    Path: {
       type: String,
       required: true
    },
    DateTime: {
        type: Date,
        default: Date.now
    },
    Description: {
       type: String,
    }
})
module.exports = mongoose.model("Data", Data, "Datasets");