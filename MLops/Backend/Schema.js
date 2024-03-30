const mongoose = require('mongoose');
mongoose.connect("mongodb://localhost:27017/GEM_DB"); // Add the connection address here to either the local or network server point of MongoDB

const Models = new mongoose.Schema({
    ID: {
        type: Number,
        required: true
    },
    ModelName: {
        type: String,
        required: true
    },
    Address: {
       type: String,
       required: true
    },
    ReadME: {
       type: String,
    }
})
module.exports = mongoose.model("Models", Customer);

const Data = new mongoose.Schema({
    ID: {
        type: Number,
        required: true
    },
    DataName: {
        type: String,
        required: true
    },
    Address: {
       type: String,
       required: true
    },
    ReadME: {
       type: String,
    }
})
module.exports = mongoose.model("Models", Customer);