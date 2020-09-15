// Auto-generated. Do not edit!

// (in-package blaser_pcl.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class VoxelGridStitchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start = null;
      this.pause = null;
      this.clear = null;
      this.leaf_size = null;
    }
    else {
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = 0;
      }
      if (initObj.hasOwnProperty('pause')) {
        this.pause = initObj.pause
      }
      else {
        this.pause = 0;
      }
      if (initObj.hasOwnProperty('clear')) {
        this.clear = initObj.clear
      }
      else {
        this.clear = 0;
      }
      if (initObj.hasOwnProperty('leaf_size')) {
        this.leaf_size = initObj.leaf_size
      }
      else {
        this.leaf_size = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VoxelGridStitchRequest
    // Serialize message field [start]
    bufferOffset = _serializer.int8(obj.start, buffer, bufferOffset);
    // Serialize message field [pause]
    bufferOffset = _serializer.int8(obj.pause, buffer, bufferOffset);
    // Serialize message field [clear]
    bufferOffset = _serializer.int8(obj.clear, buffer, bufferOffset);
    // Serialize message field [leaf_size]
    bufferOffset = _serializer.float64(obj.leaf_size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VoxelGridStitchRequest
    let len;
    let data = new VoxelGridStitchRequest(null);
    // Deserialize message field [start]
    data.start = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [pause]
    data.pause = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [clear]
    data.clear = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [leaf_size]
    data.leaf_size = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 11;
  }

  static datatype() {
    // Returns string type for a service object
    return 'blaser_pcl/VoxelGridStitchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9b2baaa3ec3d1724df9a4f441fb6f00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 start
    int8 pause
    int8 clear
    float64 leaf_size
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VoxelGridStitchRequest(null);
    if (msg.start !== undefined) {
      resolved.start = msg.start;
    }
    else {
      resolved.start = 0
    }

    if (msg.pause !== undefined) {
      resolved.pause = msg.pause;
    }
    else {
      resolved.pause = 0
    }

    if (msg.clear !== undefined) {
      resolved.clear = msg.clear;
    }
    else {
      resolved.clear = 0
    }

    if (msg.leaf_size !== undefined) {
      resolved.leaf_size = msg.leaf_size;
    }
    else {
      resolved.leaf_size = 0.0
    }

    return resolved;
    }
};

class VoxelGridStitchResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VoxelGridStitchResponse
    // Serialize message field [status]
    bufferOffset = _serializer.int64(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VoxelGridStitchResponse
    let len;
    let data = new VoxelGridStitchResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'blaser_pcl/VoxelGridStitchResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4107476a6271fc2684d94be17d33f854';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VoxelGridStitchResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: VoxelGridStitchRequest,
  Response: VoxelGridStitchResponse,
  md5sum() { return '60c18bc0467086605bc6bd7bde5154dd'; },
  datatype() { return 'blaser_pcl/VoxelGridStitch'; }
};
