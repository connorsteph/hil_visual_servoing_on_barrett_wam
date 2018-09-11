// Auto-generated. Do not edit!

// (in-package simulator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Teleop {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dir_2D = null;
    }
    else {
      if (initObj.hasOwnProperty('dir_2D')) {
        this.dir_2D = initObj.dir_2D
      }
      else {
        this.dir_2D = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Teleop
    // Serialize message field [dir_2D]
    bufferOffset = _arraySerializer.float64(obj.dir_2D, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Teleop
    let len;
    let data = new Teleop(null);
    // Deserialize message field [dir_2D]
    data.dir_2D = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.dir_2D.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'simulator/Teleop';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e5c4f3d30b301acd82ac238f760cdb9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] dir_2D
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Teleop(null);
    if (msg.dir_2D !== undefined) {
      resolved.dir_2D = msg.dir_2D;
    }
    else {
      resolved.dir_2D = []
    }

    return resolved;
    }
};

module.exports = Teleop;
