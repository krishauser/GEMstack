
"use strict";

let ReassignReportIdCmd = require('./ReassignReportIdCmd.js');
let AutoZeroCalCmd = require('./AutoZeroCalCmd.js');
let UniqueDeviceIdRpt = require('./UniqueDeviceIdRpt.js');
let MotorOverCurrentConfigCmd = require('./MotorOverCurrentConfigCmd.js');
let PriorityConfigCmd = require('./PriorityConfigCmd.js');
let SoftwareRevisionRpt = require('./SoftwareRevisionRpt.js');
let ScheduledReportRatesReq = require('./ScheduledReportRatesReq.js');
let EnhancedPositionRpt = require('./EnhancedPositionRpt.js');
let ActuatorUniqueIdReq = require('./ActuatorUniqueIdReq.js');
let ZeroingMessageRpt = require('./ZeroingMessageRpt.js');
let PositionReachErrorTimeConfigCmd = require('./PositionReachErrorTimeConfigCmd.js');
let ConfigureOutputsKpKiCmd = require('./ConfigureOutputsKpKiCmd.js');
let ReportPollReq = require('./ReportPollReq.js');
let SoftwareVersionReq = require('./SoftwareVersionReq.js');
let ConfigureOutputsPwmFreqCmd = require('./ConfigureOutputsPwmFreqCmd.js');
let MotorCurrentRpt = require('./MotorCurrentRpt.js');
let ReassignCommandIdCmd = require('./ReassignCommandIdCmd.js');
let ConfigureOutputsKdFreqDeadbandCmd = require('./ConfigureOutputsKdFreqDeadbandCmd.js');
let ResetCmd = require('./ResetCmd.js');
let ReportIndex = require('./ReportIndex.js');
let PositionCmd = require('./PositionCmd.js');
let PositionRpt = require('./PositionRpt.js');

module.exports = {
  ReassignReportIdCmd: ReassignReportIdCmd,
  AutoZeroCalCmd: AutoZeroCalCmd,
  UniqueDeviceIdRpt: UniqueDeviceIdRpt,
  MotorOverCurrentConfigCmd: MotorOverCurrentConfigCmd,
  PriorityConfigCmd: PriorityConfigCmd,
  SoftwareRevisionRpt: SoftwareRevisionRpt,
  ScheduledReportRatesReq: ScheduledReportRatesReq,
  EnhancedPositionRpt: EnhancedPositionRpt,
  ActuatorUniqueIdReq: ActuatorUniqueIdReq,
  ZeroingMessageRpt: ZeroingMessageRpt,
  PositionReachErrorTimeConfigCmd: PositionReachErrorTimeConfigCmd,
  ConfigureOutputsKpKiCmd: ConfigureOutputsKpKiCmd,
  ReportPollReq: ReportPollReq,
  SoftwareVersionReq: SoftwareVersionReq,
  ConfigureOutputsPwmFreqCmd: ConfigureOutputsPwmFreqCmd,
  MotorCurrentRpt: MotorCurrentRpt,
  ReassignCommandIdCmd: ReassignCommandIdCmd,
  ConfigureOutputsKdFreqDeadbandCmd: ConfigureOutputsKdFreqDeadbandCmd,
  ResetCmd: ResetCmd,
  ReportIndex: ReportIndex,
  PositionCmd: PositionCmd,
  PositionRpt: PositionRpt,
};
