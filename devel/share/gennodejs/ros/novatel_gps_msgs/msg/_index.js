
"use strict";

let NovatelUtmPosition = require('./NovatelUtmPosition.js');
let Trackstat = require('./Trackstat.js');
let Gpgsv = require('./Gpgsv.js');
let Range = require('./Range.js');
let TrackstatChannel = require('./TrackstatChannel.js');
let Gprmc = require('./Gprmc.js');
let Inspva = require('./Inspva.js');
let NovatelReceiverStatus = require('./NovatelReceiverStatus.js');
let Gphdt = require('./Gphdt.js');
let Gpgga = require('./Gpgga.js');
let Gpgsa = require('./Gpgsa.js');
let RangeInformation = require('./RangeInformation.js');
let NovatelCorrectedImuData = require('./NovatelCorrectedImuData.js');
let ClockSteering = require('./ClockSteering.js');
let Inspvax = require('./Inspvax.js');
let NovatelXYZ = require('./NovatelXYZ.js');
let Inscov = require('./Inscov.js');
let Insstdev = require('./Insstdev.js');
let NovatelSignalMask = require('./NovatelSignalMask.js');
let NovatelHeading2 = require('./NovatelHeading2.js');
let NovatelExtendedSolutionStatus = require('./NovatelExtendedSolutionStatus.js');
let Satellite = require('./Satellite.js');
let NovatelDualAntennaHeading = require('./NovatelDualAntennaHeading.js');
let NovatelPosition = require('./NovatelPosition.js');
let NovatelVelocity = require('./NovatelVelocity.js');
let NovatelMessageHeader = require('./NovatelMessageHeader.js');
let Time = require('./Time.js');

module.exports = {
  NovatelUtmPosition: NovatelUtmPosition,
  Trackstat: Trackstat,
  Gpgsv: Gpgsv,
  Range: Range,
  TrackstatChannel: TrackstatChannel,
  Gprmc: Gprmc,
  Inspva: Inspva,
  NovatelReceiverStatus: NovatelReceiverStatus,
  Gphdt: Gphdt,
  Gpgga: Gpgga,
  Gpgsa: Gpgsa,
  RangeInformation: RangeInformation,
  NovatelCorrectedImuData: NovatelCorrectedImuData,
  ClockSteering: ClockSteering,
  Inspvax: Inspvax,
  NovatelXYZ: NovatelXYZ,
  Inscov: Inscov,
  Insstdev: Insstdev,
  NovatelSignalMask: NovatelSignalMask,
  NovatelHeading2: NovatelHeading2,
  NovatelExtendedSolutionStatus: NovatelExtendedSolutionStatus,
  Satellite: Satellite,
  NovatelDualAntennaHeading: NovatelDualAntennaHeading,
  NovatelPosition: NovatelPosition,
  NovatelVelocity: NovatelVelocity,
  NovatelMessageHeader: NovatelMessageHeader,
  Time: Time,
};
