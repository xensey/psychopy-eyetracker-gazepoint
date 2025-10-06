# -*- coding: utf-8 -*-
# Part of the PsychoPy library
# Copyright (C) 2012-2020 iSolver Software Solutions (C) 2021 Open Science Tools Ltd.
# Distributed under the terms of the GNU General Public License (GPL).

import errno
import sys

from gevent import socket

from psychopy.iohub.devices.eyetracker.eye_events import *
from psychopy.iohub.devices import Computer, Device
from psychopy.iohub.devices.eyetracker import EyeTrackerDevice
from psychopy.iohub.devices.eyetracker.eye_events import EyeTrackerEvent
from psychopy.iohub.constants import EyeTrackerConstants, EventConstants
from psychopy.iohub.errors import print2err, printExceptionDetailsToStdErr
from psychopy.iohub.util import updateSettings

ET_UNDEFINED = EyeTrackerConstants.UNDEFINED
getTime = Computer.getTime

if sys.platform == 'win32':
    from ctypes import byref, c_int64, windll
    _fcounter_ = c_int64()
    _qpfreq_ = c_int64()
    windll.Kernel32.QueryPerformanceFrequency(byref(_qpfreq_))
    _qpfreq_ = float(_qpfreq_.value)
    _winQPC_ = windll.Kernel32.QueryPerformanceCounter

    def getLocalhostPathfinderTime():
        _winQPC_(byref(_fcounter_))
        return _fcounter_.value / _qpfreq_


def to_numeric(lit):
    """Return value of a numeric literal string. If the string can not be
    converted then the original string is returned.

    :param lit:
    :return:

    """
    # Handle '0'
    if lit == '0':
        return 0
    # Hex/Binary
    litneg = lit[1:] if lit[0] == '-' else lit
    if litneg[0] == '0':
        if litneg[1] in 'xX':
            return int(lit, 16)
        elif litneg[1] in 'bB':
            return int(lit, 2)
        else:
            try:
                return int(lit, 8)
            except ValueError:
                pass

    # Int/Float/Complex
    try:
        return int(lit)
    except ValueError:
        pass
    try:
        return float(lit)
    except ValueError:
        pass
    try:
        return complex(lit)
    except ValueError:
        pass

    # return original str
    return lit


class PathfinderSampleEvent(EyeTrackerEvent):
    """PathfinderSampleEvent contains the data collected from a Pathfinder sample,
    which can contain both Pathfinder eye and biometric data, depending on the
    hardware being used.

    Fields related to eye data are a subset of the standard BinocularEyeSampleEvent.

    Event Type ID: EventConstants.GAZEPOINT_SAMPLE

    Event Type String: 'GAZEPOINT_SAMPLE'
    
    """
    _newDataTypes = [
        ('left_gaze_x', 'f4'),
        ('left_gaze_y', 'f4'),
        ('left_raw_x', 'f4'),
        ('left_raw_y', 'f4'),
        ('left_pupil_measure1', 'f4'),
        ('left_pupil_measure1_type', 'u1'),
        ('left_pupil_measure2', 'f4'),
        ('left_pupil_measure2_type', 'u1'),
        ('right_gaze_x', 'f4'),
        ('right_gaze_y', 'f4'),
        ('right_raw_x', 'f4'),
        ('right_raw_y', 'f4'),
        ('right_pupil_measure1', 'f4'),
        ('right_pupil_measure1_type', 'u1'),
        ('right_pupil_measure2', 'f4'),
        ('right_pupil_measure2_type', 'u1'),
        ('status', 'u1')
    ]

    EVENT_TYPE_ID = EventConstants.GAZEPOINT_SAMPLE
    EVENT_TYPE_STRING = 'GAZEPOINT_SAMPLE'
    IOHUB_DATA_TABLE = EVENT_TYPE_STRING

    __slots__ = [e[0] for e in _newDataTypes]

    def __init__(self, *args, **kwargs):

        #: The calibrated horizontal left eye position on the calibration plane.
        #: This value is specified in Display Coordinate Type Units.
        self.left_gaze_x = None

        #: The calibrated vertical left eye position on the calibration plane.
        #: This value is specified in Display Coordinate Type Units.
        self.left_gaze_y = None

        #: The non-calibrated x position of the calculated left eye 'center'
        #: on the camera sensor image,
        #: factoring in any corneal reflection adjustments.
        #: This is typically reported in some arbitrary unit space that
        #: often has sub-pixel resolution due to image processing techniques
        #: being applied.
        self.left_raw_x = None

        #: The non-calibrated y position of the calculated left eye 'center'
        #: on the camera sensor image,
        #: factoring in any corneal reflection adjustments.
        #: This is typically reported in some arbitrary unit space that
        #: often has sub-pixel resolution due to image processing techniques
        #: being applied.
        self.left_raw_y = None

        #: A measure related to left pupil size or diameter. The attribute
        #: pupil_measure1_type defines what type the measure represents.
        self.left_pupil_measure1 = None

        #: * EyeTrackerConstants.PUPIL_DIAMETER
        self.left_pupil_measure1_type = None

        #: A second measure related to left pupil size or diameter. The attribute
        #: pupil_measure2_type defines what type the measure represents.
        self.left_pupil_measure2 = None

        #: The type of left pupil size or shape information provided in the pupil_measure2
        #: attribute. Several possible pupil_measure types available:
        #:
        #: * EyeTrackerConstants.PUPIL_DIAMETER_MM
        self.left_pupil_measure2_type = None

        #: The calibrated horizontal right eye position on the calibration plane.
        #: This value is specified in Display Coordinate Type Units.
        self.right_gaze_x = None

        #: The calibrated vertical right eye position on the calibration plane.
        #: This value is specified in Display Coordinate Type Units.
        self.right_gaze_y = None

        #: The non-calibrated x position of the calculated right eye 'center'
        #: on the camera sensor image,
        #: factoring in any corneal reflection adjustments.
        #: This is typically reported in some arbitrary unit space that
        #: often has sub-pixel resolution due to image processing techniques
        #: being applied.
        self.right_raw_x = None

        #: The non-calibrated y position of the calculated right eye 'center'
        #: on the camera sensor image,
        #: factoring in any corneal reflection adjustments.
        #: This is typically reported in some arbitrary unit space that
        #: often has sub-pixel resolution due to image processing techniques
        #: being applied.
        self.right_raw_y = None

        #: A measure related to right pupil size or diameter. The attribute
        #: pupil_measure1_type defines what type the measure represents.
        self.right_pupil_measure1 = None

        #: * EyeTrackerConstants.PUPIL_DIAMETER
        self.right_pupil_measure1_type = None

        #: A second measure related to right pupil size or diameter. The attribute
        #: pupil_measure2_type defines what type the measure represents.
        self.right_pupil_measure2 = None

        #: * EyeTrackerConstants.PUPIL_DIAMETER_MM
        self.right_pupil_measure2_type = None

        #: An available status byte for the eye tracker sample.
        #: Meaning is completely tracker dependent.
        self.status = None

        DeviceEvent.__init__(self, *args, **kwargs)


class EyeTracker(EyeTrackerDevice):
    """
    To start iohub with a Pathfinder eye tracker device, add a Pathfinder
    device to the device dictionary passed to launchHubServer or the 
    experiment's iohub_config.yaml::

        eyetracker.hw.pathfinder.api.EyeTracker

    .. note:: The Pathfinder control application **must** be running
              while using this interface.
              
    Examples:
        A. Start ioHub with a Pathfinder device and run tracker calibration::
    
            from psychopy.iohub import launchHubServer
            from psychopy.core import getTime, wait

            iohub_config = {'eyetracker.hw.pathfinder.api.EyeTracker':
                {'name': 'tracker', 'device_timer': {'interval': 0.005}}}
                
            io = launchHubServer(**iohub_config)
            
            # Get the eye tracker device.
            tracker = io.devices.tracker
                            
            # run eyetracker calibration
            r = tracker.runSetupProcedure()
            
        B. Print all eye tracker events received for 2 seconds::
                        
            # Check for and print any eye tracker events received...
            tracker.setRecordingState(True)
            
            stime = getTime()
            while getTime()-stime < 2.0:
                for e in tracker.getEvents():
                    print(e)
            
        C. Print current eye position for 5 seconds::
                        
            # Check for and print current eye position every 100 msec.
            stime = getTime()
            while getTime()-stime < 5.0:
                print(tracker.getPosition())
                wait(0.1)
            
            tracker.setRecordingState(False)
            
            # Stop the ioHub Server
            io.quit()
    """

    # Pathfinder tracker times are received as msec
    #
    DEVICE_TIMEBASE_TO_SEC = 1.0
    EVENT_CLASS_NAMES = [
        'PathfinderSampleEvent',
        'BinocularEyeSampleEvent']
    _recording = False
    __slots__ = ['_pathfinder', '_rx_buffer', '_serverIsLocalhost']

    def __init__(self, *args, **kwargs):
        EyeTrackerDevice.__init__(self, *args, **kwargs)

        # Holds the Pathfinder socket interface
        self._pathfinder = None

        # Holds data received from Pathfinder tracker that has not yet been parsed
        # into messages
        self._rx_buffer = ''

        # Used to hold the last sample processed by iohub.
        self._latest_sample = None

        # Used to hold the last valid gaze position processed by ioHub.
        # If the last sample received from the Pathfinder indicates missing eye
        # position, then this is set to None
        #
        self._latest_gaze_position = None

        # Connect to the eye tracker server by default.
        self.setConnectionState(True)
        self._serverIsLocalhost = self.getConfiguration().get('network_settings').get('ip_address') in ['localhost',
                                                                                                        '127.0.0.1']
    
          
    @staticmethod
    def getCalibrationDict(calib):
        """
        Create a dict describing the given Calibration object, respecting this 
        eyetracker's specific limitations.

        Parameters
        ----------
        calib : psychopy.hardware.eyetracker.EyetrackerCalibration
            Object to create a dict from
        
        Returns
        -------
        dict
            Dict describing the given Calibration object
        """
        # call base function
        asDict = EyeTrackerDevice.getCalibrationDict(calib)
        # add Pathfinder-specific attributes
        asDict['use_builtin'] = False
        asDict['target_delay'] = calib.targetDelay
        asDict['target_duration'] = calib.targetDur
        # remove unused attributes
        asDict.pop('auto_pace')
        asDict.pop('pacing_speed')

        return asDict

    def trackerTime(self):
        """
        Current eye tracker time in the eye tracker's native time base.
        The Pathfinder system uses a sec.usec timebase based on the Windows QPC,
        so when running on a single computer setup, iohub can directly read
        the current Pathfinder time. When running with a two computer setup,
        current Pathfinder time is assumed to equal current local time.

        Returns:
            float: current native eye tracker time in sec.msec format.
        """
        if sys.platform == 'win32' and self._serverIsLocalhost:
            return getLocalhostPathfinderTime()
        return getTime()

    def trackerSec(self):
        """
        Same as the Pathfinder implementation of trackerTime().
        """
        return self.trackerTime() * self.DEVICE_TIMEBASE_TO_SEC

    def _sendRequest(self, rtype, ID, **kwargs):
        params = ''
        for k, v in kwargs.items():
            params += ' {}="{}"'.format(k, v)
        rqstr = '<{} ID="{}" {} />\r\n'.format(rtype, ID, params)
        # print2err("Sending: {}\n".format(rqstr))
        self._pathfinder.sendall(str.encode(rqstr))

    def _pathfinderset(self, ID, **kwargs):
        self._sendRequest("SET", ID, **kwargs)

    def _pathfinderget(self, ID, **kwargs):
        self._sendRequest("GET", ID, **kwargs)

    def _waitForAck(self, type_id, timeout=5.0):
        stime = getTime()
        while getTime() - stime < timeout:
            self._checkForNetData(0.25)
            msgs = self._parseRxBuffer()
            for m in msgs:
                if m.get('ID') == type_id:
                    return m
        return None

    def _checkForNetData(self, timeout=0.0):
        self._pathfinder.settimeout(timeout)
        while True:
            try:
                rxdat = self._pathfinder.recv(4096)
                if rxdat:
                    self._rx_buffer += bytes.decode(rxdat).replace('\r\n', '')
                    return self._rx_buffer
                else:
                    print2err('***** Pathfinder Closed Connection *****')
                    # Connection closed
                    self.setRecordingState(False)
                    self.setConnectionState(False)
                    self._rx_buffer = ''
                    return None

            except socket.error as e:
                err = e.args[0]
                if err == errno.EAGAIN or err == errno.EWOULDBLOCK or err == 'timed out':
                    # non blocking socket found no data; it happens.
                    return self._rx_buffer
                else:
                    # a valid error occurred
                    print2err('***** _checkForNetData Error *****')
                    printExceptionDetailsToStdErr()
                    return self._rx_buffer

    def _parseRxBuffer(self):
        msgs = []
        while self._rx_buffer:
            msg_end_ix = self._rx_buffer.find('/>')
            if msg_end_ix >= 0:
                msgtxt = self._rx_buffer[:msg_end_ix]
                msg_start_ix = msgtxt.find('<')
                if len(msgtxt) > 1 and msg_start_ix >= 0:
                    msgtxt = msgtxt[msg_start_ix + 1:]
                    msgtoks = msgtxt.split()
                    if msgtoks:
                        msg = dict(type=msgtoks[0])
                        for t in msgtoks[1:]:
                            tkey, tval = t.split('=')
                            try:
                                msg[tkey] = to_numeric(tval.strip('"'))
                            except Exception:
                                msg[tkey] = tval
                        msgs.append(msg)
                else:
                    print2err('Incomplete Message Found: [', msgtxt, ']')
                self._rx_buffer = self._rx_buffer[msg_end_ix + 2:]
            else:
                break
        return msgs

    def setConnectionState(self, enable):
        """
        Connects or disconnects from the Pathfinder eye tracking hardware.

        By default, when ioHub is started, a connection is automatically made,
        and when the experiment completes and ioHub is closed, so is the Pathfinder
        connection.
        Args:
            enable (bool): True = enable the connection, False = disable the connection.

        Return:
            bool: indicates the current connection state to the eye tracking hardware.
        """
        if enable is True and self._pathfinder is None:
            try:
                self._rx_buffer = ''
                self._pathfinder = socket.socket()
                haddress = self.getConfiguration().get('network_settings').get('ip_address')
                hport = int(self.getConfiguration().get('network_settings').get('port'))
                address = (haddress, hport)
                self._pathfinder.connect(address)
                init_connection_str = ''
                init_connection_str += '<SET ID="ENABLE_SEND_POG_LEFT" STATE="1" />\r\n'
                init_connection_str += '<SET ID="ENABLE_SEND_POG_RIGHT" STATE="1" />\r\n'
                init_connection_str += '<SET ID="ENABLE_SEND_POG_BEST" STATE="1" />\r\n'
                init_connection_str += '<SET ID="ENABLE_SEND_PUPILMM" STATE="1" />\r\n'
                init_connection_str += '<SET ID="ENABLE_SEND_COUNTER" STATE="1" />\r\n'
                init_connection_str += '<SET ID="ENABLE_SEND_TIME" STATE="1" />\r\n'
                init_connection_str += '<SET ID="ENABLE_SEND_DATA" STATE="0" />\r\n'
                self._pathfinder.sendall(str.encode(init_connection_str))

                if self._waitForAck('ENABLE_SEND_TIME'):
                    self._rx_buffer = ''
                    return True
                else:
                    return False

            except socket.error as e:
                if e.args[0] == 10061:
                    print2err('***** Socket Error: Check Pathfinder control software is running *****')
                print2err('Error connecting to Pathfinder ', e)

        elif enable is False and self._pathfinder:
            try:
                if self._pathfinder:
                    self.setRecordingState(False)
                self._pathfinder.close()
                self._pathfinder = None
                self._rx_buffer = ''
            except Exception:
                print2err('Problem disconnecting from device - Pathfinder')
                self._rx_buffer = ''
        return self.isConnected()

    def isConnected(self):
        """
        isConnected returns whether the Pathfinder is connected to the experiment
        PC and if the tracker state is valid. Returns True if the tracker can
        be put into Record mode, etc and False if there is an error with the
        tracker or tracker connection with the experiment PC.

        Return:
            bool:  True = the eye tracking hardware is connected. False otherwise.
        """
        return self._pathfinder is not None

    def sendMessage(self, message_contents, time_offset=None):
        """
        The sendMessage method sends the message_contents str to the Pathfinder.
        """
        try:
            if time_offset is not None:
                print2err('Warning: Pathfinder EyeTracker.sendMessage time_offset argument is ignored.')
            if self._pathfinder and self.isRecordingEnabled() is True:
                # Pathfinder server does not accept USER_DATA messages, so skip sending
                pass
        except Exception:
            print2err('Problems sending message: {0}'.format(message_contents))
            printExceptionDetailsToStdErr()
        return EyeTrackerConstants.EYETRACKER_OK

    def enableEventReporting(self, enabled=True):
        """
        enableEventReporting is functionally identical to the eye tracker
        device specific setRecordingState method.
        """

        try:
            self.setRecordingState(enabled)
            enabled = EyeTrackerDevice.enableEventReporting(self, enabled)
            return enabled
        except Exception as e:
            print2err('Exception in EyeTracker.enableEventReporting: ', str(e))
            printExceptionDetailsToStdErr()

    def setRecordingState(self, recording):
        """
        setRecordingState is used to start or stop the recording of data from the eye tracking device.

        args:
           recording (bool): if True, the eye tracker will start recordng available
              eye data and sending it to the experiment program if data streaming
              was enabled for the device. If recording == False, then the eye
              tracker stops recording eye data and streaming it to the experiment.

        If the eye tracker is already recording, and setRecordingState(True) is
        called, the eye tracker will simple continue recording and the method call
        is a no-op. Likewise if the system has already stopped recording and
        setRecordingState(False) is called again.

        Args:
            recording (bool): if True, the eye tracker will start recordng data.; false = stop recording data.

        Return:trackerTime
            bool: the current recording state of the eye tracking device
        """
        current_state = self.isRecordingEnabled()
        if self._pathfinder and recording is True and current_state is False:
            self._rx_buffer = ''
            self._pathfinder.sendall(
                str.encode('<SET ID="ENABLE_SEND_DATA" STATE="1" />\r\n'))
            rxdat = self._checkForNetData(1.0)
            if rxdat is None:
                EyeTracker._recording = False
                return EyeTrackerDevice.enableEventReporting(self, False)
            EyeTracker._recording = True
        elif self._pathfinder and recording is False and current_state is True:
            self._rx_buffer = ''
            self._pathfinder.sendall(
                str.encode('<SET ID="ENABLE_SEND_DATA" STATE="0" />\r\n'))
            self._checkForNetData(1.0)
            EyeTracker._recording = False
            self._latest_sample = None
            self._latest_gaze_position = None
        return EyeTrackerDevice.enableEventReporting(self, recording)

    def isRecordingEnabled(self):
        """
        isRecordingEnabled returns the recording state from the eye tracking device.

        Return:
            bool: True == the device is recording data; False == Recording is not occurring
        """
        if self._pathfinder:
            return self._recording
        return False

    def runSetupProcedure(self, calibration_args={}):
        """
        Start the eye tracker calibration procedure.
        """
        cal_config = updateSettings(self.getConfiguration().get('calibration'), calibration_args)
        #print2err("pathfinder cal_config:", cal_config)

        use_builtin = cal_config.get('use_builtin')

        if use_builtin is True:
            self._pathfinderset('CALIBRATE_SHOW', STATE=1)
            self._pathfinderset('CALIBRATE_START', STATE=1)

        else:
            from .calibration import PathfinderCalibrationProcedure
            calibration = PathfinderCalibrationProcedure(self, calibration_args)

            calibration.runCalibration()

            calibration.window.close()

            # calibration._unregisterEventMonitors()
            calibration.clearAllEventBuffers()

        ok = False
        for _ in range(200):
            self._pathfinderget('LAST_CALIBR_RESULT')
            ack = self._waitForAck('LAST_CALIBR_RESULT', timeout=0.05)
            if ack and int(ack.get('STATE', 0)) == 1:
                ok = True
                break

        self._pathfinderset('CALIBRATE_SHOW', STATE=0)
        self._pathfinderset('CALIBRATE_START', STATE=0)

        summary = None
        if ok:
            self._pathfinderget('CALIBRATE_RESULT_SUMMARY')
            cal_summary = self._waitForAck('CALIBRATE_RESULT_SUMMARY', timeout=2.0)
            if cal_summary:
                summary = {k: v for k, v in cal_summary.items() if k not in ('type', 'ID')}

        result = {'OK': int(ok)}
        if summary is not None:
            result['SUMMARY'] = summary
        return result

    def _poll(self):
        """
        This method is called by iohub every n msec based on the polling interval set in the eye tracker config.
        """
        try:
            if not self.isRecordingEnabled():
                return

            logged_time = Computer.getTime()
            tracker_time = self.trackerTime()

            # Check for any new rx data from pathfinder socket.
            # If None is returned, that means the pathfinder closed the socket
            # connection.
            if self._checkForNetData() is None:
                return

            # Parse any rx text received from the pathfinder into msg dicts.
            msgs = self._parseRxBuffer()
            for m in msgs:
                if m.get('type') == 'REC':
                    binocSample = self._parseSampleFromMsg(m, logged_time, tracker_time)
                    self._addNativeEventToBuffer(binocSample)

                    # left / right eye pos avg. data
                    combined_gaze_x = m.get('BPOGX', ET_UNDEFINED)
                    combined_gaze_y = m.get('BPOGY', ET_UNDEFINED)
                    combined_gaze_x, combined_gaze_y = self._eyeTrackerToDisplayCoords(
                        (combined_gaze_x, combined_gaze_y))

                    if combined_gaze_x is not None and combined_gaze_y is not None:
                        self._latest_gaze_position = (combined_gaze_x, combined_gaze_y)
                    else:
                        self._latest_gaze_position = None

                elif m.get('type') == 'ACK':
                    pass  # print2err('ACK Received: ', m)
                else:
                    # Message type is not being handled.
                    print2err('UNHANDLED Pathfinder MESSAGE: ', m)

            self._last_poll_time = logged_time

        except Exception:
            print2err('ERROR occurred during Pathfinder Sample Callback.')
            printExceptionDetailsToStdErr()
        finally:
            return 0

    def _parseSampleFromMsg(self, m, logged_time, tracker_time):
        # Always use the gaze sample event type
        event_type = EventConstants.GAZEPOINT_SAMPLE

        # in seconds, take from the REC TIME field
        event_timestamp = m.get('TIME', ET_UNDEFINED)
        sample_delay = 0

        iohub_time = logged_time - sample_delay

        confidence_interval = logged_time - self._last_poll_time

        left_gaze_x, left_gaze_y = self._eyeTrackerToDisplayCoords(
            (m.get('LPOGX', ET_UNDEFINED), m.get('LPOGY', ET_UNDEFINED)))
        if left_gaze_x is None:
            left_gaze_x = ET_UNDEFINED
        if left_gaze_y is None:
            left_gaze_y = ET_UNDEFINED
        left_pupil_size = ET_UNDEFINED
        left_pupil_size_2 = m.get('LPMM', ET_UNDEFINED)

        right_gaze_x, right_gaze_y = self._eyeTrackerToDisplayCoords(
            (m.get('RPOGX', ET_UNDEFINED), m.get('RPOGY', ET_UNDEFINED)))
        if right_gaze_x is None:
            right_gaze_x = ET_UNDEFINED
        if right_gaze_y is None:
            right_gaze_y = ET_UNDEFINED
        right_pupil_size = ET_UNDEFINED
        right_pupil_size_2 = m.get('RPMM', ET_UNDEFINED)

        #
        # The X and Y-coordinates of the left and right eye pupil
        # in the camera image, as a fraction of the
        # camera image size.
        left_raw_x = ET_UNDEFINED
        left_raw_y = ET_UNDEFINED
        right_raw_x = ET_UNDEFINED
        right_raw_y = ET_UNDEFINED

        left_valid_raw = m.get('LPOGV', ET_UNDEFINED)
        right_valid_raw = m.get('RPOGV', ET_UNDEFINED)
        left_eye_status = int(left_valid_raw) if left_valid_raw not in (None, ET_UNDEFINED) else 0
        right_eye_status = int(right_valid_raw) if right_valid_raw not in (None, ET_UNDEFINED) else 0

        # 0 = both eyes OK
        status = 0
        if left_eye_status == right_eye_status and right_eye_status == 0:
            # both eyes are missing
            status = 22
        elif left_eye_status == 0:
            # Just left eye missing
            status = 20
        elif right_eye_status == 0:
            # Just right eye missing
            status = 2

        return [
            0,  # experiment_id, iohub fills in automatically
            0,  # session_id, iohub fills in automatically
            0,  # device id, keep at 0
            Device._getNextEventID(),  # iohub event unique ID
            event_type,  # BINOCULAR_EYE_SAMPLE
            event_timestamp,  # eye tracker device time stamp
            logged_time,  # time _poll is called
            iohub_time,
            confidence_interval,
            sample_delay,
            0,
            left_gaze_x,
            left_gaze_y,
            left_raw_x,
            left_raw_y,
            left_pupil_size,
            EyeTrackerConstants.PUPIL_DIAMETER,
            left_pupil_size_2,
            EyeTrackerConstants.PUPIL_DIAMETER_MM,
            right_gaze_x,
            right_gaze_y,
            right_raw_x,
            right_raw_y,
            right_pupil_size,
            EyeTrackerConstants.PUPIL_DIAMETER,
            right_pupil_size_2,
            EyeTrackerConstants.PUPIL_DIAMETER_MM,
            status
        ]

    def _getIOHubEventObject(self, native_event_data):
        """
        The _getIOHubEventObject method is called by the ioHub Process to
        convert new native device event objects that have been received to the
        appropriate ioHub Event type representation.
        """
        self._latest_sample = native_event_data
        return self._latest_sample

    def _eyeTrackerToDisplayCoords(self, eyetracker_point):
        """
        Converts Pathfinder gaze positions to the Display device coordinate space.
        """
        gaze_x, gaze_y = eyetracker_point
        if gaze_x in (None, ET_UNDEFINED) or gaze_y in (None, ET_UNDEFINED):
            return (None, None)
        left, top, right, bottom = self._display_device.getCoordBounds()
        w, h = right - left, top - bottom
        x, y = left + w * gaze_x, bottom + h * (1.0 - gaze_y)
        return x, y

    def _displayToEyeTrackerCoords(self, display_x, display_y):
        """
        Converts a Display device point to Pathfinder gaze position coordinate space.
        """
        left, top, right, bottom = self._display_device.getCoordBounds()
        w, h = right - left, top - bottom

        return (left - display_x) / w, (top - display_y) / h

    def _close(self):
        if self._pathfinder:
            self.setRecordingState(False)
            self.setConnectionState(False)
        EyeTrackerDevice._close(self)
