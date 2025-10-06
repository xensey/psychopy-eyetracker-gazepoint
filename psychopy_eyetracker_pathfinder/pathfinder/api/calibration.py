# -*- coding: utf-8 -*-
# Part of the PsychoPy library
# Copyright (C) 2012-2020 iSolver Software Solutions (C) 2021 Open Science Tools Ltd.
# Distributed under the terms of the GNU General Public License (GPL).
from psychopy.iohub.devices.eyetracker.calibration import BaseCalibrationProcedure


class PathfinderCalibrationProcedure(BaseCalibrationProcedure):
    def __init__(self, eyetrackerInterface, calibration_args):
        BaseCalibrationProcedure.__init__(self, eyetrackerInterface, calibration_args, allow_escape_in_progress=False)

    def startCalibrationHook(self):
        self._eyetracker._pathfinderset('CALIBRATE_SHOW', STATE=0)
        self._eyetracker._pathfinderset('CALIBRATE_START', STATE=0)

        self._eyetracker._pathfinderset('CALIBRATE_START', STATE=1)
