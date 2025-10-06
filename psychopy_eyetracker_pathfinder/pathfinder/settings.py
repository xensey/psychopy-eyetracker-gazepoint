from psychopy.localization import _translate
from psychopy.experiment import Param
from psychopy.experiment.components.settings.eyetracking import EyetrackerBackend


class PathfinderEyetrackerBackend(EyetrackerBackend):
    """Experiment settings for the Pathfinder eyetracker."""

    label = 'Pathfinder (iohub)'
    key = 'eyetracker.hw.pathfinder.api.EyeTracker'

    needsFullscreen = False
    needsCalibration = True

    @classmethod
    def getParams(cls):
        # define order
        order = [
            # network settings
            "pfNetIP4Address",
            "pfNetPort",
            # runtime settings
            "pfSamplingRate",
            "pfTrackEyes"
        ]

        params = {}
        params['pfNetIP4Address'] = Param(
            "127.0.0.1",   # default value
            valType='str',
            inputType="single",
            hint=_translate("IP Address to connect to."),
            label=_translate("IP4 Address"),
            categ="Eyetracking",
        )
        params['pfNetPort'] = Param(
            "4242",  # default value
            valType='str',
            inputType="single",
            hint=_translate("Port number to connect to."),
            label=_translate("Port"),
            categ="Eyetracking",
        )

        params['pfSamplingRate'] = Param(
            "60",
            valType='str',
            inputType="single",
            hint=_translate("Sampling rate in Hz."),
            label=_translate("Sampling rate"),
            categ="Eyetracking",
        )
        params['pfTrackEyes'] = Param(
            'BINOCULAR',
            valType='str',
            inputType="choice",
            allowedVals=['BINOCULAR'],
            hint=_translate("Which eye(s) to track."),
            label=_translate("Track eyes"),
            categ="Eyetracking",
        )

        return params, order

    @classmethod
    def writeDeviceCode(cls, inits, buff):
        code = (
            "ioConfig[%(eyetracker)s] = {\n"
            "    'name': 'tracker',\n"
            "    'device_number': 0,\n"
            "    'network_settings': {\n"
            "        'ip_address': %(pfNetIP4Address)r,\n"
            "        'port': int(%(pfNetPort)s),\n"
            "    },\n"
            "    'runtime_settings': {\n"
            "        'sampling_rate': int(%(pfSamplingRate)s)\n"
            "            if str(%(pfSamplingRate)s).isdigit() else %(pfSamplingRate)r,\n"
            "        'track_eyes': %(pfTrackEyes)r,\n"
            "    },\n"
            "}\n"
        )
        buff.writeIndentedLines(code % inits)
