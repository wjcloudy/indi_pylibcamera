"""
indi_pylibcamera: CameraControl class
"""
import os.path
import numpy as np
import io
import re
import threading
import time
import datetime

from astropy.io import fits
import astropy.coordinates
import astropy.units
import astropy.utils.iers

from PIL import Image

from picamera2 import Picamera2
from libcamera import controls, Rectangle


from .indidevice import *
from .SERRecorder import SERRecorder, SER_MONO, SER_RGB, SER_BGR


class CameraSettings:
    """exposure settings
    """

    def __init__(self):
        self.ExposureTime = None
        self.DoFastExposure = None
        self.DoRaw = None
        self.DoRgbMono = None
        self.ProcSize = None
        self.RawMode = None
        self.Binning = None
        self.camera_controls = None

    def update(self, ExposureTime, knownVectors, advertised_camera_controls, has_RawModes):
        self.ExposureTime = ExposureTime
        self.DoFastExposure = knownVectors["CCD_FAST_TOGGLE"]["INDI_ENABLED"].value == ISwitchState.ON
        self.DoRaw = knownVectors["CCD_CAPTURE_FORMAT"].get_OnSwitches()[0] in ["INDI_RAW", "RAW_MONO"]
        self.DoRgbMono = knownVectors["CCD_CAPTURE_FORMAT"]["INDI_MONO"].value == ISwitchState.ON if has_RawModes else False
        self.ProcSize = (
            int(knownVectors["CCD_PROCFRAME"]["WIDTH"].value),
            int(knownVectors["CCD_PROCFRAME"]["HEIGHT"].value)
        )
        self.RawMode = knownVectors["RAW_FORMAT"].get_SelectedRawMode() if has_RawModes else None
        self.Binning = (int(knownVectors["CCD_BINNING"]["HOR_BIN"].value), int(knownVectors["CCD_BINNING"]["VER_BIN"].value)
        )
        self.camera_controls = {
            "ExposureTime": int(ExposureTime * 1e6),
            "AnalogueGain": knownVectors["CCD_GAIN"]["GAIN"].value,
        }
        if "AeEnable" in advertised_camera_controls:
            self.camera_controls["AeEnable"] = knownVectors["CAMCTRL_AEENABLE"]["INDI_ENABLED"].value == ISwitchState.ON
        if "AeConstraintMode" in advertised_camera_controls:
            self.camera_controls["AeConstraintMode"] = {
                "NORMAL": controls.AeConstraintModeEnum.Normal,
                "HIGHLIGHT": controls.AeConstraintModeEnum.Highlight,
                "SHADOWS": controls.AeConstraintModeEnum.Shadows,
                "CUSTOM": controls.AeConstraintModeEnum.Custom,
            }[knownVectors["CAMCTRL_AECONSTRAINTMODE"].get_OnSwitches()[0]]
        if "AeExposureMode" in advertised_camera_controls:
            self.camera_controls["AeExposureMode"] = {
                "NORMAL": controls.AeExposureModeEnum.Normal,
                "SHORT": controls.AeExposureModeEnum.Short,
                "LONG": controls.AeExposureModeEnum.Long,
                "CUSTOM": controls.AeExposureModeEnum.Custom,
            }[knownVectors["CAMCTRL_AEEXPOSUREMODE"].get_OnSwitches()[0]]
        if "AeMeteringMode" in advertised_camera_controls:
            self.camera_controls["AeMeteringMode"] = {
                "CENTREWEIGHTED": controls.AeMeteringModeEnum.CentreWeighted,
                "SPOT": controls.AeMeteringModeEnum.Spot,
                "MATRIX": controls.AeMeteringModeEnum.Matrix,
                "CUSTOM": controls.AeMeteringModeEnum.Custom,
            }[knownVectors["CAMCTRL_AEMETERINGMODE"].get_OnSwitches()[0]]
        if "AfMode" in advertised_camera_controls:
            self.camera_controls["AfMode"] = {
                "MANUAL": controls.AfModeEnum.Manual,
                "AUTO": controls.AfModeEnum.Auto,
                "CONTINUOUS": controls.AfModeEnum.Continuous,
            }[knownVectors["CAMCTRL_AFMODE"].get_OnSwitches()[0]]
        if "AfMetering" in advertised_camera_controls:
            self.camera_controls["AfMetering"] = {
                "AUTO": controls.AfMeteringEnum.Auto,
                "WINDOWS": controls.AfMeteringEnum.Windows,
            }[knownVectors["CAMCTRL_AFMETERING"].get_OnSwitches()[0]]
        if "AfPause" in advertised_camera_controls:
            self.camera_controls["AfPause"] = {
                "DEFERRED": controls.AfPauseEnum.Deferred,
                "IMMEDIATE": controls.AfPauseEnum.Immediate,
                "RESUME": controls.AfPauseEnum.Resume,
            }[knownVectors["CAMCTRL_AFPAUSE"].get_OnSwitches()[0]]
        if "AfRange" in advertised_camera_controls:
            self.camera_controls["AfRange"] = {
                "NORMAL": controls.AfRangeEnum.Normal,
                "MACRO": controls.AfRangeEnum.Macro,
                "FULL": controls.AfRangeEnum.Full,
            }[knownVectors["CAMCTRL_AFRANGE"].get_OnSwitches()[0]]
        if "AfSpeed" in advertised_camera_controls:
            self.camera_controls["AfSpeed"] = {
                "NORMAL": controls.AfSpeedEnum.Normal,
                "FAST": controls.AfSpeedEnum.Fast,
            }[knownVectors["CAMCTRL_AFSPEED"].get_OnSwitches()[0]]
        if "AfTrigger" in advertised_camera_controls:
            self.camera_controls["AfTrigger"] = {
                "START": controls.AfTriggerEnum.Start,
                "CANCEL": controls.AfTriggerEnum.Cancel,
            }[knownVectors["CAMCTRL_AFTRIGGER"].get_OnSwitches()[0]]
        if "AwbEnable" in advertised_camera_controls:
            self.camera_controls["AwbEnable"] = knownVectors["CAMCTRL_AWBENABLE"]["INDI_ENABLED"].value == ISwitchState.ON
        if "AwbMode" in advertised_camera_controls:
            self.camera_controls["AwbMode"] = {
                "AUTO": controls.AwbModeEnum.Auto,
                "TUNGSTEN": controls.AwbModeEnum.Tungsten,
                "FLUORESCENT": controls.AwbModeEnum.Fluorescent,
                "INDOOR": controls.AwbModeEnum.Indoor,
                "DAYLIGHT": controls.AwbModeEnum.Daylight,
                "CLOUDY": controls.AwbModeEnum.Cloudy,
                "CUSTOM": controls.AwbModeEnum.Custom,
            }[knownVectors["CAMCTRL_AWBMODE"].get_OnSwitches()[0]]
        if "Brightness" in advertised_camera_controls:
            self.camera_controls["Brightness"] = knownVectors["CAMCTRL_BRIGHTNESS"]["BRIGHTNESS"].value
        if "ColourGains" in advertised_camera_controls:
            if not self.camera_controls["AwbEnable"]:
                self.camera_controls["ColourGains"] = (
                    knownVectors["CAMCTRL_COLOURGAINS"]["REDGAIN"].value,
                    knownVectors["CAMCTRL_COLOURGAINS"]["BLUEGAIN"].value,
                )
        if "Contrast" in advertised_camera_controls:
            self.camera_controls["Contrast"] = knownVectors["CAMCTRL_CONTRAST"]["CONTRAST"].value
        if "ExposureValue" in advertised_camera_controls:
            self.camera_controls["ExposureValue"] = knownVectors["CAMCTRL_EXPOSUREVALUE"]["EXPOSUREVALUE"].value
        if "NoiseReductionMode" in advertised_camera_controls:
                self.camera_controls["NoiseReductionMode"] = {
                "OFF": controls.draft.NoiseReductionModeEnum.Off,
                "FAST": controls.draft.NoiseReductionModeEnum.Fast,
                "HIGHQUALITY": controls.draft.NoiseReductionModeEnum.HighQuality,
            }[knownVectors["CAMCTRL_NOISEREDUCTIONMODE"].get_OnSwitches()[0]]
        if "Saturation" in advertised_camera_controls:
            if self.DoRgbMono:
                # mono exposures are a special case of RGB with saturation=0
                self.camera_controls["Saturation"] = 0.0
            else:
                self.camera_controls["Saturation"] = knownVectors["CAMCTRL_SATURATION"]["SATURATION"].value
        if "Sharpness" in advertised_camera_controls:
            self.camera_controls["Sharpness"] = knownVectors["CAMCTRL_SHARPNESS"]["SHARPNESS"].value

    def get_controls(self):
        return self.camera_controls

    def is_RestartNeeded(self, NewCameraSettings):
        """would using NewCameraSettings need a camera restart?
        """
        is_RestartNeeded = (
            self.is_ReconfigurationNeeded(NewCameraSettings)
            or (self.camera_controls != NewCameraSettings.camera_controls)
         )
        return is_RestartNeeded

    def is_ReconfigurationNeeded(self, NewCameraSettings):
        """would using NewCameraSettings need a camera reconfiguration?
        """
        is_ReconfigurationNeeded = (
            (self.DoFastExposure != NewCameraSettings.DoFastExposure)
            or (self.DoRaw != NewCameraSettings.DoRaw)
            or (self.ProcSize != NewCameraSettings.ProcSize)
            or (self.RawMode != NewCameraSettings.RawMode)
        )
        return is_ReconfigurationNeeded

    def __str__(self):
        return f'CameraSettings: FastExposure={self.DoFastExposure}, DoRaw={self.DoRaw}, ProcSize={self.ProcSize}, ' \
               f'RawMode={self.RawMode}, CameraControls={self.camera_controls}'

    def __repr__(self):
        return str(self)


def getLocalFileName(dir: str = ".", prefix: str = "Image_XXX", suffix: str = ".fits"):
    """make image name for local storage

    Valid placeholder in prefix are:
        _XXX: 3 digit image count
        _ISO8601: local time

    Args:
        dir: local directory, will be created if not existing
        prefix: file name prefix with placeholders
        suffix: file name suffix

    Returns:
        path and file name with placeholders dissolved
    """
    os.makedirs(dir, exist_ok=True)
    # replace ISO8601 placeholder in prefix with current time
    now = datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    prefix_now = prefix.replace("_ISO8601", f"_{now}")
    # find largest existing image index
    maxidx = 0
    patternstring = prefix_now.replace("_XXX", "_(?P<Idx>\d{3})", 1) + suffix
    patternstring = patternstring.replace(".", "\.")
    pattern = re.compile(patternstring)
    for fn in os.listdir(dir):
        match = pattern.fullmatch(fn)
        if match:
            if "Idx" in match.groupdict():
                idx = int(match.group("Idx"))
                maxidx = max(maxidx, idx)
    #
    maxidx += 1
    filename = prefix_now.replace("_XXX",f"_{maxidx:03d}", 1) + suffix
    return os.path.join(dir, filename)


class CameraControl:
    """camera control and exposure thread
    """

    def __init__(self, parent, config):
        self.parent = parent
        self.config = config
        # From time to time astropy downloads the latest IERS-A table from internet.
        # If offline this will raise an error. Here we disable the auto update.
        if self.config.getboolean("driver", "enable_IERS_autoupdate", fallback=True):
            astropy.utils.iers.conf.auto_max_age = None
            astropy.utils.iers.conf.iers_degraded_accuracy = "ignore"
        #
        self.do_CameraAdjustments = self.config.getboolean("driver", "CameraAdjustments", fallback=True)
        self.IgnoreRawModes = self.config.getboolean("driver", "IgnoreRawModes", fallback=False)
        # reset states
        self.picam2 = None
        self.present_CameraSettings = CameraSettings()
        self.CamProps = dict()
        self.RawModes = []
        self.min_ExposureTime = None
        self.max_ExposureTime = None
        self.min_AnalogueGain = None
        self.max_AnalogueGain = None
        self.camera_controls = dict()
        self.needs_Restarts = False
        # exposure loop control
        self.ExposureTime = 0.0
        self.Sig_Do = threading.Event() # do an action
        self.Sig_ActionExpose = threading.Event()  # single or fast exposure
        self.Sig_ActionExit = threading.Event()  # exit exposure loop
        self.Sig_ActionAbort = threading.Event()  # abort running exposure
        self.Sig_CaptureDone = threading.Event()
        # exposure loop in separate thread
        self.Sig_ActionExit.clear()
        self.Sig_ActionExpose.clear()
        self.Sig_Do.clear()
        self.Sig_ActionAbort.clear()
        self.ExposureThread = None
        # streaming state
        self.is_streaming = False
        self.StreamingThread = None
        self.Sig_StopStreaming = threading.Event()
        self.Sig_StopStreaming.clear()
        # recording state
        self.is_recording = False
        self.ser_recorder = SERRecorder()
        self.record_max_duration = 0.0  # seconds, 0 = unlimited
        self.record_max_frames = 0  # 0 = unlimited


    def closeCamera(self):
        """close camera
        """
        logger.info('closing camera')
        # stop streaming if active
        self.stopStreaming()
        # stop recording if active
        self.stopRecording()
        # stop exposure loop
        if self.ExposureThread is not None:
            if self.ExposureThread.is_alive():
                self.Sig_ActionExit.set()
                self.Sig_Do.set()
                self.ExposureThread.join(timeout=5.0)  # wait until exposure loop exits
                if self.ExposureThread.is_alive():
                    logger.warning('Exposure thread did not exit within timeout')
        # close picam2
        if self.picam2 is not None:
            if self.picam2.started:
                self.picam2.stop_()
            self.picam2.close()
        # reset states
        self.picam2 = None
        self.present_CameraSettings = CameraSettings()
        self.CamProps = dict()
        self.RawModes = []
        self.min_ExposureTime = None
        self.max_ExposureTime = None
        self.min_AnalogueGain = None
        self.max_AnalogueGain = None
        self.camera_controls = dict()


    def getRawCameraModes(self):
        """get list of usable raw camera modes
        """
        sensor_modes = self.picam2.sensor_modes
        raw_modes = []
        for sensor_mode in sensor_modes:
            # sensor_mode is dict
            # it must have key "format" (usually a packed data format) and can have
            # "unpacked" (unpacked data format)
            if "unpacked" not in sensor_mode.keys():
                sensor_format = sensor_mode["format"]
            else:
                sensor_format = sensor_mode["unpacked"]
            # packed data formats are not supported
            if sensor_format.endswith("_CSI2P"):
                logger.warning(f'raw mode not supported: {sensor_mode}')
                continue
            # only monochrome and Bayer pattern formats are supported
            is_monochrome = re.match("R[0-9]+", sensor_format)
            is_bayer = re.match("S[RGB]{4}[0-9]+", sensor_format)
            if not (is_monochrome or is_bayer):
                logger.warning(f'raw mode not supported: {sensor_mode}')
                continue
            #
            size = sensor_mode["size"]
            # adjustments for cameras:
            #   * zero- or garbage-filled columns
            #   * raw modes with binning or subsampling
            true_size = size
            binning = (1, 1)
            if self.do_CameraAdjustments:
                if self.CamProps["Model"] == 'imx477':
                    if size == (1332, 990):
                        true_size = (1332, 990)
                        binning = (2, 2)
                    elif size == (2028, 1080):
                        true_size = (2024, 1080)
                        binning = (2, 2)
                    elif size == (2028, 1520):
                        true_size = (2024, 1520)
                        binning = (2, 2)
                    elif size == (4056, 3040):
                        true_size = (4056, 3040)
                    else:
                        logger.warning(f'Unsupported frame size {size} for imx477!')
                elif self.CamProps["Model"] == 'ov5647':
                    if size == (640, 480):
                        binning = (4, 4)
                    elif size == (1296, 972):
                        binning = (2, 2)
                    elif size == (1920, 1080):
                        pass
                    elif size == (2592, 1944):
                        pass
                    else:
                        logger.warning(f'Unsupported frame size {size} for ov5647!')
                elif self.CamProps["Model"].startswith("imx708"):
                    if size == (1536, 864):
                        binning = (2, 2)
                    elif size == (2304, 1296):
                        binning = (2, 2)
                    elif size == (4608, 2592):
                        pass
                    else:
                        logger.warning(f'Unsupported frame size {size} for imx708!')
                elif self.CamProps["Model"] == 'imx415':
                    # IMX415: single raw mode 3864x2192, no binning, no garbage columns
                    if size == (3864, 2192):
                        pass
                    else:
                        logger.warning(f'Unsupported frame size {size} for imx415!')
            # add to list of raw formats
            raw_mode = {
                "label": f'{size[0]}x{size[1]} {sensor_format[1:5] if is_bayer else "mono"} {sensor_mode["bit_depth"]}bit',
                "size": size,
                "true_size": true_size,
                "camera_format": sensor_format,
                "bit_depth": sensor_mode["bit_depth"],
                "binning": binning,
            }
            raw_modes.append(raw_mode)
        # sort list of raw formats by size and bit depth in descending order
        raw_modes.sort(key=lambda k: k["size"][0] * k["size"][1] * 100 + k["bit_depth"], reverse=True)
        return raw_modes

    def openCamera(self, idx: int):
        """open camera with given index idx
        """
        logger.info("opening camera")
        self.picam2 = Picamera2(idx)
        # read camera properties
        self.CamProps = self.picam2.camera_properties
        logger.info(f'camera properties: {self.CamProps}')
        # force properties with values from config file
        if "UnitCellSize" not in self.CamProps:
            logger.warning("Camera properties do not have UnitCellSize value. Need to force from config file!")
        self.CamProps["UnitCellSize"] = (
            self.config.getint(
                "driver", "force_UnitCellSize_X",
                fallback=self.CamProps["UnitCellSize"][0] if "UnitCellSize" in self.CamProps else 1000
            ),
            self.config.getint(
                "driver", "force_UnitCellSize_Y",
                fallback=self.CamProps["UnitCellSize"][1] if "UnitCellSize" in self.CamProps else 1000
            )
        )
        # newer libcamera version return a libcamera.Rectangle here!
        if type(self.CamProps["PixelArrayActiveAreas"][0]) is Rectangle:
            Rect = self.CamProps["PixelArrayActiveAreas"][0]
            self.CamProps["PixelArrayActiveAreas"] = (Rect.x, Rect.y, Rect.width, Rect.height)
        # raw modes
        self.RawModes = self.getRawCameraModes()
        if self.IgnoreRawModes:
            self.RawModes = []
        # camera controls
        self.camera_controls = self.picam2.camera_controls
        # gain and exposure time range
        self.min_ExposureTime, self.max_ExposureTime, default_exp = self.camera_controls["ExposureTime"]
        self.min_AnalogueGain, self.max_AnalogueGain, default_again = self.camera_controls["AnalogueGain"]
        # workaround for cameras reporting max_ExposureTime=0 (IMX296)
        self.max_ExposureTime = self.max_ExposureTime if self.min_ExposureTime < self.max_ExposureTime else 1000.0e6
        self.max_AnalogueGain = self.max_AnalogueGain if self.min_AnalogueGain < self.max_AnalogueGain else 1000.0
        # INI switch to force camera restarts
        force_Restart = self.config.get("driver", "force_Restart", fallback="auto").lower()
        if force_Restart == "yes":
            logger.info("INI setting forces camera restart")
            self.needs_Restarts = True
        elif force_Restart == "no":
            logger.info("INI setting for camera restarts as needed")
            self.needs_Restarts = False
        else:
            if force_Restart != "auto":
                logger.warning(f'unknown INI value for camera restart: force_Restart={force_Restart}')
            self.needs_Restarts = self.CamProps["Model"] in ["imx290", "imx519"]
        # start exposure loop
        self.Sig_ActionExit.clear()
        self.Sig_ActionExpose.clear()
        self.Sig_Do.clear()
        self.ExposureThread = threading.Thread(target=self.__ExposureLoop)
        self.ExposureThread.start()

    def getProp(self, name):
        """return camera properties
        """
        return self.CamProps[name]

    def snooped_FitsHeader(self, binnedCellSize_nm):
        """created FITS header data from snooped data

        Example:
            FOCALLEN=            2.000E+03 / Focal Length (mm)
            APTDIA  =            2.000E+02 / Telescope diameter (mm)
            ROTATANG=            0.000E+00 / Rotator angle in degrees
            SCALE   =         1.598825E-01 / arcsecs per pixel
            SITELAT =         5.105000E+01 / Latitude of the imaging site in degrees
            SITELONG=         1.375000E+01 / Longitude of the imaging site in degrees
            AIRMASS =         1.643007E+00 / Airmass
            OBJCTAZ =         1.121091E+02 / Azimuth of center of image in Degrees
            OBJCTALT=         3.744145E+01 / Altitude of center of image in Degrees
            OBJCTRA = ' 4 36 07.37'        / Object J2000 RA in Hours
            OBJCTDEC= '16 30 26.02'        / Object J2000 DEC in Degrees
            RA      =         6.903072E+01 / Object J2000 RA in Degrees
            DEC     =         1.650723E+01 / Object J2000 DEC in Degrees
            PIERSIDE= 'WEST    '           / West, looking East
            EQUINOX =                 2000 / Equinox
            DATE-OBS= '2023-04-05T11:27:53.655' / UTC start date of observation
        """
        FitsHeader = {}
        #### FOCALLEN, APTDIA ####
        Aperture = self.parent.knownVectors["SCOPE_INFO"]["APERTURE"].value
        FocalLength = self.parent.knownVectors["SCOPE_INFO"]["FOCAL_LENGTH"].value
        FitsHeader.update({
            "FOCALLEN": (FocalLength, "[mm] Focal Length"),
            "APTDIA": (Aperture, "[mm] Telescope aperture/diameter")
        })
        #### SCALE ####
        if self.config.getboolean("driver", "extended_Metadata", fallback=False):
            # some telescope driver do not provide FOCAL_LENGTH and some capture software overwrite
            # FOCALLEN without recalculating SCALE --> trouble with plate solver
            if FocalLength > 0:
                FitsHeader["SCALE"] = (
                    0.206265 * binnedCellSize_nm / FocalLength,
                    "[arcsec/px] image scale"
                    )
        #### SITELAT, SITELONG ####
        Lat = self.parent.knownVectors["GEOGRAPHIC_COORD"]["LAT"].value
        Long = self.parent.knownVectors["GEOGRAPHIC_COORD"]["LONG"].value
        Height = self.parent.knownVectors["GEOGRAPHIC_COORD"]["ELEV"].value
        FitsHeader.update({
            "SITELAT": (Lat, "[deg] Latitude of the imaging site"),
            "SITELONG": (Long, "[deg] Longitude of the imaging site")
        })
        ####
        # TODO: "EQUATORIAL_COORD" (J2000 coordinates from mount) are not used!
        if False:
            J2000RA = self.parent.knownVectors["EQUATORIAL_COORD"]["RA"].value
            J2000DEC = self.parent.knownVectors["EQUATORIAL_COORD"]["DEC"].value
            FitsHeader.update({
                #("OBJCTRA", J2000.ra.to_string(unit=astropy.units.hour).replace("h", " ").replace("m", " ").replace("s", " "),
                # "Object J2000 RA in Hours"),
                #("OBJCTDEC", J2000.dec.to_string(unit=astropy.units.deg).replace("d", " ").replace("m", " ").replace("s", " "),
                # "Object J2000 DEC in Degrees"),
                "RA": (J2000RA, "[deg] Object J2000 RA"),
                "DEC": (J2000DEC, "[deg] Object J2000 DEC")
            })
            # TODO: What about AIRMASS, OBJCTAZ and OBJCTALT?
        #### AIRMASS, OBJCTAZ, OBJCTALT, OBJCTRA, OBJCTDEC, RA, DEC ####
        RA = self.parent.knownVectors["EQUATORIAL_EOD_COORD"]["RA"].value
        DEC = self.parent.knownVectors["EQUATORIAL_EOD_COORD"]["DEC"].value
        ObsLoc = astropy.coordinates.EarthLocation(
            lon=Long * astropy.units.deg, lat=Lat * astropy.units.deg, height=Height * astropy.units.meter
        )
        c = astropy.coordinates.SkyCoord(ra=RA * astropy.units.hourangle, dec=DEC * astropy.units.deg)
        cAltAz = c.transform_to(astropy.coordinates.AltAz(obstime=astropy.time.Time.now(), location=ObsLoc))
        J2000 = cAltAz.transform_to(astropy.coordinates.ICRS())
        FitsHeader.update({
            "AIRMASS" : (float(cAltAz.secz), "Airmass"),
            "OBJCTAZ" : (float(cAltAz.az/astropy.units.deg), "[deg] Azimuth of center of image"),
            "OBJCTALT": (float(cAltAz.alt/astropy.units.deg), "[deg] Altitude of center of image"),
            "OBJCTRA" : (J2000.ra.to_string(unit=astropy.units.hour).replace("h", " ").replace("m", " ").replace("s", " "), "[HMS] Object J2000 RA"),
            "OBJCTDEC": (J2000.dec.to_string(unit=astropy.units.deg).replace("d", " ").replace("m", " ").replace("s", " "), "[DMS] Object J2000 DEC"),
            "RA"      : (float(J2000.ra.degree), "[deg] Object J2000 RA"),
            "DEC"     : (float(J2000.dec.degree), "[deg] Object J2000 DEC"),
            "EQUINOX" : (2000.0, "[yr] Equinox")
        })
        #### PIERSIDE ####
        if self.parent.knownVectors["TELESCOPE_PIER_SIDE"]["PIER_WEST"].value == ISwitchState.ON:
            FitsHeader["PIERSIDE"] = ("WEST", "West, looking East")
        else:
            FitsHeader["PIERSIDE"] = ("EAST", "East, looking West")

        logger.debug("Finished collecting snooped data.")
        ####
        return FitsHeader

    def createRawFits(self, array, metadata):
        """
        creates raw image in FITS format

        Args:
            array: image data
            metadata: image metadata

        Returns:
            FITS HDUL
        """
        format = self.picam2.camera_configuration()["raw"]["format"]
        self.log_FrameInformation(array=array, metadata=metadata, format=format)
        # Handle Pi 5 PISP compressed transport formats (e.g. "GBRG_PISP_COMP1")
        # picamera2 decompresses these transparently, so the pixel data is normal Bayer.
        if "_PISP_COMP" in format:
            # Extract Bayer pattern from prefix: "GBRG_PISP_COMP1" -> "GBRG"
            pisp_bayer = format.split("_")[0]
            if re.match(r'^[RGBW]{4}$', pisp_bayer):
                # Treat as standard Bayer: prepend 'S' and append bit depth from raw mode config
                bit_depth = self.present_CameraSettings.RawMode["bit_depth"]
                format = f'S{pisp_bayer}{bit_depth}'
                logger.info(f'PISP compressed format mapped to: {format}')
            else:
                raise NotImplementedError(f'got unsupported PISP raw format {format}')
        # we expect uncompressed format here
        elif format.count("_") > 0:
            raise NotImplementedError(f'got unsupported raw image format {format}')
        # Bayer or mono format
        if format[0] == "S":
            # Bayer pattern format
            BayerPattern = format[1:5]
            BayerPattern = self.parent.config.get("driver", "force_BayerOrder", fallback=BayerPattern)
            bit_depth = int(format[5:])
        elif format[0] == "R":
            # mono camera
            BayerPattern = None
            bit_depth = int(format[1:])
        else:
            raise NotImplementedError(f'got unsupported raw image format {format}')
        # convert to 16 bit if needed
        if bit_depth > 8:
            array = array.view(np.uint16)
        else:
            array = array.view(np.uint8)
        # calculate RAW Mono if needed
        with self.parent.knownVectorsLock:
            is_RawMono = self.parent.knownVectors["CCD_CAPTURE_FORMAT"].get_OnSwitches()[0] == "RAW_MONO"
            if is_RawMono:
                if bit_depth < 16:
                    # old libcamera data format (right aligned)
                    bit_depth += 2
                    array = np.pad(array, pad_width=1, mode="constant", constant_values=0)
                else:
                    # new libcamera data format: 16b left aligned
                    array = np.pad(array, pad_width=1, mode="constant", constant_values=0) >> 2
                array = array[1:-1, 1:-1] + array[2:, 1:-1] + array[1:-1, 2:] + array[2:, 2:]
                BayerPattern = None
        # remove 0- or garbage-filled columns
        true_size = self.present_CameraSettings.RawMode["true_size"]
        array = array[0:true_size[1], 0:true_size[0]]
        # crop
        with self.parent.knownVectorsLock:
            array = self.parent.knownVectors["CCD_FRAME"].crop(
                array=array, arrayType="mono" if BayerPattern is None else "bayer"
            )
        # left adjust if needed
        if bit_depth > 8:
            bit_pix = 16
            array *= 2 ** (bit_pix - bit_depth)
        else:
            bit_pix = 8
            array *= 2 ** (bit_pix - bit_depth)
        # convert to FITS
        hdu = fits.PrimaryHDU(array)
        # avoid access conflicts to knownVectors
        with self.parent.knownVectorsLock:
            # determine frame type
            FrameType = self.parent.knownVectors["CCD_FRAME_TYPE"].get_OnSwitchesLabels()[0]
            # FITS header and metadata
            FitsHeader = {
                "BZERO": (2 ** (bit_pix - 1), "offset data range"),
                "BSCALE": (1, "default scaling factor"),
                "ROWORDER": ("TOP-DOWN", "Row order"),
                "INSTRUME": (self.parent.device, "CCD Name"),
                "TELESCOP": (self.parent.knownVectors["ACTIVE_DEVICES"]["ACTIVE_TELESCOPE"].value, "Telescope name"),
                **self.parent.knownVectors["FITS_HEADER"].FitsHeader,
                "EXPTIME": (metadata["ExposureTime"]/1e6, "[s] Total Exposure Time"),
                "FRAME": (FrameType, "Frame Type"),
                "IMAGETYP": (FrameType+" Frame", "Frame Type"),
                **self.snooped_FitsHeader(binnedCellSize_nm = self.getProp("UnitCellSize")[0] * self.present_CameraSettings.Binning[0]),
                "DATE-END": (datetime.datetime.fromtimestamp(metadata.get("FrameWallClock", time.time()*1e9)/1e9, tz=datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3],
                             "UTC time at end of observation"),
                "GAIN": (metadata.get("AnalogueGain", 0.0), "Gain"),
                "DGAIN": (metadata.get("DigitalGain", 0.0), "Digital Gain"),
            }
            if self.config.getboolean("driver", "extended_Metadata", fallback=False):
                # This is very detailed information about the camera binning. But some plate solver ignore this and get
                # trouble with a wrong field of view.
                FitsHeader.update({
                    "PIXSIZE1": (self.getProp("UnitCellSize")[0] / 1e3, "[um] Pixel Size 1"),
                    "PIXSIZE2": (self.getProp("UnitCellSize")[1] / 1e3, "[um] Pixel Size 2"),
                    "XBINNING": (self.present_CameraSettings.Binning[0], "Binning factor in width"),
                    "YBINNING": (self.present_CameraSettings.Binning[1], "Binning factor in height"),
                    "XPIXSZ": (self.getProp("UnitCellSize")[0] / 1e3 * self.present_CameraSettings.Binning[0],
                               "[um] X binned pixel size"),
                    "YPIXSZ": (self.getProp("UnitCellSize")[1] / 1e3 * self.present_CameraSettings.Binning[1],
                               "[um] Y binned pixel size"),
                })
            else:
                # Pretend to be a camera without binning to avoid trouble with plate solver.
                FitsHeader.update({
                    "PIXSIZE1": (self.getProp("UnitCellSize")[0] / 1e3 * self.present_CameraSettings.Binning[0],
                                 "[um] Pixel Size 1"),
                    "PIXSIZE2": (self.getProp("UnitCellSize")[1] / 1e3 * self.present_CameraSettings.Binning[1],
                                 "[um] Pixel Size 2"),
                    "XBINNING": (1, "Binning factor in width"),
                    "YBINNING": (1, "Binning factor in height"),
                    "XPIXSZ": (self.getProp("UnitCellSize")[0] / 1e3 * self.present_CameraSettings.Binning[0],
                               "[um] X binned pixel size"),
                    "YPIXSZ": (self.getProp("UnitCellSize")[1] / 1e3 * self.present_CameraSettings.Binning[1],
                               "[um] Y binned pixel size"),
                })
        if 'SensorTemperature' in metadata:
            FitsHeader["CCD-TEMP"] = (metadata['SensorTemperature'], "[degC] CCD Temperature")
        if BayerPattern is not None:
            FitsHeader.update({
                "XBAYROFF": (0, "[px] X offset of Bayer array"),
                "YBAYROFF": (0, "[px] Y offset of Bayer array"),
                "BAYERPAT": (BayerPattern, "Bayer color pattern"),
            })
        if "SensorBlackLevels" in metadata:
            SensorBlackLevels = metadata["SensorBlackLevels"]
            if (len(SensorBlackLevels) == 4) and not is_RawMono:
                # according to picamera2 documentation:
                #   "The black levels of the raw sensor image. This
                #    control appears only in captured image
                #    metadata and is read-only. One value is
                #    reported for each of the four Bayer channels,
                #    scaled up as if the full pixel range were 16 bits
                #    (so 4096 represents a black level of 16 in 10-
                #    bit raw data)."
                # When image data is stored as 16bit it is not needed to scale SensorBlackLevels again.
                # But when we store image with 8bit/pixel we need to divide by 2**8.
                SensorBlackLevelScaling = 2 ** (bit_pix - 16)
                FitsHeader.update({
                    "OFFSET_0": (SensorBlackLevels[0] * SensorBlackLevelScaling, "[DN] Sensor Black Level 0"),
                    "OFFSET_1": (SensorBlackLevels[1] * SensorBlackLevelScaling, "[DN] Sensor Black Level 1"),
                    "OFFSET_2": (SensorBlackLevels[2] * SensorBlackLevelScaling, "[DN] Sensor Black Level 2"),
                    "OFFSET_3": (SensorBlackLevels[3] * SensorBlackLevelScaling, "[DN] Sensor Black Level 3"),
                })
        for kw, value_comment in FitsHeader.items():
            hdu.header[kw] = value_comment # astropy appropriately sets value and comment from tuple
        hdu.header.set("DATE-OBS", (datetime.datetime.fromisoformat(hdu.header["DATE-END"])-datetime.timedelta(seconds=hdu.header["EXPTIME"])).isoformat(timespec="milliseconds"),
                       "UTC time of observation start", before="DATE-END")
        hdul = fits.HDUList([hdu])
        return hdul

    def createRgbFits(self, array, metadata):
        """creates RGB and monochrome FITS image from RGB frame

        Args:
            array: data array
            metadata: metadata
        """
        format = self.picam2.camera_configuration()["main"]["format"]
        self.log_FrameInformation(array=array, metadata=metadata, format=format)
        # first dimension must be the color channels of one pixel
        array = array.transpose([2, 0, 1])
        if format == "BGR888":
            # each pixel is laid out as [R, G, B]
            pass
        elif format == "RGB888":
            # each pixel is laid out as [B, G, R]
            array = array[[2, 1, 0], :, :]
        elif format == "XBGR8888":
            # each pixel is laid out as [R, G, B, A] with A = 255
            array = array[[0, 1, 2], :, :]
        elif format == "XRGB8888":
            # each pixel is laid out as [B, G, R, A] with A = 255
            array = array[[2, 1, 0], :, :]
        else:
            raise NotImplementedError(f'got unsupported RGB image format {format}')
        #self.log_FrameInformation(array=array, metadata=metadata, is_raw=False)
        if self.present_CameraSettings.DoRgbMono:
            # monochrome frames are a special case of RGB: exposed with saturation=0, transmitted is R channel only
            array = array[0, :, :]
        # crop
        with self.parent.knownVectorsLock:
            array = self.parent.knownVectors["CCD_FRAME"].crop(
                array=array, arrayType="mono" if self.present_CameraSettings.DoRgbMono else "rgb"
            )
        # convert to FITS
        hdu = fits.PrimaryHDU(array)
        # The image scaling in the ISP works like a software-binning.
        # When aspect ratio of the scaled image differs from the pixel array the ISP ignores rows (columns) on
        # both sides of the pixel array to select the field of view.
        ArraySize = self.getProp("PixelArraySize")
        FrameSize = self.picam2.camera_configuration()["main"]["size"]
        SoftwareBinning = ArraySize[1] / FrameSize[1] if (ArraySize[0] / ArraySize[1]) > (FrameSize[0] / FrameSize[1]) \
            else ArraySize[0] / FrameSize[0]
        # avoid access conflicts to knownVectors
        with self.parent.knownVectorsLock:
            # determine frame type
            FrameType = self.parent.knownVectors["CCD_FRAME_TYPE"].get_OnSwitchesLabels()[0]
            # FITS header and metadata
            FitsHeader = {
                # "CTYPE3": 'RGB',  # Is that needed to make it a RGB image?
                "BZERO": (0, "offset data range"),
                "BSCALE": (1, "default scaling factor"),
                "DATAMAX": 255,
                "DATAMIN": 0,
                #"ROWORDER": ("TOP-DOWN", "Row Order"),
                "INSTRUME": (self.parent.device, "CCD Name"),
                "TELESCOP": (self.parent.knownVectors["ACTIVE_DEVICES"]["ACTIVE_TELESCOPE"].value, "Telescope name"),
                **self.parent.knownVectors["FITS_HEADER"].FitsHeader,
                "EXPTIME": (metadata["ExposureTime"]/1e6, "[s] Total Exposure Time"),
                "FRAME": (FrameType, "Frame Type"),
                "IMAGETYP": (FrameType+" Frame", "Frame Type"),
                **self.snooped_FitsHeader(binnedCellSize_nm = self.getProp("UnitCellSize")[0] * SoftwareBinning),
                "DATE-END": (datetime.datetime.fromtimestamp(metadata.get("FrameWallClock", time.time()*1e9)/1e9, tz=datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3],
                             "UTC time at end of observation"),
                # more info from camera
                "GAIN": (metadata.get("AnalogueGain", 0.0), "Analog gain setting"),
                "DGAIN": (metadata.get("DigitalGain", 0.0), "Digital Gain"),
            }
            if self.config.getboolean("driver", "extended_Metadata", fallback=False):
                # This is very detailed information about the camera binning. But some plate solver ignore this and get
                # trouble with a wrong field of view.
                FitsHeader.update({
                    "PIXSIZE1": (self.getProp("UnitCellSize")[0] / 1e3, "[um] Pixel Size 1"),
                    "PIXSIZE2": (self.getProp("UnitCellSize")[1] / 1e3, "[um] Pixel Size 2"),
                    "XBINNING": (SoftwareBinning, "Binning factor in width"),
                    "YBINNING": (SoftwareBinning, "Binning factor in height"),
                    "XPIXSZ": (self.getProp("UnitCellSize")[0] / 1e3 * SoftwareBinning, "[um] X binned pixel size"),
                    "YPIXSZ": (self.getProp("UnitCellSize")[1] / 1e3 * SoftwareBinning, "[um] Y binned pixel size"),
                })
            else:
                # Pretend to be a camera without binning to avoid trouble with plate solver.
                FitsHeader.update({
                    "PIXSIZE1": (self.getProp("UnitCellSize")[0] / 1e3 * SoftwareBinning, "[um] Pixel Size 1"),
                    "PIXSIZE2": (self.getProp("UnitCellSize")[1] / 1e3 * SoftwareBinning, "[um] Pixel Size 2"),
                    "XBINNING": (1, "Binning factor in width"),
                    "YBINNING": (1, "Binning factor in height"),
                    "XPIXSZ": (self.getProp("UnitCellSize")[0] / 1e3 * SoftwareBinning, "[um] X binned pixel size"),
                    "YPIXSZ": (self.getProp("UnitCellSize")[1] / 1e3 * SoftwareBinning, "[um] Y binned pixel size"),

                })
        if 'SensorTemperature' in metadata:
            FitsHeader["CCD-TEMP"] = (metadata['SensorTemperature'], "[degC] CCD Temperature")
        for kw, value_comment in FitsHeader.items():
            hdu.header[kw] = value_comment
        hdu.header.set("DATE-OBS", (datetime.datetime.fromisoformat(hdu.header["DATE-END"])-datetime.timedelta(seconds=hdu.header["EXPTIME"])).isoformat(timespec="milliseconds"),
                       "UTC time of observation start", before="DATE-END")
        hdul = fits.HDUList([hdu])
        return hdul

    def log_FrameInformation(self, array, metadata, format):
        """write frame information to log

        Args:
            array: raw frame data
            metadata: frame metadata
            format: format string
        """
        if self.config.getboolean("driver", "log_FrameInformation", fallback=False):
            if array.ndim == 2:
                arr = array.view(np.uint16)
                BitUsages = list()
                for b in range(15, -1, -1):
                    BitSlice = (arr & (1 << b)) != 0
                    BitUsage = BitSlice.sum() / arr.size
                    BitUsages.append(BitUsage)
                BitUsages = [f'{bu:.1e}' for bu in BitUsages]
                logger.info(f'Frame format: {format}, shape: {array.shape} {array.dtype}, bit usages: (MSB) {" ".join(BitUsages)} (LSB)')
            else:
                logger.info(f'Frame format: {format}, shape: {array.shape} {array.dtype}')
            logger.info(f'Frame metadata: {metadata}')

    def __ExposureLoop(self):
        """exposure loop

        Made to run in a separate thread.

        typical communications between client and device:
            start single exposure:
              new CCD_EXPOSURE_VALUE 1
              set CCD_EXPOSURE_VALUE 1 Busy
              set CCD_EXPOSURE_VALUE 0.1 Busy
              set CCD_EXPOSURE_VALUE 0 Busy
              set CCD1 blob Ok
              set CCD_EXPOSURE_VALUE 0 Ok
            start Fast Exposure:
              new CCD_FAST_COUNT 100000
              set CCD_FAST_COUNT 100000 Ok
              new CCD_EXPOSURE_VALUE 1
              set CCD_EXPOSURE_VALUE 1 Busy
              set CCD_EXPOSURE_VALUE 0.1 Busy
              set CCD_EXPOSURE_VALUE 0 Busy
              set CCD_FAST_COUNT 99999 Busy
              set CCD_EXPOSURE_VALUE 0 Busy
              set CCD1 blob
              set CCD_EXPOSURE_VALUE 0 Ok
              set CCD_EXPOSURE_VALUE 0 Busy
              set CCD_FAST_COUNT 99998 Busy
              set CCD_EXPOSURE_VALUE 0 Busy
              set CCD1 blob
            abort:
              new CCD_ABORT_EXPOSURE On
              set CCD_FAST_COUNT 1, Idle
              set CCD_ABORT_EXPOSURE Off, Ok
        """
        while True:
            with self.parent.knownVectorsLock:
                DoFastExposure = self.parent.knownVectors["CCD_FAST_TOGGLE"]["INDI_ENABLED"].value == ISwitchState.ON
                FastCount_Frames = self.parent.knownVectors["CCD_FAST_COUNT"]["FRAMES"].value
            if not DoFastExposure or (FastCount_Frames < 1):
                # prepare for next exposure
                if FastCount_Frames < 1:
                    self.parent.setVector("CCD_FAST_COUNT", "FRAMES", value=1, state=IVectorState.OK)
                # wait for next action
                self.Sig_ActionAbort.clear()
                self.Sig_Do.wait()
                self.Sig_Do.clear()
            if self.Sig_ActionExpose.is_set():
                self.Sig_ActionExpose.clear()
            if self.Sig_ActionExit.is_set():
                # exit exposure loop
                self.picam2.stop_()
                self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IVectorState.OK)
                return
            # picam2 needs to be open!
            if self.picam2 is None:
                raise RuntimeError("trying to make an exposure without camera opened")
            # get new camera settings for exposure
            has_RawModes = len(self.RawModes) > 0
            NewCameraSettings = CameraSettings()
            with self.parent.knownVectorsLock:
                NewCameraSettings.update(
                    ExposureTime=self.ExposureTime,
                    knownVectors=self.parent.knownVectors,
                    advertised_camera_controls=self.camera_controls,
                    has_RawModes=has_RawModes,
                )
            logger.info(f'exposure settings: {NewCameraSettings}')
            # need a camera stop/start when something has changed on exposure controls
            IsRestartNeeded = self.present_CameraSettings.is_RestartNeeded(NewCameraSettings) or self.needs_Restarts
            if self.picam2.started and IsRestartNeeded:
                logger.info(f'stopping camera for deeper reconfiguration')
                self.picam2.stop_()
            # RawMode, Raw/Processed or processed frame size has changed: need to reset grop settings
            if self.present_CameraSettings.is_ReconfigurationNeeded(NewCameraSettings):
                with self.parent.knownVectorsLock:
                    self.parent.knownVectors["CCD_FRAME"].reset()
            # change of DoFastExposure needs a configuration change
            if self.present_CameraSettings.is_ReconfigurationNeeded(NewCameraSettings) or self.needs_Restarts:
                logger.info(f'reconfiguring camera')
                # need a new camera configuration
                config = self.picam2.create_still_configuration(
                    queue=NewCameraSettings.DoFastExposure,
                    buffer_count=2  # 2 if NewCameraSettings.DoFastExposure else 1  # need at least 2 buffer for queueing
                )
                if NewCameraSettings.DoRaw:
                    # we do not need the main stream and configure it to smaller size to save memory
                    config["main"]["size"] = (240, 190)
                    # configure raw stream
                    config["raw"] = {"size": NewCameraSettings.RawMode["size"], "format": NewCameraSettings.RawMode["camera_format"]}
                    # libcamera internal binning does not change sensor array mechanical dimensions!
                    #self.parent.setVector("CCD_FRAME", "WIDTH", value=NewCameraSettings.RawMode["size"][0], send=False)
                    #self.parent.setVector("CCD_FRAME", "HEIGHT", value=NewCameraSettings.RawMode["size"][1])
                else:
                    config["main"]["size"] = NewCameraSettings.ProcSize
                    config["main"]["format"] = "BGR888"
                    # software image scaling does not change sensor array mechanical dimensions!
                    #self.parent.setVector("CCD_FRAME", "WIDTH", value=NewCameraSettings.ProcSize[0], send=False)
                    #self.parent.setVector("CCD_FRAME", "HEIGHT", value=NewCameraSettings.ProcSize[1])
                # optimize (align) configuration: small changes to some main stream configurations
                # (for instance: size) will fit better to hardware
                self.picam2.align_configuration(config)
                # set still configuration
                self.picam2.configure(config)
            # changing exposure time or analogue gain needs a restart
            if IsRestartNeeded:
                # change camera controls
                self.picam2.set_controls(NewCameraSettings.get_controls())
            # start camera if not already running in Fast Exposure mode
            if not self.picam2.started:
                self.picam2.start()
                logger.debug(f'camera started')
            # camera runs now with new parameter
            self.present_CameraSettings = NewCameraSettings
            # last chance to exit or abort before doing exposure
            if self.Sig_ActionExit.is_set():
                # exit exposure loop
                self.picam2.stop_()
                self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IVectorState.OK)
                return
            if self.Sig_ActionAbort.is_set():
                self.Sig_ActionAbort.clear()
            else:
                # get (non-blocking!) frame and meta data
                self.Sig_CaptureDone.clear()
                ExpectedEndOfExposure = time.time() + self.present_CameraSettings.ExposureTime
                job = self.picam2.capture_arrays(
                    ["raw" if self.present_CameraSettings.DoRaw else "main"],
                    wait=False, signal_function=self.on_CaptureFinished,
                )
                with self.parent.knownVectorsLock:
                    PollingPeriod_s = self.parent.knownVectors["POLLING_PERIOD"]["PERIOD_MS"].value / 1e3
                Abort = False
                while ExpectedEndOfExposure - time.time() > PollingPeriod_s:
                    # exposure count down
                    self.parent.setVector(
                        "CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=ExpectedEndOfExposure - time.time(),
                        state=IVectorState.BUSY
                    )
                    # allow to close camera
                    if self.Sig_ActionExit.is_set():
                        # exit exposure loop
                        self.picam2.stop_()
                        self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IVectorState.OK)
                        return
                    # allow to abort exposure
                    Abort = self.Sig_ActionAbort.is_set()
                    if Abort:
                        self.picam2.stop_()  # stop exposure immediately
                        self.Sig_ActionAbort.clear()
                        break
                    # allow exposure to finish earlier than expected (for instance when in fast exposure mode)
                    if self.Sig_CaptureDone.is_set():
                        break
                    time.sleep(PollingPeriod_s)
                # get frame and its metadata
                if not Abort:
                    (array, ), metadata = self.picam2.wait(job)
                    logger.info('got exposed frame')
                    # update CCD temperature only if the sensor actually reports it
                    if 'SensorTemperature' in metadata:
                        self.parent.setVector("CCD_TEMPERATURE", "CCD_TEMPERATURE_VALUE",
                                              value=metadata['SensorTemperature'])
                    # inform client about progress
                    self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IVectorState.BUSY)
                # last chance to exit or abort before sending blob
                if self.Sig_ActionExit.is_set():
                    # exit exposure loop
                    self.picam2.stop_()
                    self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IVectorState.OK)
                    return
                if self.Sig_ActionAbort.is_set():
                    self.Sig_ActionAbort.clear()
                    Abort = True
                with self.parent.knownVectorsLock:
                    DoFastExposure = self.parent.knownVectors["CCD_FAST_TOGGLE"]["INDI_ENABLED"].value == ISwitchState.ON
                    FastCount_Frames = self.parent.knownVectors["CCD_FAST_COUNT"]["FRAMES"].value
                if not DoFastExposure:
                    # in normal exposure mode the camera needs to be started with exposure command
                    self.picam2.stop()
                if not Abort:
                    if DoFastExposure:
                        FastCount_Frames -= 1
                        self.parent.setVector("CCD_FAST_COUNT", "FRAMES", value=FastCount_Frames, state=IVectorState.BUSY)
                    # create FITS images
                    if self.present_CameraSettings.DoRaw:
                        hdul = self.createRawFits(array=array, metadata=metadata)
                    else:
                        # RGB and Mono
                        hdul = self.createRgbFits(array=array, metadata=metadata)
                    bstream = io.BytesIO()
                    hdul.writeto(bstream)
                    # free up some memory
                    del hdul
                    del array
                    # save and/or transmit frame
                    size = bstream.tell()
                    # what to do with image
                    with self.parent.knownVectorsLock:
                        tv = self.parent.knownVectors["UPLOAD_SETTINGS"]
                        upload_dir = tv["UPLOAD_DIR"].value
                        upload_prefix = tv["UPLOAD_PREFIX"].value
                        upload_mode = self.parent.knownVectors["UPLOAD_MODE"].get_OnSwitches()
                    if upload_mode[0] in ["UPLOAD_LOCAL", "UPLOAD_BOTH"]:
                        # requested to save locally
                        local_filename = getLocalFileName(dir=upload_dir, prefix=upload_prefix, suffix=".fits")
                        bstream.seek(0)
                        logger.info(f"saving image to file {local_filename}")
                        with open(local_filename, 'wb') as fh:
                            fh.write(bstream.getbuffer())
                        self.parent.setVector("CCD_FILE_PATH", "FILE_PATH", value=local_filename, state=IVectorState.OK)
                    if upload_mode[0] in ["UPLOAD_CLIENT", "UPLOAD_BOTH"]:
                        # send blob to client
                        bstream.seek(0)
                        # make BLOB
                        logger.info(f"preparing frame as BLOB: {size} bytes")
                        bv = self.parent.knownVectors["CCD1"]
                        compress = self.parent.knownVectors["CCD_COMPRESSION"]["CCD_COMPRESS"].value == ISwitchState.ON
                        bv["CCD1"].set_data(data=bstream.getbuffer(), format=".fits", compress=compress)
                        logger.info(f"sending BLOB")
                        bv.send_setVector()
                    # tell client that we finished exposure
                    if DoFastExposure:
                        if FastCount_Frames == 0:
                            self.parent.setVector("CCD_FAST_COUNT", "FRAMES", value=0, state=IVectorState.OK)
                            self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IVectorState.OK)
                    else:
                        self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IVectorState.OK)

    def on_CaptureFinished(self, Job):
        """callback function for capture done
        """
        self.Sig_CaptureDone.set()

    def startExposure(self, exposuretime):
        """start a single or fast exposure

        Args:
            exposuretime: exposure time (seconds)
        """
        if self.is_streaming:
            logger.warning("Cannot start exposure while streaming is active. Stop streaming first.")
            self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IVectorState.ALERT)
            return
        if not self.ExposureThread.is_alive():
            raise RuntimeError("Try ro start exposure without having exposure loop running!")
        self.ExposureTime = exposuretime
        self.Sig_ActionExpose.set()
        self.Sig_ActionAbort.clear()
        self.Sig_ActionExit.clear()
        self.Sig_Do.set()

    def abortExposure(self):
        self.Sig_ActionExpose.clear()
        self.Sig_ActionAbort.set()

    # ---- Video Streaming ----

    def startStreaming(self):
        """Start live video streaming.

        Stops the still-exposure loop, reconfigures picamera2 for video,
        and launches the streaming thread.
        """
        if self.is_streaming:
            logger.warning("Streaming already active")
            return
        if self.picam2 is None:
            logger.error("Cannot start streaming: camera not open")
            return

        logger.info("Starting video stream")

        try:
            # Stop exposure loop so we own the camera
            if self.picam2.started:
                self.picam2.stop_()

            # Read streaming settings
            with self.parent.knownVectorsLock:
                stream_exp = self.parent.knownVectors["STREAMING_EXPOSURE"]["STREAMING_EXPOSURE_VALUE"].value
                stream_w = int(self.parent.knownVectors["CCD_STREAM_FRAME"]["WIDTH"].value)
                stream_h = int(self.parent.knownVectors["CCD_STREAM_FRAME"]["HEIGHT"].value)

            # Cap stream resolution for performance — full sensor res is too slow for live video
            max_stream_dim = 1280
            if stream_w > max_stream_dim or stream_h > max_stream_dim:
                scale = max_stream_dim / max(stream_w, stream_h)
                stream_w = int(stream_w * scale) & ~1  # ensure even
                stream_h = int(stream_h * scale) & ~1
                logger.info(f"Stream resolution capped to {stream_w}x{stream_h} for performance")
                # Update the INDI property so the client shows the actual values
                self.parent.setVector("CCD_STREAM_FRAME", "WIDTH", value=stream_w, send=False)
                self.parent.setVector("CCD_STREAM_FRAME", "HEIGHT", value=stream_h, state=IVectorState.OK)

            logger.info(f"Configuring video: {stream_w}x{stream_h}, exposure={stream_exp}s")

            # Configure for video
            config = self.picam2.create_video_configuration(
                main={"size": (stream_w, stream_h), "format": "BGR888"},
                queue=True,
                buffer_count=4,
            )
            self.picam2.align_configuration(config)
            self.picam2.configure(config)
            logger.info(f"Video configuration applied: {config['main']}")

            # Set controls
            exp_us = max(int(stream_exp * 1e6), 1)
            cam_controls = {"ExposureTime": exp_us}
            with self.parent.knownVectorsLock:
                cam_controls["AnalogueGain"] = self.parent.knownVectors["CCD_GAIN"]["GAIN"].value
            self.picam2.set_controls(cam_controls)
            self.picam2.start()
            logger.info("Video camera started")

            # Discard initial warm-up frames (auto-exposure needs a few frames to settle)
            warmup_frames = 5
            logger.info(f"Discarding {warmup_frames} warm-up frames")
            for i in range(warmup_frames):
                try:
                    self.picam2.capture_array("main")
                    logger.debug(f"Warm-up frame {i+1}/{warmup_frames} captured")
                except Exception as e:
                    logger.warning(f"Warm-up frame capture failed: {e}")
                    break

        except Exception as e:
            logger.error(f"Failed to configure video streaming: {e}")
            # Try to restart camera in a usable state
            try:
                if self.picam2 is not None and not self.picam2.started:
                    self.present_CameraSettings = CameraSettings()
            except Exception:
                pass
            return

        self.is_streaming = True
        self.Sig_StopStreaming.clear()
        self.StreamingThread = threading.Thread(target=self.__StreamingLoop, daemon=True)
        self.StreamingThread.start()

    def stopStreaming(self):
        """Stop live video streaming and return camera to still mode."""
        if not self.is_streaming:
            return
        logger.info("Stopping video stream")
        self.Sig_StopStreaming.set()
        if self.StreamingThread is not None and self.StreamingThread.is_alive():
            self.StreamingThread.join(timeout=5.0)
        self.is_streaming = False
        # Stop recording if still running
        self.stopRecording()
        # Stop camera; the exposure loop will reconfigure for stills on next exposure
        if self.picam2 is not None and self.picam2.started:
            self.picam2.stop_()
        # Force still-mode reconfiguration on next exposure
        self.present_CameraSettings = CameraSettings()
        logger.info("Video stream stopped")

    def __StreamingLoop(self):
        """Continuous streaming loop running in its own thread.

        Captures frames from picamera2, JPEG-encodes them and sends
        as INDI BLOBs via the CCD_VIDEO_STREAM BlobVector. Also updates
        FPS and handles recording.
        """
        frame_count = 0
        fps_frame_count = 0
        fps_start_time = time.time()
        avg_fps_frames = 0
        avg_fps_start = time.time()
        jpeg_quality = 80
        jpeg_buf = io.BytesIO()  # reusable buffer for JPEG encoding
        last_exp_us = None  # track last exposure to avoid redundant set_controls

        logger.info("Streaming loop started")

        while not self.Sig_StopStreaming.is_set():
            try:
                # Read streaming settings (may change live)
                with self.parent.knownVectorsLock:
                    stream_exp = self.parent.knownVectors["STREAMING_EXPOSURE"]["STREAMING_EXPOSURE_VALUE"].value
                    divisor = max(1, int(self.parent.knownVectors["STREAMING_EXPOSURE"]["STREAMING_DIVISOR_VALUE"].value))

                # Update exposure only when changed
                exp_us = max(int(stream_exp * 1e6), 1)
                if exp_us != last_exp_us:
                    self.picam2.set_controls({"ExposureTime": exp_us})
                    last_exp_us = exp_us

                # Capture frame
                array = self.picam2.capture_array("main")

                frame_count += 1

                # Debug: log frame stats for first few frames to detect black frames
                if frame_count <= 3:
                    logger.info(f"Stream frame {frame_count}: shape={array.shape}, min={array.min()}, max={array.max()}, mean={array.mean():.1f}")

                # Apply frame divisor — skip frames to reduce bandwidth
                if (frame_count % divisor) != 0:
                    continue

                # JPEG encode (reuse buffer to reduce allocations)
                jpeg_buf.seek(0)
                jpeg_buf.truncate()
                img = Image.fromarray(array[:, :, ::-1])  # BGR888 -> RGB for PIL
                img.save(jpeg_buf, format='JPEG', quality=jpeg_quality)
                jpeg_bytes = jpeg_buf.getvalue()

                # Recording: write raw frame data to SER file
                if self.is_recording and self.ser_recorder.is_open:
                    # SER expects raw pixel data, use BGR order to match config
                    raw_bytes = array.tobytes()
                    self.ser_recorder.add_frame(raw_bytes, timestamp_ns=time.time_ns())
                    # Check recording limits
                    with self.parent.knownVectorsLock:
                        rec_frames_done = self.ser_recorder.frame_count
                    if self.record_max_frames > 0 and rec_frames_done >= self.record_max_frames:
                        logger.info(f"Recording frame limit reached ({self.record_max_frames})")
                        self.stopRecording()
                        self.parent.setVector("RECORD_STREAM", "RECORD_OFF", value=ISwitchState.ON, state=IVectorState.OK)
                    elif self.record_max_duration > 0:
                        if not hasattr(self, '_record_start_time'):
                            self._record_start_time = time.time()
                        elapsed = time.time() - self._record_start_time
                        if elapsed >= self.record_max_duration:
                            logger.info(f"Recording duration limit reached ({self.record_max_duration}s)")
                            self.stopRecording()
                            self.parent.setVector("RECORD_STREAM", "RECORD_OFF", value=ISwitchState.ON, state=IVectorState.OK)

                # Send BLOB to client via CCD1 with format ".stream_jpg" —
                # this is the standard INDI StreamManager convention.
                # KStars routes BLOBs to the video viewer vs FITS viewer
                # based on the format string, NOT the blob vector name.
                try:
                    bv = self.parent.knownVectors["CCD1"]
                    bv["CCD1"].set_data(data=jpeg_bytes, format=".stream_jpg", compress=False)
                    bv.state = IVectorState.OK
                    bv.send_setVector()
                except Exception as e:
                    logger.error(f"Error sending streaming BLOB: {e}")

                # FPS calculation
                fps_frame_count += 1
                avg_fps_frames += 1
                now = time.time()
                elapsed = now - fps_start_time
                if elapsed >= 1.0:
                    est_fps = fps_frame_count / elapsed
                    avg_elapsed = now - avg_fps_start
                    avg_fps = avg_fps_frames / avg_elapsed if avg_elapsed > 0 else 0
                    self.parent.setVector("FPS", "EST_FPS", value=round(est_fps, 1), send=False)
                    self.parent.setVector("FPS", "AVG_FPS", value=round(avg_fps, 1), state=IVectorState.OK)
                    fps_frame_count = 0
                    fps_start_time = now

            except Exception as e:
                logger.error(f"Error in streaming loop: {e}")
                time.sleep(0.1)

        # Cleanup
        self.parent.setVector("FPS", "EST_FPS", value=0, send=False)
        self.parent.setVector("FPS", "AVG_FPS", value=0, state=IVectorState.IDLE)
        logger.info("Streaming loop exited")

    # ---- Recording ----

    def startRecording(self, mode="RECORD_ON"):
        """Start recording the video stream to a SER file.

        Args:
            mode: One of 'RECORD_ON' (unlimited), 'RECORD_DURATION' (time-limited),
                  'RECORD_FRAME_COUNT' (frame-limited).
        """
        if not self.is_streaming:
            logger.error("Cannot start recording: streaming not active")
            return
        if self.is_recording:
            logger.warning("Recording already active")
            return

        with self.parent.knownVectorsLock:
            rec_dir = self.parent.knownVectors["RECORD_FILE"]["RECORD_FILE_DIR"].value
            rec_name = self.parent.knownVectors["RECORD_FILE"]["RECORD_FILE_NAME"].value
            self.record_max_duration = self.parent.knownVectors["RECORD_OPTIONS"]["RECORD_DURATION"].value if mode == "RECORD_DURATION" else 0.0
            self.record_max_frames = int(self.parent.knownVectors["RECORD_OPTIONS"]["RECORD_FRAME_TOTAL"].value) if mode == "RECORD_FRAME_COUNT" else 0
            stream_w = int(self.parent.knownVectors["CCD_STREAM_FRAME"]["WIDTH"].value)
            stream_h = int(self.parent.knownVectors["CCD_STREAM_FRAME"]["HEIGHT"].value)

        # Build filename with timestamp
        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
        if not rec_name:
            rec_name = "record"
        filename = f"{rec_name}_{timestamp_str}.ser"
        filepath = os.path.join(rec_dir, filename)

        self.ser_recorder.open(
            filepath=filepath,
            width=stream_w,
            height=stream_h,
            color_id=SER_BGR,  # We capture BGR888
            bit_depth=8,
            observer="",
            instrument=self.parent.device,
        )
        self._record_start_time = time.time()
        self.is_recording = True
        logger.info(f"Recording started: {filepath}")

    def stopRecording(self):
        """Stop recording."""
        if not self.is_recording:
            return
        self.is_recording = False
        if self.ser_recorder.is_open:
            self.ser_recorder.close()
        logger.info("Recording stopped")
