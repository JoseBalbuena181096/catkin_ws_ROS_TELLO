## *********************************************************
##
## File autogenerated for the bebop_driver package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [], 'groups': [{'name': 'pilotingsettings', 'type': '', 'state': True, 'cstate': 'true', 'id': 1, 'parent': 0, 'parameters': [{'name': 'PilotingSettingsMaxAltitudeCurrent', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Current altitude max in m', 'min': 0.0, 'max': 160.0, 'srcline': 16, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'PilotingSettingsMaxTiltCurrent', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Current tilt max in degree', 'min': -180.0, 'max': 180.0, 'srcline': 18, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'PilotingSettingsAbsolutControlOn', 'type': 'int', 'default': 0, 'level': 0, 'description': '1 to enable, 0 to disable', 'min': 0, 'max': 1, 'srcline': 24, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PilotingSettingsAbsolutControlOn_OFF', 'type': 'int', 'value': 0, 'srcline': 21, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Disabled', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsAbsolutControlOn_ON', 'type': 'int', 'value': 1, 'srcline': 22, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Enabled', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': '1 to enable, 0 to disable'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsMaxDistanceValue', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Current max distance in meter', 'min': 0.0, 'max': 2000.0, 'srcline': 26, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover', 'type': 'int', 'default': 0, 'level': 0, 'description': '1 if the drone cant fly further than max distance, 0 if no limitation on the drone should be done', 'min': 0, 'max': 1, 'srcline': 32, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover_OFF', 'type': 'int', 'value': 0, 'srcline': 29, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Disabled', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover_ON', 'type': 'int', 'value': 1, 'srcline': 30, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Enabled', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': '1 if the drone cant fly further than max distance, 0 if no limitation on the drone should be done'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsBankedTurnValue', 'type': 'int', 'default': 0, 'level': 0, 'description': '1 to enable, 0 to disable', 'min': 0, 'max': 1, 'srcline': 38, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PilotingSettingsBankedTurnValue_OFF', 'type': 'int', 'value': 0, 'srcline': 35, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Disabled', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsBankedTurnValue_ON', 'type': 'int', 'value': 1, 'srcline': 36, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Enabled', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': '1 to enable, 0 to disable'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsMinAltitudeCurrent', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Current altitude min in m', 'min': 0.0, 'max': 160.0, 'srcline': 40, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'PilotingSettingsCirclingDirectionValue', 'type': 'int', 'default': 0, 'level': 0, 'description': 'The circling direction', 'min': 0, 'max': 1, 'srcline': 46, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PilotingSettingsCirclingDirectionValue_CW', 'type': 'int', 'value': 0, 'srcline': 43, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Circling ClockWise', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsCirclingDirectionValue_CCW', 'type': 'int', 'value': 1, 'srcline': 44, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Circling Counter ClockWise', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The circling direction'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsCirclingRadiusValue', 'type': 'int', 'default': 0, 'level': 0, 'description': 'The circling radius in meter', 'min': 0, 'max': 160, 'srcline': 48, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsCirclingAltitudeValue', 'type': 'int', 'default': 0, 'level': 0, 'description': 'The circling altitude in meter', 'min': 0, 'max': 160, 'srcline': 50, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsPitchModeValue', 'type': 'int', 'default': 0, 'level': 0, 'description': 'The Pitch mode', 'min': 0, 'max': 1, 'srcline': 56, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PilotingSettingsPitchModeValue_NORMAL', 'type': 'int', 'value': 0, 'srcline': 53, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Positive pitch values will make the drone lower its nose. Negative pitch values will make the drone raise its nose.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PilotingSettingsPitchModeValue_INVERTED', 'type': 'int', 'value': 1, 'srcline': 54, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Pitch commands are inverted. Positive pitch values will make the drone raise its nose. Negative pitch values will make the drone lower its nose.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The Pitch mode'}", 'ctype': 'int', 'cconsttype': 'const int'}], 'groups': [], 'srcline': 124, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT::PILOTINGSETTINGS', 'parentclass': 'DEFAULT', 'parentname': 'Default', 'field': 'DEFAULT::pilotingsettings', 'upper': 'PILOTINGSETTINGS', 'lower': 'pilotingsettings'}, {'name': 'speedsettings', 'type': '', 'state': True, 'cstate': 'true', 'id': 2, 'parent': 0, 'parameters': [{'name': 'SpeedSettingsMaxVerticalSpeedCurrent', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Current max vertical speed in m/s', 'min': 0.0, 'max': 10.0, 'srcline': 61, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'SpeedSettingsMaxRotationSpeedCurrent', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Current max yaw rotation speed in degree/s', 'min': 0.0, 'max': 900.0, 'srcline': 63, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'SpeedSettingsHullProtectionPresent', 'type': 'int', 'default': 0, 'level': 0, 'description': '1 if present, 0 if not present', 'min': 0, 'max': 1, 'srcline': 69, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'SpeedSettingsHullProtectionPresent_OFF', 'type': 'int', 'value': 0, 'srcline': 66, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Disabled', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'SpeedSettingsHullProtectionPresent_ON', 'type': 'int', 'value': 1, 'srcline': 67, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Enabled', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': '1 if present, 0 if not present'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'SpeedSettingsOutdoorOutdoor', 'type': 'int', 'default': 0, 'level': 0, 'description': '1 if outdoor flight, 0 if indoor flight', 'min': 0, 'max': 1, 'srcline': 75, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'SpeedSettingsOutdoorOutdoor_OFF', 'type': 'int', 'value': 0, 'srcline': 72, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Disabled', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'SpeedSettingsOutdoorOutdoor_ON', 'type': 'int', 'value': 1, 'srcline': 73, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Enabled', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': '1 if outdoor flight, 0 if indoor flight'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'SpeedSettingsMaxPitchRollRotationSpeedCurrent', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'Current max pitch/roll rotation speed in degree/s', 'min': 0.0, 'max': 900.0, 'srcline': 77, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}], 'groups': [], 'srcline': 124, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT::SPEEDSETTINGS', 'parentclass': 'DEFAULT', 'parentname': 'Default', 'field': 'DEFAULT::speedsettings', 'upper': 'SPEEDSETTINGS', 'lower': 'speedsettings'}, {'name': 'networksettings', 'type': '', 'state': True, 'cstate': 'true', 'id': 3, 'parent': 0, 'parameters': [{'name': 'NetworkSettingsWifiSelectionType', 'type': 'int', 'default': 0, 'level': 0, 'description': 'The type of wifi selection (auto, manual)', 'min': 0, 'max': 1, 'srcline': 86, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'NetworkSettingsWifiSelectionType_auto', 'type': 'int', 'value': 0, 'srcline': 83, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Auto selection', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'NetworkSettingsWifiSelectionType_manual', 'type': 'int', 'value': 1, 'srcline': 84, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Manual selection', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The type of wifi selection (auto, manual)'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'NetworkSettingsWifiSelectionBand', 'type': 'int', 'default': 0, 'level': 0, 'description': 'The allowed band(s) : 2.4 Ghz, 5 Ghz, or all', 'min': 0, 'max': 2, 'srcline': 92, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'NetworkSettingsWifiSelectionBand_2_4ghz', 'type': 'int', 'value': 0, 'srcline': 88, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': '2.4 GHz band', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'NetworkSettingsWifiSelectionBand_5ghz', 'type': 'int', 'value': 1, 'srcline': 89, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': '5 GHz band', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'NetworkSettingsWifiSelectionBand_all', 'type': 'int', 'value': 2, 'srcline': 90, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Both 2.4 and 5 GHz bands', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The allowed band(s) : 2.4 Ghz, 5 Ghz, or all'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'NetworkSettingsWifiSelectionChannel', 'type': 'int', 'default': 0, 'level': 0, 'description': 'The channel (not used in auto mode)', 'min': 0, 'max': 50, 'srcline': 93, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'int', 'cconsttype': 'const int'}], 'groups': [], 'srcline': 124, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT::NETWORKSETTINGS', 'parentclass': 'DEFAULT', 'parentname': 'Default', 'field': 'DEFAULT::networksettings', 'upper': 'NETWORKSETTINGS', 'lower': 'networksettings'}, {'name': 'picturesettings', 'type': '', 'state': True, 'cstate': 'true', 'id': 4, 'parent': 0, 'parameters': [{'name': 'PictureSettingsVideoStabilizationModeMode', 'type': 'int', 'default': 0, 'level': 0, 'description': 'Video stabilization mode', 'min': 0, 'max': 3, 'srcline': 104, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PictureSettingsVideoStabilizationModeMode_roll_pitch', 'type': 'int', 'value': 0, 'srcline': 99, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Video flat on roll and pitch', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoStabilizationModeMode_pitch', 'type': 'int', 'value': 1, 'srcline': 100, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Video flat on pitch only', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoStabilizationModeMode_roll', 'type': 'int', 'value': 2, 'srcline': 101, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Video flat on roll only', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoStabilizationModeMode_none', 'type': 'int', 'value': 3, 'srcline': 102, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Video follows drone angles', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Video stabilization mode'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoRecordingModeMode', 'type': 'int', 'default': 0, 'level': 0, 'description': 'Video recording mode', 'min': 0, 'max': 1, 'srcline': 110, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PictureSettingsVideoRecordingModeMode_quality', 'type': 'int', 'value': 0, 'srcline': 107, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Maximize recording quality.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoRecordingModeMode_time', 'type': 'int', 'value': 1, 'srcline': 108, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'Maximize recording time.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Video recording mode'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoFramerateFramerate', 'type': 'int', 'default': 0, 'level': 0, 'description': 'Video framerate', 'min': 0, 'max': 2, 'srcline': 117, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PictureSettingsVideoFramerateFramerate_24_FPS', 'type': 'int', 'value': 0, 'srcline': 113, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': '23.976 frames per second.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoFramerateFramerate_25_FPS', 'type': 'int', 'value': 1, 'srcline': 114, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': '25 frames per second.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoFramerateFramerate_30_FPS', 'type': 'int', 'value': 2, 'srcline': 115, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': '29.97 frames per second.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Video framerate'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoResolutionsType', 'type': 'int', 'default': 0, 'level': 0, 'description': 'Video streaming and recording resolutions', 'min': 0, 'max': 1, 'srcline': 123, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'PictureSettingsVideoResolutionsType_rec1080_stream480', 'type': 'int', 'value': 0, 'srcline': 120, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': '1080p recording, 480p streaming.', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'PictureSettingsVideoResolutionsType_rec720_stream720', 'type': 'int', 'value': 1, 'srcline': 121, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': '720p recording, 720p streaming.', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Video streaming and recording resolutions'}", 'ctype': 'int', 'cconsttype': 'const int'}], 'groups': [], 'srcline': 124, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT::PICTURESETTINGS', 'parentclass': 'DEFAULT', 'parentname': 'Default', 'field': 'DEFAULT::picturesettings', 'upper': 'PICTURESETTINGS', 'lower': 'picturesettings'}, {'name': 'gpssettings', 'type': '', 'state': True, 'cstate': 'true', 'id': 5, 'parent': 0, 'parameters': [{'name': 'GPSSettingsHomeTypeType', 'type': 'int', 'default': 0, 'level': 0, 'description': 'The type of the home position', 'min': 0, 'max': 2, 'srcline': 133, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': "{'enum': [{'name': 'GPSSettingsHomeTypeType_TAKEOFF', 'type': 'int', 'value': 0, 'srcline': 129, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'The drone will try to return to the take off position', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'GPSSettingsHomeTypeType_PILOT', 'type': 'int', 'value': 1, 'srcline': 130, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'The drone will try to return to the pilot position', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'GPSSettingsHomeTypeType_FOLLOWEE', 'type': 'int', 'value': 2, 'srcline': 131, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'description': 'The drone will try to return to the target of the current (or last) follow me', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'The type of the home position'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'GPSSettingsReturnHomeDelayDelay', 'type': 'int', 'default': 0, 'level': 0, 'description': 'Delay in second', 'min': 0, 'max': 120, 'srcline': 135, 'srcfile': '/home/jose/catkin_ws/src/bebop_autonomy/bebop_driver/cfg/autogenerated/BebopArdrone3.cfg', 'edit_method': '', 'ctype': 'int', 'cconsttype': 'const int'}], 'groups': [], 'srcline': 124, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT::GPSSETTINGS', 'parentclass': 'DEFAULT', 'parentname': 'Default', 'field': 'DEFAULT::gpssettings', 'upper': 'GPSSETTINGS', 'lower': 'gpssettings'}], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

BebopArdrone3_PilotingSettingsAbsolutControlOn_OFF = 0
BebopArdrone3_PilotingSettingsAbsolutControlOn_ON = 1
BebopArdrone3_PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover_OFF = 0
BebopArdrone3_PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover_ON = 1
BebopArdrone3_PilotingSettingsBankedTurnValue_OFF = 0
BebopArdrone3_PilotingSettingsBankedTurnValue_ON = 1
BebopArdrone3_PilotingSettingsCirclingDirectionValue_CW = 0
BebopArdrone3_PilotingSettingsCirclingDirectionValue_CCW = 1
BebopArdrone3_PilotingSettingsPitchModeValue_NORMAL = 0
BebopArdrone3_PilotingSettingsPitchModeValue_INVERTED = 1
BebopArdrone3_SpeedSettingsHullProtectionPresent_OFF = 0
BebopArdrone3_SpeedSettingsHullProtectionPresent_ON = 1
BebopArdrone3_SpeedSettingsOutdoorOutdoor_OFF = 0
BebopArdrone3_SpeedSettingsOutdoorOutdoor_ON = 1
BebopArdrone3_NetworkSettingsWifiSelectionType_auto = 0
BebopArdrone3_NetworkSettingsWifiSelectionType_manual = 1
BebopArdrone3_NetworkSettingsWifiSelectionBand_2_4ghz = 0
BebopArdrone3_NetworkSettingsWifiSelectionBand_5ghz = 1
BebopArdrone3_NetworkSettingsWifiSelectionBand_all = 2
BebopArdrone3_PictureSettingsVideoStabilizationModeMode_roll_pitch = 0
BebopArdrone3_PictureSettingsVideoStabilizationModeMode_pitch = 1
BebopArdrone3_PictureSettingsVideoStabilizationModeMode_roll = 2
BebopArdrone3_PictureSettingsVideoStabilizationModeMode_none = 3
BebopArdrone3_PictureSettingsVideoRecordingModeMode_quality = 0
BebopArdrone3_PictureSettingsVideoRecordingModeMode_time = 1
BebopArdrone3_PictureSettingsVideoFramerateFramerate_24_FPS = 0
BebopArdrone3_PictureSettingsVideoFramerateFramerate_25_FPS = 1
BebopArdrone3_PictureSettingsVideoFramerateFramerate_30_FPS = 2
BebopArdrone3_PictureSettingsVideoResolutionsType_rec1080_stream480 = 0
BebopArdrone3_PictureSettingsVideoResolutionsType_rec720_stream720 = 1
BebopArdrone3_GPSSettingsHomeTypeType_TAKEOFF = 0
BebopArdrone3_GPSSettingsHomeTypeType_PILOT = 1
BebopArdrone3_GPSSettingsHomeTypeType_FOLLOWEE = 2
