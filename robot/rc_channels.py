"""
rc_channels.py — RadioMaster TX16S channel layout constants.

Single source of truth for channel indices, display names, and switch
position labels.  Import from here rather than duplicating these values.

Channel layout (robot.ini defaults):
  CH1  Right X   right stick ← →   steering
  CH2  Right Y   right stick ↑ ↓
  CH3  Left Y    left  stick ↑ ↓   forward / reverse
  CH4  Left X    left  stick ← →
  CH5  SF        2-pos switch       MANUAL / AUTO
  CH6  SE        3-pos switch       speed limit: slow / mid / max
  CH7  SA        3-pos switch       AUTO type: Camera / GPS / Cam+GPS
  CH8  SB        2-pos switch       GPS logging: off / on
  CH9  SC        2-pos switch       data logging: off / on
  CH10 SD        3-pos switch       AUTO motor pause: low=running, mid/high=paused
  CH11 SG        3-pos switch       camera recording: low=off, mid=front cam, high=all
  CH12 SH        momentary button   GPS log bookmark / ESTOP reset
"""

# ── Channel indices (0-based) ─────────────────────────────────────────────────

CH_STEER     = 0    # CH1   Right X  — steering
CH_RIGHT_Y   = 1    # CH2   Right Y
CH_THROTTLE  = 2    # CH3   Left Y   — forward / reverse
CH_LEFT_X    = 3    # CH4   Left X
CH_MODE      = 4    # CH5   SF  2-pos: MANUAL / AUTO
CH_SPEED     = 5    # CH6   SE  3-pos: speed limit
CH_AUTO_TYPE = 6    # CH7   SA  3-pos: Camera / GPS / Cam+GPS
CH_GPS_LOG   = 7    # CH8   SB  2-pos: GPS logging on / off
CH_DLOG      = 8    # CH9   SC  2-pos: data logging off / on
CH_PAUSE     = 9    # CH10  SD  3-pos: AUTO motor pause (low=running, mid/high=paused)
CH_REC       = 10   # CH11  SG  3-pos: camera recording (low=off, mid=front, high=all)
CH_BOOKMARK  = 11   # CH12  SH  momentary: GPS bookmark or ESTOP reset
					# CH13  unused (index 12)
					# CH14  unused (index 13)

NUM_CHANNELS = 14

# ── Display names for all 14 channels ────────────────────────────────────────

CHANNEL_NAMES = [
    'CH1  Right X',   # 0
    'CH2  Right Y',   # 1
    'CH3  Left Y ',   # 2
    'CH4  Left X ',   # 3
    'CH5  SF     ',   # 4
    'CH6  SE     ',   # 5
    'CH7  SA     ',   # 6
    'CH8  SB     ',   # 7
    'CH9  SC     ',   # 8
    'CH10 SD     ',   # 9
    'CH11 SG     ',   # 10
    'CH12 SH     ',   # 11
    'CH13 S1     ',   # 12
    'CH14 S2     ',   # 13
]

# ── Switch position labels ────────────────────────────────────────────────────

MODE_NAMES      = {1000: 'MANUAL',   2000: 'AUTO'}
SPEED_NAMES     = {1000: 'slow 25%', 1500: 'mid',    2000: 'max'}
AUTO_TYPE_NAMES = {1000: 'Camera',   1500: 'GPS',     2000: 'Cam+GPS'}
GPS_LOG_NAMES   = {1000: 'off',      2000: 'on'}
DLOG_NAMES      = {1000: 'off',      2000: 'on'}
PAUSE_NAMES     = {1000: 'running',  1500: 'paused',  2000: 'paused'}  # mid/high both pause
REC_NAMES       = {1000: 'off',      1500: 'front',   2000: 'all'}
