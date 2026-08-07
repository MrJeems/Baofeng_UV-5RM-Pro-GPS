import struct
import logging
import time
import random
from datetime import datetime
from chirp import chirp_common, directory, memmap
from chirp import bitwise, errors
from chirp.settings import (
    RadioSetting, RadioSettingGroup,
    RadioSettingValueInteger, RadioSettingValueList,
    RadioSettingValueBoolean, RadioSettingValueString,
    RadioSettings, RadioSettingValueMap
)

LOG = logging.getLogger(__name__)

# Standard Baofeng Memory-Mapped Protocol Key
XOR_KEY = 0x1D

def _xor(data):
    """XORs a byte string with the standard radio key."""
    return bytes([x ^ XOR_KEY for x in data])

def _clean_read(serial_obj, size):
    """Helper to ensure we read the exact number of bytes requested."""
    data = serial_obj.read(size)
    if len(data) != size:
        raise errors.RadioError(f"Timeout waiting for data. Expected {size} bytes, got {len(data)}")
    return data

# --- UV-5RM Pro GPS Value Lists ---
VOX_LIST = ["OFF"] + [f"Level {i}" for i in range(1, 10)]
VOICE_LIST = ["OFF", "Chinese", "English"]
ROGER_LIST = ["OFF", "ROGER 1", "ROGER 2", "ROGER 3"]
DISPLAY_LIST = ["PICTURE", "MESSAGE", "VOLTAGE"]
LANG_LIST = ["English", "Chinese"]
TAIL_LIST = ["OFF", "55", "120", "180", "240"]
DIR_LIST = ["STAN", "FAIL"]
APO_LIST = ["OFF", "30min", "60min", "120min", "240min", "480min"]
ALERT_LIST = ["1000Hz", "1450Hz", "1750Hz", "2100Hz"]
TOT_LIST = ["OFF"] + [f"{i}s" for i in range(15, 225, 15)]
TOA_LIST = ["OFF"] + [f"{i}s" for i in range(1, 11)]
PTT_DLY_LIST = ["10ms"] + [f"{i*10}ms" for i in range(1, 250)]
WX_LIST = [f"WX{i}" for i in range(1, 11)]
SCAN_MODE_LIST = ["TO", "CO", "SE"]
SCAN_MEM_LIST = ["ENCODER", "DECODER", "ALL"]
PROG_KEY_MAP = [
    ("None", 0x00), ("SCAN", 0x01), ("MONITOR", 0x02), ("FM Radio", 0x04),
    ("SOS", 0x05), ("GNSS System", 0x06), ("One Touch Search", 0x07),
    ("1750Hz", 0x09), ("Falling Alarm", 0x0A), ("One Touch Call", 0x0B),
    ("Zone", 0x0C), ("Battery Display", 0x0D), ("Power", 0x0E), ("Vox", 0x0F)
]

# --- Verified Hex Memory Map ---
MEM_FORMAT = """
# seekto 0x0000;
u8 radio_name_0000[16];

# seekto 0x0050;
struct{
    char        soft_ver[7];
    u8          pad_0x57;
    char        hard_ver[6];
    u8          pad_0x5e_f[2];
    char        last_prog[16];
} dev_info;

# seekto 0x0080;
struct{
    lbcd        rx_freq[4];
    lbcd        tx_freq[4];
    bbcd        rx_tone[2];
    bbcd        tx_tone[2];
    u8          unk_0x0d[4];
    u8          power:2,
                bandwidth:1,
                unk1_0x10:3,
                invert:1,
                talkaround:1;
    u8          pttid_mode:4,
                sqlch_mode:4;
    u8          signaling:4,
                jump_mode:4;
    u8          unk_0x13:2,
                launch_banned:1,
                unk1_0x13:5;
    u8          unk_0x14_18[5];
    u8          dtmf;
    u8          twotone;
    u8          unk_0x1b[1];
    u8          mdc;
    u8          unk_0x1d[1];
    u8          emerg_sys;
    u8          unk_0x1f[1];
    u8          name[16]; 
} memory[640];

# seekto 0x798e;
struct {
    u8 vox_lvl;
    u8 vox_dly_detect;
    u8 unk_0x90_93[4];
    u8 apo;
    u8 tot;
    u8 toa;
    u8 unk_0x97;
    u8 gps_zone;
    u8 unk_0x99;
    u8 alert_freq;
    u8 unk_0x9b_9d[3];
    u8 wx_channel;      
    u8 password_enabled;
    u8 byte_79a0;
    u8 byte_79a1;
    u8 byte_79a2;
    u8 byte_79a3;
    u8 unknown_79a4;
    u8 byte_79a5;
    u8 byte_79a6;
} glob_settings;

# seekto 0x79b0;
struct{
    u8      pf1_short;
    u8      pf2_short;
    u8      pf1_long;
    u8      pf2_long;
}prog_key;

# seekto 0x79c0;
u8 power_on_pwd[8];

# seekto 0x7a20;
struct{
    lbit      bitfield[640];
} chan_empty;

# seekto 0x7b18;
struct {
    char    name[16];
    u16     channel_count;
    u16     channels[64];
    u8      pad[6];
} zones[10];

# seekto 0x8180;
struct{
    u8      scan_mode;
    u8      flyback;
    u8      rx_recovery;
    u8      tx_recovery;
    u8      channel_rtn;
    u8      priority;
    u8      unk_0x8186[1];
    u8      prio_scan_chan;
    u8      scan_range;
    u8      unk_0x8189[2];
    u8      scan_memory;
} scan_menu;

# seekto 0x81a0;
struct{
    lbit      bitfield[640];
} scan_skip;

# seekto 0x8200;
struct{
    u8      dtmf_ani;
    u8      sending_rate;
    u8      first_tm_code;
    u8      precarrier_tm;
    u8      delay_tm;
    u8      ptt_pause_tm;
    u8      dtmf_st;
    u8      auto_reset_tm;
    u8      sepr_opts;
    u8      group_num;
    u8      decode_resp;
    u8      unk_0xb_f[5];
    u8      self_id[3];
    u8      unk_0x13_17[5];
    u8      ptt_id[16];
    u8      ptt_id_offline[16];
    u8      stun[11];
    u8      pad_0x43_47[5];
    u8      kill[11];
}dtmf;

# seekto 0x8260;
struct{
    u8      entry[16];
} dtmf_list[16];

# seekto 0x8404;
u8 st_2tone;

# seekto 0x9443;
u8 fivetone_id[5];

# seekto 0x9450;
u8 st_5tone;

# seekto 0x9c80;
u16 biis_id;

# seekto 0x9c88;
u8 st_bdc1200;
"""

ADDRS = [0x000000, 0x100000, 0x200000, 0x300000,
         0x400000, 0x500000, 0x600000, 0x700000,
         0x800000, 0x900000, 0xa00000]
BLK_SZ = 0x1000
T_INFO = bytes(12) + bytes([0xFF, 0xFF, 0xFF, 0xFF])
DEFAULT_PWD = bytes([0xFF] * 8)


def raw_send(serial, data, exlen):
    serial.write(data)
    return serial.read(exlen)


def wakeup(serial, xor):
    header_sync = b'PROGRAM' + bytes([xor])
    serial.write(T_INFO)
    time.sleep(0.5)
    serial.timeout = 2.0
    resp = serial.read(16)

    if len(resp) >= 1 and resp[0] == 0x41:
        serial.write(header_sync)
        return

    try:
        serial.close()
        serial.baudrate = 115200
        serial.open()
        serial.dtr = True
        serial.rts = True
        time.sleep(0.1)
        serial.reset_input_buffer()
        serial.write(header_sync)
        return
    except Exception as e:
        err = f"Radio Wakeup baud switch failed: {e}"
        LOG.error(err)
        raise errors.RadioError(err)


def exit_prog(serial, xor):
    ba = [b ^ xor for b in b'END\x00']
    try:
        raw_send(serial, bytes(ba), 1)
    except Exception as e:
        LOG.warning(f"Exit prog error: {e}")


def do_init(serial, xor):
    resp = raw_send(serial, b'', 1)
    if len(resp) < 1 or (resp[0] ^ xor) != 0x41:
        raise errors.RadioError(f"Init HANDSHAKE3 expected 0x41 got {resp!r}")

    pwd_bytes = bytes([b ^ xor for b in DEFAULT_PWD])
    resp = raw_send(serial, pwd_bytes, 1)
    if len(resp) < 1 or (resp[0] ^ xor) != 0x41:
        raise errors.RadioError(f"Init HANDSHAKE4 (password) expected 0x41 got {resp!r}")

    xs = bytes([xor ^ ord(c) for c in 'INFORMATION'])
    resp = raw_send(serial, xs, 16)
    if len(resp) < 16:
        raise errors.RadioError(f"Init HANDSHAKE5 short response: got {len(resp)} bytes")

    mode_byte = bytes([xor ^ 0x52])  
    resp = raw_send(serial, mode_byte, 1)
    if len(resp) < 1 or (resp[0] ^ xor) != 0x41:
        raise errors.RadioError(f"Init HANDSHAKE6 expected 0x41 got {resp!r}")


def read_block(serial, xor, addr, blk_len):
    i_cmd = (0x52000000 + addr) ^ (0x01010101 * xor)
    cmd = struct.pack('>L', i_cmd)
    resp = raw_send(serial, cmd, blk_len + 4)
    i_resp = struct.unpack('>L', resp[:4])[0]
    if i_resp != ((0x57000000 + addr) ^ (0x01010101 * xor)):
        raise errors.RadioError("Read block echo failed")
    return resp[4:]


def write_block(serial, xor, addr, data, blk_len):
    i_cmd = (0x57000000 + addr) ^ (0x01010101 * xor)
    cmd = struct.pack('>L', i_cmd)
    resp = raw_send(serial, cmd + data, 1)
    if len(resp) < 1 or (resp[0] ^ xor) != 0x41:
        raise errors.RadioError(f"Write block ack failed expect 0x41 got {resp[0] if resp else 'empty'}")
    return blk_len


def do_download(radio):
    data = b''
    try:
        xor = random.randint(1, 255)
        serial = radio.pipe
        serial.timeout = 5.0
        status = chirp_common.Status()
        status.msg = "Connecting to Radio..."
        radio.status_fn(status)

        wakeup(serial, xor)
        do_init(serial, xor)

        status.max = len(ADDRS) * BLK_SZ
        status.msg = "Downloading..."
        for addr in ADDRS:
            resp = read_block(serial, xor, addr, BLK_SZ)
            da = bytes([b ^ xor for b in resp])
            data += da
            status.cur += len(resp)
            radio.status_fn(status)
    except errors.RadioError:
        raise
    except Exception as e:
        raise errors.RadioError(f"Error during download: {e}")
    finally:
        exit_prog(serial, xor)

    return memmap.MemoryMapBytes(data)


def do_upload(radio):
    try:
        fmt = "%Y.%m.%d %H:%M"
        prog_dt = datetime.now().strftime(fmt)
        radio._memobj.dev_info.last_prog = prog_dt

        xor = random.randint(1, 255)
        serial = radio.pipe
        serial.timeout = 5.0
        status = chirp_common.Status()
        status.msg = "Connecting to Radio..."
        radio.status_fn(status)

        wakeup(serial, xor)
        do_init(serial, xor)

        status.max = len(ADDRS) * BLK_SZ
        status.msg = "Uploading..."
        for i, addr in enumerate(ADDRS):
            si = BLK_SZ * i
            ei = BLK_SZ * (i + 1)
            data = radio._mmap[si:ei]
            enc_data = bytes([b ^ xor for b in data])
            resp = write_block(serial, xor, addr, enc_data, BLK_SZ)
            status.cur += resp
            radio.status_fn(status)
    except errors.RadioError:
        raise
    except Exception as e:
        raise errors.RadioError(f"Error during upload: {e}")
    finally:
        exit_prog(serial, xor)


@directory.register
class BaofengUV5RMProGPS(chirp_common.CloneModeRadio):
    """Baofeng UV-5RM Pro GPS"""
    VENDOR = "Baofeng"
    MODEL = "UV-5RM Pro GPS"
    BAUD_RATE = 19200
    
    _memsize = 0x10000
    
    POWER_LEVELS = [chirp_common.PowerLevel("Low", watts=2.00),
                    chirp_common.PowerLevel("Medium", watts=5.00),
                    chirp_common.PowerLevel("High", watts=10.00)]
                    
    VALID_MODES = ["NFM", "FM"]
    VALID_BANDS = [(136000000, 174000000), (400000000, 470000000)]
    VALID_TONES = (63.0, ) + chirp_common.TONES
    VALID_DCS = (17, 50, 645) + chirp_common.DTCS_CODES
    VALID_CHARSET = "".join(chr(i) for i in range(32, 127))
    
    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_bank = False
        rf.has_tuning_step = False
        rf.has_name = True
        rf.valid_characters = self.VALID_CHARSET
        rf.valid_name_length = 16
        rf.has_offset = True
        rf.has_mode = True
        rf.has_dtcs = True
        rf.has_rx_dtcs = True
        rf.has_dtcs_polarity = True
        rf.has_ctone = True
        rf.has_cross = True
        rf.valid_tones = self.VALID_TONES
        rf.can_odd_split = False
        rf.can_delete = True
        rf.valid_modes = self.VALID_MODES
        rf.valid_duplexes = ["", "-", "+", "off"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = [
            "Tone->DTCS", "DTCS->Tone", "->Tone",
            "Tone->Tone", "->DTCS", "DTCS->", "DTCS->DTCS"]
        rf.valid_power_levels = self.POWER_LEVELS
        rf.valid_skips = ["", "S"]
        rf.valid_bands = self.VALID_BANDS
        rf.memory_bounds = (1, 640)
        return rf

    @classmethod
    def get_prompts(cls):
        p = chirp_common.RadioPrompts()
        p.has_bank = False
        return p

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    def sync_in(self):
        try:
            data = do_download(self)
        except Exception as e:
            raise errors.RadioError(f"Error during download {e}")
        self._mmap = data
        self.process_mmap()

    def sync_out(self):
        try:
            do_upload(self)
        except Exception as e:
            raise errors.RadioError(f"Error during upload {e}")

    def get_memory(self, number):
        mem = chirp_common.Memory()
        mem.number = number
        _mem = self._memobj.memory[number-1]
        mem.freq = int(_mem.rx_freq) * 10
        if self.get_empty(number - 1):
            mem.empty = True
            return mem
            
        name_chars = []
        for c in _mem.name:
            val = int(c)
            if val == 0xFF or val == 0x00:
                break
            if 32 <= val < 127:
                name_chars.append(chr(val))
            else:
                name_chars.append(' ') 
        mem.name = "".join(name_chars).rstrip()
        
        mem.power = self.POWER_LEVELS[int(_mem.power) if int(_mem.power) < 3 else 2]
        mem.mode = self.VALID_MODES[_mem.bandwidth]
        mem.skip = 'S' if self.get_skip(number - 1) else ""

        if int(_mem.tx_freq) == int(_mem.rx_freq):
            mem.duplex = ''
            mem.offset = 0
        elif int(_mem.tx_freq) == 0:
            mem.duplex = 'off'
            mem.offset = 0
        elif int(_mem.tx_freq) > int(_mem.rx_freq):
            mem.duplex = '+'
            mem.offset = (int(_mem.tx_freq) - int(_mem.rx_freq)) * 10
        elif int(_mem.rx_freq) > int(_mem.tx_freq):
            mem.duplex = '-'
            mem.offset = ((int(_mem.rx_freq) - int(_mem.tx_freq)) * 10)

        self.get_tones(_mem, mem)

        mem.extra = RadioSettingGroup("Extra", "extra")
        
        mem.extra.append(RadioSetting("invert", "TX Invert", RadioSettingValueBoolean(bool(int(_mem.invert)))))
        mem.extra.append(RadioSetting("talkaround", "Talkaround", RadioSettingValueBoolean(bool(int(_mem.talkaround)))))

        rstype = RadioSettingValueMap([('Off', 0), ('PTT BOT', 1), ('PTT EOT', 2), ('PTT Both', 3), ('5-Tone BOT', 4), ('5-Tone EOT', 8), ('5-Tone Both', 12)], int(_mem.pttid_mode))
        mem.extra.append(RadioSetting("pttid_mode", "PTT/5-Tone ID", rstype))

        rstype = RadioSettingValueMap([('Off', 0), ('CTDCS', 1), ('Optional', 2), ('Both', 3)], int(_mem.sqlch_mode))
        mem.extra.append(RadioSetting("sqlch_mode", "SP-Mute", rstype))

        rstype = RadioSettingValueMap([('Off', 0), ('DTMF', 2), ('2-Tone', 4), ('5-Tone', 6), ('MDC', 8), ('BDC1200', 10), ('BIIS', 12)], int(_mem.signaling))
        mem.extra.append(RadioSetting("signaling", "Signaling", rstype))

        rstype = RadioSettingValueMap([('Off', 0), ('Normal', 1), ('Strict', 2)], int(_mem.jump_mode))
        mem.extra.append(RadioSetting("jump_mode", "Frequency Jump", rstype))

        mem.extra.append(RadioSetting("launch_banned", "Busy Lockout", RadioSettingValueBoolean(bool(int(_mem.launch_banned)))))

        return mem

    def set_memory(self, mem):
        _mem = self._memobj.memory[mem.number - 1]

        if mem.empty:
            self.set_empty(mem.number - 1, 1)
            _mem.fill_raw(b'\x00')
            return

        _mem.rx_freq = mem.freq // 10

        if self.get_empty(mem.number - 1):
            self.set_empty(mem.number - 1, 0)
            mem.duplex == ''
            _mem.tx_freq.set_value(int(_mem.rx_freq))
            _mem.power = 2

        try:
            _mem.power = self.POWER_LEVELS.index(mem.power)
        except ValueError:
            _mem.power = 2

        padded_name = mem.name.ljust(16)
        for i in range(16):
            char_val = ord(padded_name[i])
            if char_val > 127: char_val = 32 
            _mem.name[i] = char_val

        _skip = 1 if mem.skip == 'S' else 0
        self.set_skip(mem.number - 1, _skip)
        _mem.bandwidth = self.VALID_MODES.index(mem.mode)
        self.set_tone(_mem, mem)

        if mem.duplex == '+':
            _mem.tx_freq = (mem.freq + mem.offset) // 10
        elif mem.duplex == '-':
            _mem.tx_freq = (mem.freq - mem.offset) // 10
        elif mem.duplex == '':
            _mem.tx_freq.set_value(int(_mem.rx_freq))
        elif mem.duplex == 'off':
            _mem.tx_freq.set_value(0)

        for setting in mem.extra:
            val = setting.value
            if isinstance(val, bool):
                val = 1 if val else 0
            setattr(_mem, setting.get_name(), val)

    def get_tones(self, _mem, mem):
        _memrxtone = int(_mem.rx_tone)
        msb = _mem.rx_tone.get_raw()[0]
        if _memrxtone == 16665 or _memrxtone == 0:
            rxtone = ("", 0, None)
        elif 0x80 <= msb < 0xc0:
            rxtone = ("DTCS", _memrxtone % 8000, "N")
        elif msb >= 0xc0:
            rxtone = ("DTCS", _memrxtone % 12000, "R")
        else:
            rxtone = ("Tone", _memrxtone / 10, None)

        _memtxtone = int(_mem.tx_tone)
        msb = _mem.tx_tone.get_raw()[0]
        if _memtxtone == 16665 or _memtxtone == 0:
            txtone = ("", 0, None)
        elif 0x80 <= msb < 0xc0:
            txtone = ("DTCS", _memtxtone % 8000, "N")
        elif msb >= 0xc0:
            txtone = ("DTCS", _memtxtone % 12000, "R")
        else:
            txtone = ("Tone", _memtxtone / 10, None)

        chirp_common.split_tone_decode(mem, txtone, rxtone)

    def set_tone(self, _mem, mem):
        ((txmode, txval, txpol),
         (rxmode, rxval, rxpol)) = chirp_common.split_tone_encode(mem)
        if txmode == "":
            _mem.tx_tone.set_raw(b'\xff\xff')
        if rxmode == "":
            _mem.rx_tone.set_raw(b'\xff\xff')
        if txmode == "Tone":
            _mem.tx_tone = int(txval * 10)
        if rxmode == "Tone":
            _mem.rx_tone = int(rxval * 10)
        if txmode == "DTCS" and txpol == "N":
            _mem.tx_tone = int(txval + 8000)
        if rxmode == "DTCS" and rxpol == "N":
            _mem.rx_tone = int(rxval + 8000)
        if txmode == "DTCS" and txpol == "R":
            msb = (txval // 100) + 0xc0
            lsb = int("%02i" % (txval % 100), 16)
            _mem.tx_tone.set_raw(bytes([msb, lsb]))
        if rxmode == "DTCS" and rxpol == "R":
            msb = (txval // 100) + 0xc0
            lsb = int("%02i" % (txval % 100), 16)
            _mem.rx_tone.set_raw(bytes([msb, lsb]))

    def get_skip(self, chan_n):
        return self._memobj.scan_skip['bitfield'][chan_n].get_value()

    def set_skip(self, chan_n, value):
        self._memobj.scan_skip['bitfield'][chan_n].set_value(value)

    def get_empty(self, chan_n):
        return self._memobj.chan_empty['bitfield'][chan_n].get_value()

    def set_empty(self, chan_n, value):
        self._memobj.chan_empty['bitfield'][chan_n].set_value(value)

    def get_settings(self):
        top = RadioSettingGroup("top", "UV-5RM Pro GPS Settings")
        basic = RadioSettingGroup("basic", "Basic Settings")
        ui = RadioSettingGroup("ui", "UI & Display")
        gps = RadioSettingGroup("gps", "GNSS Settings")
        sidetone = RadioSettingGroup("sidetone", "Sidetone Settings")
        ident = RadioSettingGroup("ident", "Radio Identities")
        keys = RadioSettingGroup("keys", "Programmable Keys")
        zones = RadioSettingGroup("zones", "Zones")
        scan = RadioSettingGroup("scan", "Scan Settings")

        raw_name = bytes(self._memobj.radio_name_0000)
        end_idx = raw_name.find(b'\xff')
        if end_idx != -1:
            raw_name = raw_name[:end_idx]
        name_str = "".join([chr(b) for b in raw_name if 32 <= b < 127])
        rs_name = RadioSetting("radio_name", "Radio Name", RadioSettingValueString(0, 16, name_str))
        rs_name.value.set_charset(self.VALID_CHARSET)
        basic.append(rs_name)
        
        val_79a0 = int(self._memobj.glob_settings.byte_79a0)
        
        vox_on = bool((val_79a0 & 0x80) == 0x80)
        vox_level = int(self._memobj.glob_settings.vox_lvl) if vox_on else 0
        if vox_level > 9: vox_level = 0
        basic.append(RadioSetting("vox", "VOX Level", RadioSettingValueList(options=VOX_LIST, current_index=vox_level)))
        
        aprs_on = bool((val_79a0 & 0x40) == 0x40)
        gps.append(RadioSetting("aprs", "GNSS APRS", RadioSettingValueBoolean(aprs_on)))
        
        voice_val = (val_79a0 & 0x0C) >> 2
        if voice_val > 2: voice_val = 0
        basic.append(RadioSetting("voice", "Voice Prompt", RadioSettingValueList(options=VOICE_LIST, current_index=voice_val)))
        
        val_79a1 = int(self._memobj.glob_settings.byte_79a1)
        autolock_on = bool((val_79a1 & 0x40) == 0x40)
        basic.append(RadioSetting("autolock", "Auto Keypad Lock", RadioSettingValueBoolean(autolock_on)))

        val_79a2 = int(self._memobj.glob_settings.byte_79a2)
        beep_on = bool((val_79a2 & 0x80) == 0x80)
        basic.append(RadioSetting("beep", "Keypad Beep", RadioSettingValueBoolean(beep_on)))
        
        roger_val = (val_79a2 & 0x60) >> 5
        if roger_val > 3: roger_val = 0
        basic.append(RadioSetting("roger", "Roger Beep", RadioSettingValueList(options=ROGER_LIST, current_index=roger_val)))
        
        val_79a3 = int(self._memobj.glob_settings.byte_79a3)
        gnss_on = bool((val_79a3 & 0x80) == 0x80)
        gps.append(RadioSetting("gnss", "GNSS Enable", RadioSettingValueBoolean(gnss_on)))
                     
        radio_int = bool((val_79a3 & 0x04) == 0x04)
        basic.append(RadioSetting("radio_int", "Radio Interrupt", RadioSettingValueBoolean(radio_int)))

        val_79a5 = int(self._memobj.glob_settings.byte_79a5)
        display_val = val_79a5 & 0x03
        if display_val > 2: display_val = 0
        ui.append(RadioSetting("display", "Power On Display", RadioSettingValueList(options=DISPLAY_LIST, current_index=display_val)))
        
        lang_val = (val_79a5 & 0x04) >> 2
        if lang_val > 1: lang_val = 0
        ui.append(RadioSetting("language", "Language", RadioSettingValueList(options=LANG_LIST, current_index=lang_val)))
        
        noaa_alert = bool((val_79a5 & 0x08) == 0x08)
        basic.append(RadioSetting("noaa_alert", "NOAA WX Alert", RadioSettingValueBoolean(noaa_alert)))

        val_79a6 = int(self._memobj.glob_settings.byte_79a6)
        tail_val = (val_79a6 & 0xE0) >> 5
        if tail_val > 4: tail_val = 0
        basic.append(RadioSetting("tail", "Squelch Tail", RadioSettingValueList(options=TAIL_LIST, current_index=tail_val)))
        
        noaa_on = bool((val_79a6 & 0x10) == 0x10)
        basic.append(RadioSetting("noaa_on", "NOAA Weather Enable", RadioSettingValueBoolean(noaa_on)))

        wx_val = int(self._memobj.glob_settings.wx_channel)
        if wx_val > 9: wx_val = 0
        basic.append(RadioSetting("wx_channel", "NOAA Weather Channel", RadioSettingValueList(options=WX_LIST, current_index=wx_val)))
        
        dir_val = (val_79a6 & 0x08) >> 3
        if dir_val > 1: dir_val = 0
        ui.append(RadioSetting("dir", "Display Direction", RadioSettingValueList(options=DIR_LIST, current_index=dir_val)))

        tot_val = int(self._memobj.glob_settings.tot)
        if tot_val > 14: tot_val = 0
        basic.append(RadioSetting("tot", "Time-Out Timer (TOT)", RadioSettingValueList(options=TOT_LIST, current_index=tot_val)))

        toa_val = int(self._memobj.glob_settings.toa)
        if toa_val > 10: toa_val = 0
        basic.append(RadioSetting("toa", "Timer Pre-Alarm (TOA)", RadioSettingValueList(options=TOA_LIST, current_index=toa_val)))

        ptt_dly_val = int(self._memobj.dtmf.delay_tm)
        if ptt_dly_val > 249: ptt_dly_val = 0
        basic.append(RadioSetting("ptt_dly", "PTT-ID Delay", RadioSettingValueList(options=PTT_DLY_LIST, current_index=ptt_dly_val)))
                     
        apo_val = int(self._memobj.glob_settings.apo)
        if apo_val > 5: apo_val = 0
        basic.append(RadioSetting("apo", "Auto Power Off (APO)", RadioSettingValueList(options=APO_LIST, current_index=apo_val)))
                     
        alert_val = int(self._memobj.glob_settings.alert_freq)
        if alert_val > 3: alert_val = 0
        basic.append(RadioSetting("alert", "Alert Tone", RadioSettingValueList(options=ALERT_LIST, current_index=alert_val)))

        tz_raw = int(self._memobj.glob_settings.gps_zone)
        tz_val = tz_raw - 12
        if tz_val < -12 or tz_val > 12: tz_val = 0
        gps.append(RadioSetting("timezone", "GNSS Time Zone", RadioSettingValueInteger(-12, 12, tz_val)))

        pwd_raw = int(self._memobj.glob_settings.password_enabled)
        pwd_toggle = bool(pwd_raw == 1)
        basic.append(RadioSetting("password_enabled", "Password Enabled", RadioSettingValueBoolean(pwd_toggle)))

        raw_pwd = self._memobj.power_on_pwd
        pwd_str = "".join([chr(int(b)) for b in raw_pwd if 0x30 <= int(b) <= 0x39])
        rs_pwd = RadioSetting("password", "Power-on Password", RadioSettingValueString(0, 8, pwd_str))
        rs_pwd.value.set_charset("0123456789")
        basic.append(rs_pwd)

        sidetone.append(RadioSetting("st_dtmf", "DTMF Sidetone", RadioSettingValueBoolean(bool(int(self._memobj.dtmf.dtmf_st)))))
        sidetone.append(RadioSetting("st_2tone", "2Tone Sidetone", RadioSettingValueBoolean(bool(int(self._memobj.st_2tone)))))
        sidetone.append(RadioSetting("st_5tone", "5Tone Sidetone", RadioSettingValueBoolean(bool(int(self._memobj.st_5tone)))))
        sidetone.append(RadioSetting("st_bdc", "BDC1200 Sidetone", RadioSettingValueBoolean(bool(int(self._memobj.st_bdc1200)))))
        
        dtmf_str = "".join([str(int(b)) for b in self._memobj.dtmf.self_id if 0 <= int(b) <= 9])
        rs_dtmf = RadioSetting("dtmf_id", "DTMF ID", RadioSettingValueString(0, 3, dtmf_str))
        rs_dtmf.value.set_charset("0123456789")
        ident.append(rs_dtmf)

        fivetone_str = "".join([str(int(b)) for b in self._memobj.fivetone_id if 0 <= int(b) <= 9])
        rs_5t = RadioSetting("fivetone_id", "5Tone ID", RadioSettingValueString(0, 5, fivetone_str))
        rs_5t.value.set_charset("0123456789")
        ident.append(rs_5t)
        
        biis_val = int(self._memobj.biis_id)
        if biis_val > 65535: biis_val = 0
        ident.append(RadioSetting("biis_id", "BIIS ID", RadioSettingValueInteger(0, 65535, biis_val)))

        val_pf1_s = int(self._memobj.prog_key.pf1_short)
        if val_pf1_s not in [v for k, v in PROG_KEY_MAP]: val_pf1_s = 0
        keys.append(RadioSetting("pf1_short", "SK1 (PF1) Short Press", RadioSettingValueMap(PROG_KEY_MAP, val_pf1_s)))
        
        val_pf2_s = int(self._memobj.prog_key.pf2_short)
        if val_pf2_s not in [v for k, v in PROG_KEY_MAP]: val_pf2_s = 0
        keys.append(RadioSetting("pf2_short", "SK2 (PF2) Short Press", RadioSettingValueMap(PROG_KEY_MAP, val_pf2_s)))
        
        val_pf1_l = int(self._memobj.prog_key.pf1_long)
        if val_pf1_l not in [v for k, v in PROG_KEY_MAP]: val_pf1_l = 0
        keys.append(RadioSetting("pf1_long", "SK1 (PF1) Long Press", RadioSettingValueMap(PROG_KEY_MAP, val_pf1_l)))
        
        val_pf2_l = int(self._memobj.prog_key.pf2_long)
        if val_pf2_l not in [v for k, v in PROG_KEY_MAP]: val_pf2_l = 0
        keys.append(RadioSetting("pf2_long", "SK2 (PF2) Long Press", RadioSettingValueMap(PROG_KEY_MAP, val_pf2_l)))

        # --- Map Zones into Settings ---
        for i in range(10):
            _zone = self._memobj.zones[i]
            raw_zname = []
            for char in _zone.name:
                val = int(char)
                if val == 0x00 or val == 0xFF:
                    break
                raw_zname.append(val)
            zname_str = bytes(raw_zname).decode('ascii', 'ignore').strip()
            rs_zname = RadioSetting(f"zone_name_{i}", f"Zone {i+1} Name", RadioSettingValueString(0, 16, zname_str))
            rs_zname.value.set_charset(self.VALID_CHARSET)
            zones.append(rs_zname)

        scan_mode_idx = int(self._memobj.scan_menu.scan_mode)
        if scan_mode_idx >= len(SCAN_MODE_LIST): scan_mode_idx = 0
        scan.append(RadioSetting("scan_mode", "Scan Mode", RadioSettingValueList(options=SCAN_MODE_LIST, current_index=scan_mode_idx)))

        scan_mem_idx = int(self._memobj.scan_menu.scan_memory)
        if scan_mem_idx >= len(SCAN_MEM_LIST): scan_mem_idx = 0
        scan.append(RadioSetting("scan_memory", "Scan Memory", RadioSettingValueList(options=SCAN_MEM_LIST, current_index=scan_mem_idx)))

        top.append(basic)
        top.append(ui)
        top.append(gps)
        top.append(sidetone)
        top.append(ident)
        top.append(keys)
        top.append(scan)
        top.append(zones)
        
        return top

    def set_settings(self, settings):
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
                
            setting = element.get_name()
            value = element.value
            
            if setting == "radio_name":
                name_bytes = str(value).encode('ascii', 'replace')
                name_bytes = name_bytes.ljust(16, b'\xff')[:16]
                for i in range(16):
                    self._memobj.radio_name_0000[i] = name_bytes[i]

            elif setting == "vox":
                val = int(self._memobj.glob_settings.byte_79a0)
                if int(value) == 0:
                    self._memobj.glob_settings.byte_79a0 = val & 0x7F 
                    self._memobj.glob_settings.vox_lvl = 0x01 
                else:
                    self._memobj.glob_settings.byte_79a0 = val | 0x80 
                    self._memobj.glob_settings.vox_lvl = int(value)
            elif setting == "aprs":
                val = int(self._memobj.glob_settings.byte_79a0)
                if value:
                    self._memobj.glob_settings.byte_79a0 = val | 0x40
                else:
                    self._memobj.glob_settings.byte_79a0 = val & 0xBF
            elif setting == "voice":
                val = int(self._memobj.glob_settings.byte_79a0)
                self._memobj.glob_settings.byte_79a0 = (val & 0xF3) | (int(value) << 2)
                
            elif setting == "autolock":
                val = int(self._memobj.glob_settings.byte_79a1)
                if value:
                    self._memobj.glob_settings.byte_79a1 = val | 0x40
                else:
                    self._memobj.glob_settings.byte_79a1 = val & 0xBF

            elif setting == "beep":
                val = int(self._memobj.glob_settings.byte_79a2)
                if value:
                    self._memobj.glob_settings.byte_79a2 = val | 0x80
                else:
                    self._memobj.glob_settings.byte_79a2 = val & 0x7F
            elif setting == "roger":
                val = int(self._memobj.glob_settings.byte_79a2)
                self._memobj.glob_settings.byte_79a2 = (val & 0x9F) | (int(value) << 5)
                
            elif setting == "gnss":
                val = int(self._memobj.glob_settings.byte_79a3)
                if value:
                    self._memobj.glob_settings.byte_79a3 = val | 0x80
                else:
                    self._memobj.glob_settings.byte_79a3 = val & 0x7F
            elif setting == "radio_int":
                val = int(self._memobj.glob_settings.byte_79a3)
                if value:
                    self._memobj.glob_settings.byte_79a3 = val | 0x04
                else:
                    self._memobj.glob_settings.byte_79a3 = val & 0xFB

            elif setting == "display":
                val = int(self._memobj.glob_settings.byte_79a5)
                self._memobj.glob_settings.byte_79a5 = (val & 0xFC) | int(value)
            elif setting == "language":
                val = int(self._memobj.glob_settings.byte_79a5)
                self._memobj.glob_settings.byte_79a5 = (val & 0xFB) | (int(value) << 2)
            elif setting == "noaa_alert":
                val = int(self._memobj.glob_settings.byte_79a5)
                if value:
                    self._memobj.glob_settings.byte_79a5 = val | 0x08
                else:
                    self._memobj.glob_settings.byte_79a5 = val & 0xF7
                    
            elif setting == "tail":
                val = int(self._memobj.glob_settings.byte_79a6)
                self._memobj.glob_settings.byte_79a6 = (val & 0x1F) | (int(value) << 5)
            elif setting == "noaa_on":
                val = int(self._memobj.glob_settings.byte_79a6)
                if value:
                    self._memobj.glob_settings.byte_79a6 = val | 0x10
                else:
                    self._memobj.glob_settings.byte_79a6 = val & 0xEF
            elif setting == "wx_channel":
                self._memobj.glob_settings.wx_channel = int(value)
            elif setting == "dir":
                val = int(self._memobj.glob_settings.byte_79a6)
                self._memobj.glob_settings.byte_79a6 = (val & 0xF7) | (int(value) << 3)

            elif setting == "tot":
                self._memobj.glob_settings.tot = int(value)
            elif setting == "toa":
                self._memobj.glob_settings.toa = int(value)
            elif setting == "ptt_dly":
                self._memobj.dtmf.delay_tm = int(value)
            elif setting == "apo":
                self._memobj.glob_settings.apo = int(value)
            elif setting == "alert":
                self._memobj.glob_settings.alert_freq = int(value)
            elif setting == "timezone":
                self._memobj.glob_settings.gps_zone = int(value) + 12
                
            elif setting == "password_enabled":
                self._memobj.glob_settings.password_enabled = 0x01 if bool(value) else 0x00
            elif setting == "password":
                clean_pwd = "".join([c for c in str(value) if c.isdigit()])
                pwd_bytes = [ord(char) for char in clean_pwd]
                while len(pwd_bytes) < 8:
                    pwd_bytes.append(0xFF)
                for i in range(8):
                    self._memobj.power_on_pwd[i] = pwd_bytes[i]

            elif setting == "st_dtmf":
                self._memobj.dtmf.dtmf_st = 0x01 if bool(value) else 0x00
            elif setting == "st_2tone":
                self._memobj.st_2tone = 0x01 if bool(value) else 0x00
            elif setting == "st_5tone":
                self._memobj.st_5tone = 0x01 if bool(value) else 0x00
            elif setting == "st_bdc":
                self._memobj.st_bdc1200 = 0x01 if bool(value) else 0x00
                
            elif setting == "dtmf_id":
                clean_dtmf = "".join([c for c in str(value) if c.isdigit()])
                padded = clean_dtmf.ljust(3, '\xFF')
                for i in range(3):
                    char = padded[i]
                    self._memobj.dtmf.self_id[i] = int(char) if char != '\xFF' else 0xFF
            elif setting == "fivetone_id":
                clean_5t = "".join([c for c in str(value) if c.isdigit()])
                padded = clean_5t.ljust(5, '\xFF')
                for i in range(5):
                    char = padded[i]
                    self._memobj.fivetone_id[i] = int(char) if char != '\xFF' else 0xFF
            elif setting == "biis_id":
                self._memobj.biis_id = int(value)
                
            elif setting == "pf1_short":
                self._memobj.prog_key.pf1_short = int(value)
            elif setting == "pf2_short":
                self._memobj.prog_key.pf2_short = int(value)
            elif setting == "pf1_long":
                self._memobj.prog_key.pf1_long = int(value)
            elif setting == "pf2_long":
                self._memobj.prog_key.pf2_long = int(value)
                
            elif setting == "scan_mode":
                self._memobj.scan_menu.scan_mode = int(value)
            elif setting == "scan_memory":
                self._memobj.scan_menu.scan_memory = int(value)

            elif setting.startswith("zone_name_"):
                idx = int(setting.split("_")[-1])
                name_bytes = str(value).encode('ascii', 'replace')
                name_bytes = name_bytes.ljust(16, b'\xff')[:16]
                for i in range(16):
                    self._memobj.zones[idx].name[i] = name_bytes[i]
