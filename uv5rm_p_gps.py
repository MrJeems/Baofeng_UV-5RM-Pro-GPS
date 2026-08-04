import logging
import time
from chirp import chirp_common, errors, directory, memmap
from chirp.settings import (
    RadioSettingGroup, 
    RadioSetting, 
    RadioSettingValueBoolean, 
    RadioSettingValueList,
    RadioSettingValueString
)

LOG = logging.getLogger(__name__)

# --- GLOBAL CHIRP INJECTIONS FOR CSV EXPORT ---
_temp_tones = list(chirp_common.TONES)
for _tone in [63.0, 202.0]:
    if _tone not in _temp_tones:
        _temp_tones.append(_tone)
_temp_tones.sort()
chirp_common.TONES = tuple(_temp_tones)

_temp_dtcs = list(chirp_common.DTCS_CODES)
if 17 not in _temp_dtcs:
    _temp_dtcs.append(17)
    _temp_dtcs.sort()
chirp_common.DTCS_CODES = tuple(_temp_dtcs)
# ----------------------------------------------

XOR_KEY = 0x1D

def _xor(data):
    """XORs a byte string with the radio's secret key."""
    if isinstance(data, str):
        data = data.encode('latin1')
    return bytes([x ^ XOR_KEY for x in data])

@directory.register
class UV5RMProGPS(chirp_common.CloneModeRadio):
    """Baofeng UV-5RM Pro GPS"""
    VENDOR = "Baofeng"
    MODEL = "UV-5RM Pro GPS"
    BAUD_RATE = 19200
    
    def get_features(self):
        feat = chirp_common.RadioFeatures()
        feat.has_settings = True
        feat.has_bank = False
        feat.has_name = True
        
        feat.valid_bands = [
            (136000000, 174000000),
            (200000000, 260000000),
            (350000000, 399000000),
            (400000000, 600000000),
        ]
        feat.memory_bounds = (1, 999) 
        feat.valid_modes = ["FM", "NFM", "WFM"]
        feat.valid_power_levels = [
            chirp_common.PowerLevel("High", watts=10.0),
            chirp_common.PowerLevel("Mid", watts=5.0),
            chirp_common.PowerLevel("Low", watts=1.0)
        ]
        feat.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        feat.valid_cross_modes = ["Tone->Tone", "DTCS->", "->DTCS", "Tone->DTCS", "DTCS->Tone", "->Tone", "DTCS->DTCS"]
        feat.valid_tuning_steps = [2.5, 5.0, 6.25, 10.0, 12.5, 20.0, 25.0, 30.0, 50.0, 100.0]
        feat.valid_tones = list(chirp_common.TONES)
        feat.valid_dtcs_codes = list(chirp_common.DTCS_CODES)
        return feat

    def custom_read_block(self, address, size=4096):
        serial = self.pipe
        addr_hi = (address >> 8) & 0xFF
        addr_lo = address & 0xFF
        
        cmd = bytes([0x52, addr_hi, addr_lo, 0x00]) 
        serial.write(_xor(cmd))
        
        expected_len = 4 + size
        resp = b""
        
        timeout_loops = 0
        while len(resp) < expected_len and timeout_loops < 40:
            chunk = serial.read(expected_len - len(resp))
            if chunk:
                resp += chunk
            else:
                time.sleep(0.1)
                timeout_loops += 1
                
        if len(resp) < expected_len:
            raise errors.RadioError("Short read at 0x%04X" % address)
            
        unxored = _xor(resp)
        if unxored[0] != 0x57: 
            raise errors.RadioError("Radio rejected read at 0x%04X" % address)
            
        return unxored[4:]

    def sync_in(self):
        """
        [DEVELOPER NOTES FOR RE-INTEGRATING SYNC_OUT]
        ---------------------------------------------
        - WAKEUP: \x00*12 + \xFF*4 -> 'PROGRAM\x00' -> \xFF*8 -> 'INFORMATION' -> 'W'
        - LIMITS: DO NOT EXCEED 0xC000! (0xC000-0xFFFF = Factory Calibration)
        - END: Terminate with b'END\x00' (XOR'd)
        """
        serial = self.pipe
        serial.flushInput()
        
        try:
            serial.write(b'\x00' * 12 + b'\xFF' * 4)
            serial.read(1)
            serial.write(_xor(b'PROGRAM\x00'))
            serial.read(1) 
            serial.write(_xor(b'\xff' * 8))
            serial.read(1) 
            serial.write(_xor(b'INFORMATION'))
            serial.read(16)
            serial.write(_xor(b'R'))
            serial.read(1)
        except Exception as e:
            raise errors.RadioError("Handshake failed: %s" % e)
            
        MEM_SIZE = 0x10000 
        BLOCK_SIZE = 0x1000
        data = b""
        
        for addr in range(0, MEM_SIZE, BLOCK_SIZE):
            data += self.custom_read_block(addr, BLOCK_SIZE)
            status = chirp_common.Status()
            status.msg = "Extracting Radio Memory..."
            status.max = MEM_SIZE
            status.cur = addr + BLOCK_SIZE
            if self.status_fn:
                self.status_fn(status)
                
        self._mmap = memmap.MemoryMap(data)

    def get_settings(self):
        """
        [DEVELOPER NOTES: GLOBAL SETTINGS MAP]
        --------------------------------------
        0x0091: Dual Watch
        0x7988: Backlight
        0x7989: Brightness
        0x798A: MDF-A / MDF-B
        0x798B: Voice Prompt
        0x798D: Squelch
        0x798F: VOX Delay
        0x79A1: AutoLock
        0x79A2: Power On Display
        0x79B0-0x79B3: SK1 / SK2 Keys
        """
        # Pull the global block
        settings_data = self._mmap.get_packed()[0x7980:0x79C0]
        data = bytearray([x if isinstance(x, int) else ord(x) for x in settings_data])
        
        top = RadioSettingGroup("top", "Settings")
        basic = RadioSettingGroup("basic", "Radio Settings")
        
        # Dual Watch (Located in Radio Info block at 0x0091)
        dw_raw = self._mmap.get_packed()[0x0091]
        if dw_raw == 0x10:
            dw_val = 0 # OFF
        elif dw_raw == 0x30:
            dw_val = 2 # Signal Wait
        else:
            dw_val = 1 # Double Wait
        dw_opts = ["OFF", "Double Wait", "Signal Wait"]
        basic.append(RadioSetting("dual_watch", "Dual Watch", RadioSettingValueList(dw_opts, current_index=dw_val)))

        # Voice Prompt (0x798B: 0=Chinese, 1=OFF, 2=English)
        vp_raw = data[0x0B]
        if vp_raw > 2: vp_raw = 0
        # Reordered array so index 0 = Chinese, 1 = OFF, 2 = English
        vp_opts = ["Chinese", "OFF", "English"]
        basic.append(RadioSetting("voice", "Voice Prompt", RadioSettingValueList(vp_opts, current_index=vp_raw)))

        # Squelch
        sq_val = data[0x0D]
        if sq_val > 9: sq_val = 9
        sq_opts = ["OFF", "Level 1", "Level 2", "Level 3", "Level 4", 
                   "Level 5", "Level 6", "Level 7", "Level 8", "Level 9"]
        basic.append(RadioSetting("squelch", "Squelch Level", RadioSettingValueList(sq_opts, current_index=sq_val)))

        # Power Save
        ps_val = data[0x10]
        if ps_val > 3: ps_val = 0
        ps_opts = ["OFF", "1:1", "1:2", "1:4"]
        basic.append(RadioSetting("power_save", "Power Save", RadioSettingValueList(ps_opts, current_index=ps_val)))

        # VOX Delay (10 to 100 in steps of 5)
        vd_val = data[0x0F]
        vd_opts = [f"{x / 10.0}s" for x in range(10, 105, 5)]
        vd_idx = (vd_val - 10) // 5
        if vd_idx < 0 or vd_idx >= len(vd_opts): vd_idx = 0
        basic.append(RadioSetting("vox_delay", "VOX Delay", RadioSettingValueList(vd_opts, current_index=vd_idx)))

        # TOT
        tot_val = data[0x15]
        tot_opts = ["OFF"] + [f"{x}s" for x in range(15, 225, 15)]
        if tot_val >= len(tot_opts): tot_val = 0
        basic.append(RadioSetting("tot", "TOT", RadioSettingValueList(tot_opts, current_index=tot_val)))

        # TOA
        toa_val = data[0x16]
        toa_opts = ["OFF"] + [f"{x}s" for x in range(1, 11)]
        if toa_val >= len(toa_opts): toa_val = 0
        basic.append(RadioSetting("toa", "TOA", RadioSettingValueList(toa_opts, current_index=toa_val)))

        # Backlight
        bl_val = data[0x08]
        bl_opts = ["Always On"] + [f"{x}s" for x in range(5, 31)]
        bl_idx = 0 if bl_val == 0 else (bl_val - 4)
        if bl_idx < 0 or bl_idx >= len(bl_opts): bl_idx = 0
        basic.append(RadioSetting("backlight", "Backlight", RadioSettingValueList(bl_opts, current_index=bl_idx)))

        # Brightness (0-4 maps to 1-5)
        br_val = data[0x09]
        if br_val > 4: br_val = 4
        br_opts = ["1", "2", "3", "4", "5"]
        basic.append(RadioSetting("brightness", "Brightness", RadioSettingValueList(br_opts, current_index=br_val)))

        # Power On Display (0x79A2)
        pod_raw = data[0x22]
        if pod_raw == 0x20:
            pod_val = 0 # MESSAGE
        elif pod_raw == 0xE0:
            pod_val = 2 # VOLTAGE
        else:
            pod_val = 1 # PICTURE
        pod_opts = ["MESSAGE", "PICTURE", "VOLTAGE"]
        basic.append(RadioSetting("power_on_display", "Power on Display", RadioSettingValueList(pod_opts, current_index=pod_val)))

        # AutoLock (0x79A1)
        al_val = True if (data[0x21] == 0x40) else False
        basic.append(RadioSetting("autolock", "AutoLock", RadioSettingValueBoolean(al_val)))

        # MDF-A & MDF-B
        mdf_opts = ["Freq", "Name", "CH", "Freq+Name"]
        mdf_a_val = (data[0x0A] & 0xF0) >> 4
        mdf_b_val = data[0x0A] & 0x0F
        if mdf_a_val > 3: mdf_a_val = 0
        if mdf_b_val > 3: mdf_b_val = 0
        basic.append(RadioSetting("mdf_a", "MDF-A", RadioSettingValueList(mdf_opts, current_index=mdf_a_val)))
        basic.append(RadioSetting("mdf_b", "MDF-B", RadioSettingValueList(mdf_opts, current_index=mdf_b_val)))

        # Programmable Keys (SK1/SK2) - Rebuilt with exact known indexes!
        sk_map = {
            0: "None", 1: "SCAN", 2: "MONITOR", 3: "Unknown_3",
            4: "GNSS System", 5: "Zone", 6: "FM Radio", 7: "One Touch Search",
            8: "Unknown_8", 9: "1750Hz", 10: "Falling Alarm", 11: "Battery Display",
            12: "SOS", 13: "One Touch Call", 14: "Power", 15: "Vox"
        }
        # Build strict 16-item list so index lookups never crash
        sk_opts = [sk_map.get(i, f"Unknown {i}") for i in range(16)]
        
        basic.append(RadioSetting("sk1_press", "Press SK1", RadioSettingValueList(sk_opts, current_index=data[0x30] & 0x0F)))
        basic.append(RadioSetting("sk1_long", "LongPress SK1", RadioSettingValueList(sk_opts, current_index=data[0x31] & 0x0F)))
        basic.append(RadioSetting("sk2_press", "Press SK2", RadioSettingValueList(sk_opts, current_index=data[0x32] & 0x0F)))
        basic.append(RadioSetting("sk2_long", "LongPress SK2", RadioSettingValueList(sk_opts, current_index=data[0x33] & 0x0F)))

        # Power On MSG (Updated for Chinese Character Support)
        msg_raw = self._mmap.get_packed()[0x79D0:0x79E0]
        msg_bytes = bytes([x if isinstance(x, int) else ord(x) for x in msg_raw])
        clean_msg = msg_bytes.replace(b'\x00', b'').replace(b'\xFF', b'')
        
        # Attempt GB2312 decoding for Chinese characters, fallback to ASCII
        try:
            msg_str = clean_msg.decode('gb2312').strip()
        except UnicodeDecodeError:
            msg_str = clean_msg.decode('ascii', errors='ignore').strip()
            
        msg_setting = RadioSetting("power_on_msg", "Power on MSG", RadioSettingValueString(0, 16, msg_str))
        basic.append(msg_setting)

        top.append(basic)
        return top

    def get_memory(self, loc):
        BLOCK_SIZE = 0x30 
        BASE_ADDR = 0x0080
        addr = BASE_ADDR + ((loc - 1) * BLOCK_SIZE)
        mem = chirp_common.Memory()
        mem.number = loc
        
        try:
            raw_slice = self._mmap.get_packed()[addr:addr+BLOCK_SIZE]
            data = bytes([x if isinstance(x, int) else ord(x) for x in raw_slice])
        except Exception as e:
            LOG.error("Failed to read MemoryMap on CH %d: %s", loc, e)
            mem.empty = True
            return mem
            
        if len(data) < BLOCK_SIZE or data[0:4] in (b'\xff\xff\xff\xff', b'\x00\x00\x00\x00'):
            mem.empty = True
            return mem
            
        mem.empty = False
        
        try:
            freq_str = "%02X%02X%02X%02X" % (data[3], data[2], data[1], data[0])
            mem.freq = int(freq_str) * 10
            
            if mem.freq < 1000000 or mem.freq > 999999990:
                mem.empty = True
                return mem
                
            tx_str = "%02X%02X%02X%02X" % (data[7], data[6], data[5], data[4])
            tx_freq = int(tx_str) * 10
            
            if tx_freq == 0 or tx_freq == int("FFFFFFFF", 16) * 10 or tx_freq == mem.freq:
                mem.duplex = ""
                mem.offset = 0
            else:
                offset_val = tx_freq - mem.freq
                if offset_val > 0:
                    mem.duplex = "+"
                    mem.offset = offset_val
                else:
                    mem.duplex = "-"
                    mem.offset = abs(offset_val)
        except ValueError:
            mem.empty = True
            return mem
            
        mode_byte = data[0x10]
        
        if mode_byte & 0x80:
            mem.power = chirp_common.PowerLevel("High", watts=10.0)
        elif mode_byte & 0x40:
            mem.power = chirp_common.PowerLevel("Mid", watts=5.0)
        else:
            mem.power = chirp_common.PowerLevel("Low", watts=1.0)
            
        mem.mode = "WFM" if (mode_byte & 0x20) else "NFM"
        
        skip_val = data[0x12] & 0x03
        if skip_val == 0:
            mem.skip = ""   
        else:
            mem.skip = "S"  
            
        step_idx = data[0x18]
        step_floats = [2.5, 5.0, 6.25, 10.0, 12.5, 20.0, 25.0, 30.0, 50.0, 100.0]
        if step_idx < len(step_floats):
            mem.tuning_step = step_floats[step_idx]
            
        rx_raw = (data[0x08], data[0x09])
        tx_raw = (data[0x0A], data[0x0B])

        def parse_tone_or_dcs(byte1, byte2):
            if byte1 == 0xFF and byte2 == 0xFF:
                return "", 0.0
            if byte1 == 0x00 and byte2 == 0x00:
                return "", 0.0
                
            if byte1 & 0x80:  
                code = ((byte1 & 0x07) * 100) + int(f"{byte2:02X}")
                if code not in chirp_common.DTCS_CODES:
                    code = 23
                return "DCS", code
            else:  
                try:
                    val = float(f"{byte1:02X}{byte2:02X}") / 10.0
                    if 60.0 <= val <= 300.0:
                        return "CTCSS", val
                except ValueError:
                    pass
            return "", 0.0

        rx_type, rx_val = parse_tone_or_dcs(rx_raw[0], rx_raw[1])
        tx_type, tx_val = parse_tone_or_dcs(tx_raw[0], tx_raw[1])

        if rx_type == "CTCSS" and tx_type == "CTCSS":
            if rx_val == tx_val:
                mem.tmode = "TSQL"
                mem.ctone = rx_val
            else:
                mem.tmode = "Cross"
                mem.cross_mode = "Tone->Tone"
                mem.ctone = rx_val
                mem.rtone = tx_val
        elif rx_type == "CTCSS" and tx_type == "":
            mem.tmode = "Cross"
            mem.cross_mode = "->Tone"
            mem.ctone = rx_val
        elif rx_type == "" and tx_type == "CTCSS":
            mem.tmode = "Tone"
            mem.rtone = tx_val
        elif rx_type == "DCS" and tx_type == "DCS":
            mem.dtcs_polarity = "NN"
            if rx_val == tx_val:
                mem.tmode = "DTCS"
                mem.dtcs = tx_val
                mem.rx_dtcs = rx_val
            else:
                mem.tmode = "Cross"
                mem.cross_mode = "DTCS->DTCS"
                mem.rx_dtcs = rx_val
                mem.dtcs = tx_val
        elif rx_type == "DCS" and tx_type == "":
            mem.dtcs_polarity = "NN"
            mem.tmode = "Cross"
            mem.cross_mode = "->DTCS"
            mem.rx_dtcs = rx_val
        elif rx_type == "" and tx_type == "DCS":
            mem.dtcs_polarity = "NN"
            mem.tmode = "DTCS"
            mem.dtcs = tx_val
        elif rx_type == "CTCSS" and tx_type == "DCS":
            mem.dtcs_polarity = "NN"
            mem.tmode = "Cross"
            mem.cross_mode = "Tone->DTCS"
            mem.ctone = rx_val
            mem.dtcs = tx_val
        elif rx_type == "DCS" and tx_type == "CTCSS":
            mem.dtcs_polarity = "NN"
            mem.tmode = "Cross"
            mem.cross_mode = "DTCS->Tone"
            mem.rx_dtcs = rx_val
            mem.rtone = tx_val
        else:
            mem.tmode = ""

        name_bytes = data[0x20:0x30].strip(b'\x00\xff ')
        try:
            mem.name = name_bytes.decode('gb2312')
        except UnicodeDecodeError:
            mem.name = name_bytes.decode('ascii', errors='ignore')
        
        extra = RadioSettingGroup("extra", "Extra")
        
        bcl_val = True if (mode_byte & 0x04) else False
        extra.append(RadioSetting(
            "bcl", "Busy Channel Lockout",
            RadioSettingValueBoolean(bcl_val)))
            
        sig_raw = data[0x11]
        digital_flag = data[0x12] & 0x80
        
        sig_idx = 0
        if digital_flag:
            if sig_raw == 0:
                sig_idx = 4  
            elif sig_raw == 3:
                sig_idx = 5  
        else:
            if sig_raw <= 3:
                sig_idx = sig_raw
                
        sig_options = ["NONE", "DTMF", "2Tone", "5Tone", "MDC", "BDC1200"]
        extra.append(RadioSetting(
            "signaling", "Signaling",
            RadioSettingValueList(sig_options, current_index=sig_idx)))

        mute_idx = (data[0x12] & 0x60) >> 5
        mute_options = ["OFF", "QT", "Optional Signaling", "QT+DTMF"]
        if mute_idx < len(mute_options):
            extra.append(RadioSetting(
                "sp_mute", "SP-Mute",
                RadioSettingValueList(mute_options, current_index=mute_idx)))
                
        skip_options = ["OFF", "Normal", "Strict"]
        safe_skip_idx = skip_val if skip_val < len(skip_options) else 0
        extra.append(RadioSetting(
            "scan_skip", "Scan Skip",
            RadioSettingValueList(skip_options, current_index=safe_skip_idx)))
            
        mem.extra = extra
        
        return mem
