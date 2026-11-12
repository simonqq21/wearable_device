import struct, zlib 
from pathlib import Path

dest_path = Path("littlefs_data/haptics")
NUM_CHANNELS = 4
HAPTIC_SEQUENCE_FILE_EXTENSION = ".bin"

# define haptic pulse sequences structs
'''
typedef struct
{
    uint16_t duration_ms;
    uint16_t fade_to_ms;
    uint8_t values[NUM_CHANNELS];
} haptic_pulse_t;
'''
HAPTIC_PULSE_FMT = f"<HH{NUM_CHANNELS}B"
HAPTIC_PULSE = struct.Struct(HAPTIC_PULSE_FMT)
class HapticPulse:
    duration_ms: int
    fade_to_ms: int
    values: tuple[int]
    def __init__(self, 
                 duration_ms: int, 
                 fade_to_ms: int,
                 values: tuple[int]):
        self.duration_ms = duration_ms
        self.fade_to_ms = fade_to_ms 
        self.values = values
    
'''
typedef struct
{
    uint8_t len;
    haptic_pulse_t haptic_sequence[];
} haptic_sequence_t;
'''
HAPTIC_SEQUENCE_FMT = "<B"
HAPTIC_SEQUENCE = struct.Struct(HAPTIC_SEQUENCE_FMT)
class HapticSequence:
    filename: str
    seq_len: int
    haptic_sequence: list[HapticPulse]
    def __init__(self,
                 filename: str, 
                 seq_len: int, 
                 haptic_sequence: list[HapticPulse]):
        self.filename = filename 
        self.seq_len = seq_len
        self.haptic_sequence = haptic_sequence 

'''
@brief store array of haptic patterns into a binary file 

'''
def write_haptic_sequence_to_file(haptic_sequence: HapticSequence):
    # print(HAPTIC_SEQUENCE.size, haptic_sequence.seq_len, HAPTIC_PULSE.size)
    buf = bytearray(HAPTIC_SEQUENCE.size + int(haptic_sequence.seq_len) * HAPTIC_PULSE.size)
    # print(type(haptic_sequence.seq_len))
    
    HAPTIC_SEQUENCE.pack_into(buf, 0, int(haptic_sequence.seq_len))
    offset = HAPTIC_SEQUENCE.size
    
    for p in haptic_sequence.haptic_sequence:
        if len(p.values) != NUM_CHANNELS:
            raise ValueError(f"expected {NUM_CHANNELS} values, got {len(p.values)} values.")
        HAPTIC_PULSE.pack_into(buf, 
                               offset, 
                               p.duration_ms, 
                               p.fade_to_ms,
                               *p.values)
        offset += HAPTIC_PULSE.size
    print(" ".join(f"0x{b:02x}" for b in buf))
    
    complete_filepath = Path(dest_path, haptic_sequence.filename)
    print(f"Writing to {complete_filepath}...")
    with open(complete_filepath, "wb") as f:
        f.write(buf)

    print(f"Reading back from {complete_filepath}:")
    with open(complete_filepath, "rb") as f:
        readback = bytearray(f.read())
        print(" ".join(f"0x{b:02x}" for b in readback))
    
    print()

if __name__ == "__main__":
    # create destination folder 
    dest_path.mkdir(exist_ok=True)
    
    # define haptic pulse sequences to write.
    hs_start_stop = HapticSequence(filename=f"start{HAPTIC_SEQUENCE_FILE_EXTENSION}",
                        seq_len="2",
                        haptic_sequence=[
                            HapticPulse(duration_ms=1000,
                                        fade_to_ms=0,
                                        values = (255,255,255,255)),
                            HapticPulse(duration_ms=10,
                                        fade_to_ms=0,
                                        values = (0,0,0,0)),
                        ])

    hs_left = HapticSequence(filename=f"left{HAPTIC_SEQUENCE_FILE_EXTENSION}",
                        seq_len="2",
                        haptic_sequence=[
                            HapticPulse(duration_ms=1000,
                                        fade_to_ms=0,
                                        values = (255,255,0,0)),
                            HapticPulse(duration_ms=10,
                                        fade_to_ms=0,
                                        values = (0,0,0,0)),
                        ])

    hs_right = HapticSequence(filename=f"right{HAPTIC_SEQUENCE_FILE_EXTENSION}",
                        seq_len="2",
                        haptic_sequence=[
                            HapticPulse(duration_ms=1000,
                                        fade_to_ms=0,
                                        values = (0,0,255,255)),
                            HapticPulse(duration_ms=10,
                                        fade_to_ms=0,
                                        values = (0,0,0,0)),
                        ])
    hs_2 = HapticSequence(filename=f"point01{HAPTIC_SEQUENCE_FILE_EXTENSION}",
                        seq_len="8",
                        haptic_sequence=[
                            HapticPulse(duration_ms=250,
                                        fade_to_ms=0,
                                        values = (255,255,255,255)),
                            HapticPulse(duration_ms=250,
                                        fade_to_ms=0,
                                        values = (0,0,0,0)),
                            HapticPulse(duration_ms=250,
                                        fade_to_ms=0,
                                        values = (255,255,255,255)),
                            HapticPulse(duration_ms=250,
                                        fade_to_ms=0,
                                        values = (0,0,0,0)),
                            HapticPulse(duration_ms=250,
                                        fade_to_ms=0,
                                        values = (255,255,255,255)),
                            HapticPulse(duration_ms=250,
                                        fade_to_ms=0,
                                        values = (0,0,0,0)),
                            HapticPulse(duration_ms=250,
                                        fade_to_ms=0,
                                        values = (255,255,255,255)),
                            HapticPulse(duration_ms=250,
                                        fade_to_ms=0,
                                        values = (0,0,0,0)),
                        ])
    write_haptic_sequence_to_file(hs_start_stop)
    write_haptic_sequence_to_file(hs_left)
    write_haptic_sequence_to_file(hs_right)
    write_haptic_sequence_to_file(hs_2)