"""Build a small FAT32 image (superfloppy, no MBR) for tb_slave_audio.v.

Files in the root directory:
  VID1.RAW    2 frames of 320x240 RGB332, byte = (frame*61 + x*3 + y*7) & 0xFF
  AUDIO1.WAV  48 kHz PCM16 stereo, sample i: left = 0x1000+i, right = ~left
  AUDIO2.WAV  same format, sample i: left = 0x6000+i, right = left ^ 0x5555
Both WAV files carry a LIST chunk before 'data', like the real files.
Writes sd_image.hex (one byte per line) and prints the layout.
"""
import struct

SPC = 8              # sectors per cluster
RESERVED = 32
FAT_SECTORS = 1      # 128 cluster entries
TOTAL = 512
A1_SAMPLES = 1200
A2_SAMPLES = 1800


def wav(samples, fn):
    data = b''.join(struct.pack('<hh', *fn(i)) for i in range(samples))
    lst = b'INFO' + b'ISFT' + struct.pack('<I', 14) + b'Lavf62.3.100\x00\x00'
    fmt = struct.pack('<HHIIHH', 1, 2, 48000, 48000 * 4, 4, 16)
    body = (b'WAVE' + b'fmt ' + struct.pack('<I', 16) + fmt +
            b'LIST' + struct.pack('<I', len(lst)) + lst +
            b'data' + struct.pack('<I', len(data)) + data)
    return b'RIFF' + struct.pack('<I', len(body)) + body


def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


vid = bytes((f * 61 + x * 3 + y * 7) & 0xFF
            for f in range(2) for y in range(240) for x in range(320))
a1 = wav(A1_SAMPLES, lambda i: (s16(0x1000 + i), s16(~(0x1000 + i))))
a2 = wav(A2_SAMPLES, lambda i: (s16(0x6000 + i), s16((0x6000 + i) ^ 0x5555)))

img = bytearray(TOTAL * 512)
bs = bytearray(512)
bs[0:3] = b'\xEB\x58\x90'
bs[3:11] = b'MSWIN4.1'
struct.pack_into('<HBHBHHBHHHII', bs, 11, 512, SPC, RESERVED, 2, 0, 0, 0xF8,
                 0, 63, 255, 0, TOTAL)
struct.pack_into('<IHHI', bs, 36, FAT_SECTORS, 0, 0, 2)
bs[510:512] = b'\x55\xAA'
img[0:512] = bs

data_start = RESERVED + 2 * FAT_SECTORS
fat = [0x0FFFFFF8, 0x0FFFFFFF] + [0] * 126
next_free = 2


def alloc(nbytes):
    global next_free
    n = max(1, -(-nbytes // (SPC * 512)))
    first = next_free
    for k in range(n):
        fat[first + k] = first + k + 1 if k < n - 1 else 0x0FFFFFFF
    next_free += n
    return first


def put(cluster, payload):
    off = (data_start + (cluster - 2) * SPC) * 512
    img[off:off + len(payload)] = payload


root = alloc(512)
entries = b''
for name, payload in ((b'VID1    RAW', vid), (b'AUDIO1  WAV', a1), (b'AUDIO2  WAV', a2)):
    c = alloc(len(payload))
    put(c, payload)
    e = bytearray(32)
    e[0:11] = name
    e[11] = 0x20
    struct.pack_into('<H', e, 20, c >> 16)
    struct.pack_into('<H', e, 26, c & 0xFFFF)
    struct.pack_into('<I', e, 28, len(payload))
    entries += e
    print('%s cluster %d size %d' % (name.decode(), c, len(payload)))
put(root, entries + bytes(32))
fatb = b''.join(struct.pack('<I', v) for v in fat)
for k in range(2):
    off = (RESERVED + k * FAT_SECTORS) * 512
    img[off:off + 512] = fatb
assert data_start + (next_free - 2) * SPC <= TOTAL, 'image too small'
with open('sd_image.hex', 'w') as f:
    f.write('\n'.join('%02x' % b for b in img) + '\n')
print('sectors used', data_start + (next_free - 2) * SPC, 'of', TOTAL)
