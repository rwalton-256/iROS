import glob
import time
import socket

images = []

for file in glob.glob("data/*.jpg"):
    with open(f"{file}",'rb') as f:
        images.append( f.read() )

FRAME_RATE = 60
DELTA_T = 1 / FRAME_RATE

tt = time.time()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(('localhost', 8891))
    while True:
        for im in images:
            time.sleep( max( 0, tt - time.time() ) )
            tt += DELTA_T
            print(len(im))
            s.send(
                # First entry in the header is number of bytes of the payload (uint32_t)
                int(len(im)).to_bytes( length=4, byteorder='big', signed=False ) +
                im 
            )

