import sys
import logging

import numpy as np
from aravis import Camera


if __name__ == "__main__":
    #cam = ar.get_camera("Prosilica-02-2130A-06106")
    #cam = Camera("AT-Automation Technology GmbH-20805103")
    cam = Camera(loglevel=logging.DEBUG)
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = "frame.npy"

    #cam.start_acquisition_trigger()
    cam.start_acquisition_continuous()
    frame = cam.pop_frame()
    print("Saving frame to ", path)
    np.save(path, frame)
    cam.stop_acquisition()
