"""
1. Read trajory file in standardised format
2. Downsample trajectory
3. Create rays from current location to downsampled points
4. Project to 2D image plane

"""
import os
import time
import numpy as np
import pandas as pd
import astrix as at

def read_varda_traj(file_path, test_time_adjustment=False):
    df = pd.read_csv(os.path.expanduser(file_path))
    t = at.Time(df["Time_From_Separation_s"])
    ecef = np.array([df["ECEF_x_m"], df["ECEF_y_m"], df["ECEF_z_m"]]).T
    pt = at.Point(ecef, t)
    path_nom = at.Path(pt)
    if test_time_adjustment:
        unix_30s = time.time() - path_nom.time_at_alt(110e3).unix[0] + 30 
        t = t.offset(unix_30s)
        pt = at.Point(ecef, t)
        path_nom = at.Path(pt)
    path_nom = path_nom.truncate(
        start_time=path_nom.time_at_alt(100e3), end_time=path_nom.time_at_alt(20e3)
    )
    path_nom = at.utils.defeature_path(path_nom, tol = 50)
    return path_nom

