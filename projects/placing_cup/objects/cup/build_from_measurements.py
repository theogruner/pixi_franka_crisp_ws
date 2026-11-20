"""The cup from its measurements: rim 97 mm, foot 67 mm, height 65 mm, handle 41 mm high and 26 mm
out (a D loop just below the rim). A rounded bowl (radius rises fast above the foot, nearly vertical at
the rim), 3.5 mm wall, floor 6 mm; frame as the scanned model: origin at the box centre, z up, the
handle at HANDLE_AZ (deg, world) as it stood in the scan.
  python build_from_measurements.py out.ply 3.0   (then decimated to 6000 triangles: mesh.ply)"""
import os, sys, numpy as np
from scipy import ndimage
from scipy.interpolate import splprep, splev
from scipy.spatial import cKDTree
from skimage import measure
RT, RB, H = 0.0485, 0.0335, 0.065
WALL, FLOOR, TUBE = 0.0035, 0.006, 0.0035
H_TOP, H_LEN, H_OUT = H - float(sys.argv[4] if len(sys.argv) > 4 else 0.005), 0.041, 0.026     # handle: top attachment, height, reach (outer)
AZ = np.radians(float(sys.argv[3]) if len(sys.argv) > 3 else -114.7)
EXP = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
def r_out(z):
    t = np.clip(z / H, 0, 1)
    return RB + (RT - RB) * (1 - (1 - t) ** EXP)
v = 0.0005
lo, hi = np.array([-0.09, -0.09, -0.002]), np.array([0.09, 0.09, H + 0.002])
shape = np.ceil((hi - lo) / v).astype(int)
gx, gy, gz = [lo[i] + v * (np.arange(shape[i]) + 0.5) for i in range(3)]
X, Y, Z = np.meshgrid(gx, gy, gz, indexing="ij")
RR = np.hypot(X, Y)
ro = r_out(Z)
body = (RR <= ro) & (Z >= 0) & (Z <= H)
# the inside: a rounded bowl, z_in(r) = Z0 + (H - Z0) (r / r_rim_in)^NIN (measured live: ~10 mm above
# the table 20-30 mm from the axis, ~19 mm at 30-40 mm), and at least WALL inside the outer surface
Z0, NIN = float(os.environ.get("Z0", 0.012)), float(os.environ.get("NIN", 4.0))
z_in = Z0 + (H - Z0) * np.clip(RR / (RT - WALL), 0, 1) ** NIN
cavity = (RR < ro - WALL) & (Z > z_in)
body &= ~cavity
# handle: tube along a D loop in the vertical plane at AZ; outer edge H_OUT beyond the body
zt, zb = H_TOP, H_TOP - H_LEN
def rb(z): return float(r_out(z))
reach = lambda z: rb(z) + H_OUT - TUBE        # tube centre at the outer bar
zm = (zt + zb) / 2
pts = np.array([(rb(zt) - 0.003, zt), (rb(zt) + 0.010, zt + 0.0025), (reach(zt) - 0.002, zt - 0.006),
                (reach(zm + 0.008), zm + 0.008), (reach(zm - 0.008), zm - 0.008),
                (reach(zb) - 0.003, zb + 0.005), (rb(zb) + 0.010, zb - 0.0015), (rb(zb) - 0.003, zb + 0.002)])
tck, _ = splprep(pts.T, s=0, k=3)
rz = np.array(splev(np.linspace(0, 1, 600), tck)).T
curve = np.c_[rz[:, 0] * np.cos(AZ), rz[:, 0] * np.sin(AZ), rz[:, 1]]
near = cKDTree(curve).query(np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1), distance_upper_bound=0.01)[0].reshape(shape)
handle = (near <= TUBE) & ~cavity
solid = ndimage.gaussian_filter((body | handle).astype(np.float32), 0.7)
verts, faces, _, _ = measure.marching_cubes(solid, 0.5, spacing=(v, v, v))
verts = verts + lo
centre = (verts.min(0) + verts.max(0)) / 2
verts -= centre
print(f"extent mm {np.round(1000 * np.ptp(verts, 0), 1)}, rim {2000*RT:.0f} foot {2000*RB:.0f} height {1000*H:.0f}, "
      f"handle z {1000*zb:.0f}-{1000*zt:.0f} reach {1000*H_OUT:.0f}, centre offset {np.round(1000*centre,1)}")
with open(sys.argv[1], "wb") as f:
    f.write((f"ply\nformat binary_little_endian 1.0\nelement vertex {len(verts)}\nproperty float x\nproperty float y\n"
             f"property float z\nelement face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n").encode())
    f.write(verts.astype("<f4").tobytes())
    rec = np.zeros(len(faces), dtype=[("n", "u1"), ("i", "<i4", 3)])
    rec["n"], rec["i"] = 3, faces[:, ::-1]
    f.write(rec.tobytes())
