import numpy as np, trimesh, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from trimesh.ray.ray_pyembree import RayMeshIntersector
from room_sensor_case_lite import *

def scene_mesh(items):
    meshes, cols = [], []
    for man, color, off in items:
        tm = to_trimesh(man.translate(list(off)))
        meshes.append(tm); cols.append(np.tile(matplotlib.colors.to_rgb(color), (len(tm.faces),1)))
    return trimesh.util.concatenate(meshes), np.vstack(cols)

def render(items, fname, elev, azim, title, W=1000, H=760, pad=1.08):
    mesh, fcol = scene_mesh(items)
    e, a = np.radians(elev), np.radians(azim)
    d = -np.array([np.cos(e)*np.sin(a), np.sin(e), np.cos(e)*np.cos(a)])   # Y-up camera, azim from +Z (front)
    up = np.array([0,1,0]) if abs(d[1])<0.95 else np.array([0,0,-1])
    r = np.cross(d, up); r/=np.linalg.norm(r); u = np.cross(r, d)
    c = mesh.bounds.mean(0); ext = np.linalg.norm(mesh.extents)/2*pad
    asp = W/H
    xs = np.linspace(-ext*asp, ext*asp, W); ys = np.linspace(ext, -ext, H)
    X, Y = np.meshgrid(xs, ys)
    origins = c + X.reshape(-1,1)*r + Y.reshape(-1,1)*u - d*ext*4
    dirs = np.tile(d, (len(origins),1))
    inter = RayMeshIntersector(mesh)
    tri = inter.intersects_first(origins, dirs)
    img = np.ones((W*H,3))
    hit = tri >= 0
    n = mesh.face_normals[tri[hit]]
    n = np.where((n@d)[:,None] > 0, -n, n)
    L1 = -d + 0.6*u + 0.3*r; L1/=np.linalg.norm(L1)
    sh = 0.35 + 0.65*np.clip(n@L1,0,1)
    img[hit] = np.clip(fcol[tri[hit]]*sh[:,None] + 0.05, 0, 1)
    img = img.reshape(H,W,3)
    # edge darkening from depth discontinuity via triangle id change
    t = tri.reshape(H,W); nrm = np.zeros((H,W,3)); nrm[hit.reshape(H,W)] = n
    edge = np.zeros((H,W),bool)
    dn = np.linalg.norm(np.diff(nrm,axis=0),axis=2)>0.3; edge[1:]|=dn
    dn = np.linalg.norm(np.diff(nrm,axis=1),axis=2)>0.3; edge[:,1:]|=dn
    img[edge] *= 0.55
    plt.figure(figsize=(W/100,H/100),dpi=100); plt.imshow(img); plt.axis('off'); plt.title(title, fontsize=13)
    plt.tight_layout(); plt.savefig(fname); plt.close()


if __name__ == "__main__":
    base, lid = make_base(), make_lid()
    pcb = board_solid()
    for (u, v) in HOLES: pcb -= cyl(bx(u), by(v), 0, 30, 2.2)
    P = parts_solids()
    radar = P.pop("radar"); xiao = P.pop("XIAO") + P.pop("USB-C")
    small = M()
    for s in P.values(): small += s
    Z = (0, 0, 0)
    brd = [(pcb, '#2e7d32', Z), (xiao, '#3a3a3a', Z), (radar, '#2b4c9b', Z), (small, '#c9a227', Z)]
    render([(base, '#d8d2c2', Z)] + brd, 'l1_base_board.png', 0, 0, 'Base with board fitted (lid off), seen from the room')
    render([(lid, '#ece6d6', Z)], 'l2_lid_inside.png', 40, 195, 'Lid from inside: radar pads, light tube, SHT40 chamber, posts')
    render([(base, '#d8d2c2', Z)] + brd + [(lid, '#ece6d6', (0, 0, 25))], 'l3_exploded.png', 28, 30, 'Exploded view')
    render([(base, '#d8d2c2', Z), (lid, '#ece6d6', Z)], 'l4_front.png', -18, 25, 'Closed: front + bottom (USB-C, vents)')
    render([(base, '#d8d2c2', Z), (lid, '#ece6d6', Z)], 'l5_back.png', 20, 160, 'Back: keyholes + 2 x M3')
    render([(make_foot(), '#9fb3bf', Z), (base, '#d8d2c2', Z), (lid, '#ece6d6', Z)], 'l6_foot.png', 15, 40, 'On the desk foot')
