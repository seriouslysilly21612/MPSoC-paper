#!/usr/bin/env python3
"""Step 1 of the desktop simulation (2026-11-27): are the rendered images good enough for the detector?

Renders the bench of the paper in MuJoCo from the calibrated camera (the base -> camera_link TF of
pick_place_vitis_ai.launch.py, RealSense D435i colour 848x480, fx = fy = 612 px: an assumed
intrinsic, not read from the board): the Indy7 (the STL meshes of indy7/models), a 36h11 tag on the
tool, an apple moving on the table. The ground truth of every frame goes to truth.csv. Then it runs
  - the tag detector of the pipeline (target_3d_pkg/tool_tag.py: AprilTag 36h11, cv2.aruco), and
  - a YOLO detector: --onnx MODEL (the 5-class v5n of the board, exported to ONNX, input 416 or
    320) or, without it, ultralytics yolov5nu COCO (class "apple") as a stand-in,
and reports how often each finds its object and how far from the truth (pixels).

  python render_check.py --out OUT [--frames 300] [--apple-obj textured.obj] [--onnx MODEL.onnx
                         --classes apple,...] [--hover-x -0.13] [--show]

Windows or Linux, Python 3.10+: pip install mujoco opencv-contrib-python numpy
  (+ onnxruntime for --onnx, or ultralytics for the stand-in).
Rendering is offscreen; on Linux without a display set MUJOCO_GL=egl or osmesa.
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys

import cv2
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.normpath(os.path.join(HERE, '..', '..', 'indy7', 'models'))

W, H, FX = 848, 480, 612.0                  # colour stream of realsense_pick_place.yaml; FX assumed
# base_link -> camera_link, pick_place_vitis_ai.launch.py (eye-to-hand calibration 2026-07-27)
CAM_T = np.array([0.811666, 0.087684, 0.924974])
CAM_Q_XYZW = np.array([0.72462829, 0.00008362, -0.68706199, 0.05347582])
TAG_ID, TAG_SIZE = 0, 0.065                 # the tag on the Indy7: 65 mm (I-109); id assumed 0
TAG_OFFSET = np.array([-0.085, 0.0, 0.0])   # INDY7.cfg TAG_OFFSET_M, in the frame of the tool (tcp)
APPLE_R = 0.04
TABLE_Z = 0.125                             # the apple centre of I-112 was at z 0.165 = table + r
HOVER = np.array([-0.05, 0.0, 0.20])       # tool goal = apple + this (INDY7.cfg APP_HOVER_XY_M, APP_HOVER_M)
Q0 = np.radians([0, -20, -100, 0, -60, 0])  # the start of the IK


def quat_to_mat(q_xyzw):
    x, y, z, w = q_xyzw / np.linalg.norm(q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tag_texture(path):
    """the 36h11 marker with a white border, as a png MuJoCo can load"""
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dic, TAG_ID, 400) if hasattr(cv2.aruco, 'generateImageMarker') \
        else cv2.aruco.drawMarker(dic, TAG_ID, 400)
    img = cv2.copyMakeBorder(marker, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    return 600 / 400                        # the white plate is 1.5 x the black square


def apple_texture(path):
    """a red apple skin with streaks and a highlight, so the sphere is not a flat red disc"""
    rng = np.random.default_rng(1)
    h, w = 256, 512
    v = np.linspace(0, 1, h)[:, None]
    base = np.zeros((h, w, 3), np.float32)
    base[..., 2] = 0.75 + 0.15 * rng.random((h, w))                   # R (BGR order)
    base[..., 1] = 0.10 + 0.25 * (v ** 3) + 0.05 * rng.random((h, w))  # yellow towards a pole
    base[..., 0] = 0.08
    streaks = cv2.GaussianBlur(rng.random((h, w)).astype(np.float32), (1, 31), 0)
    base[..., 1] += 0.15 * (streaks - 0.5)
    cv2.imwrite(path, np.clip(base * 255, 0, 255).astype(np.uint8))


def build(out, apple_obj=''):
    spec = mujoco.MjSpec.from_file(os.path.join(MODELS, 'indy7_mujoco.xml'))
    spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
    spec.visual.quality.shadowsize = 4096
    for i in range(7):
        spec.add_mesh(name=f'indy{i}', file=os.path.join(MODELS, 'meshes', f'Indy7_{i}.stl'))
    spec.add_material(name='indy', rgba=[0.86, 0.87, 0.88, 1], specular=0.4, shininess=0.5)
    spec.add_material(name='indyblue', rgba=[0.15, 0.35, 0.65, 1], specular=0.3)
    for i in range(7):
        body = spec.body(f'link{i}')
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f'indy{i}',
                      material='indyblue' if i in (1, 3) else 'indy', contype=0, conaffinity=0)
    # the tag on the tool: a thin plate, its face along -z of the tool (up when the tool looks down)
    plate = tag_texture(os.path.join(out, 'tag36h11.png'))
    spec.add_texture(name='tag', type=mujoco.mjtTexture.mjTEXTURE_2D,
                     file=os.path.join(out, 'tag36h11.png'))
    mat = spec.add_material(name='tag', specular=0.05)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'tag'
    half = TAG_SIZE * plate / 2
    tcp = spec.body('tcp')
    tcp.add_geom(name='tag', type=mujoco.mjtGeom.mjGEOM_BOX, size=[half, half, 0.001],
                 pos=list(TAG_OFFSET + [0, 0, -0.012]), euler=[math.pi, 0, 0], material='tag',
                 contype=0, conaffinity=0)
    # table (grey board with holes is not modelled: a matte grey plate) and the apple
    spec.add_texture(name='table', type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.62, 0.63, 0.64], rgb2=[0.58, 0.59, 0.60], width=512, height=512)
    tmat = spec.add_material(name='table', texrepeat=[20, 20], specular=0.1)
    tmat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'table'
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.6, 0.6, 0.01],
                            pos=[0.7, 0.0, TABLE_Z - 0.01], material='table')
    apple = spec.worldbody.add_body(name='apple', mocap=True, pos=[0.75, 0.06, TABLE_Z + APPLE_R])
    if apple_obj:
        # a scanned apple with its photo texture (e.g. YCB 013_apple google_16k: textured.obj +
        # texture_map.png in the same folder); MuJoCo centres the mesh, so the body is its centre
        tex = os.path.join(os.path.dirname(apple_obj), 'texture_map.png')
        spec.add_texture(name='apple', type=mujoco.mjtTexture.mjTEXTURE_2D, file=tex)
        amat = spec.add_material(name='apple', specular=0.3, shininess=0.4)
        amat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'apple'
        spec.add_mesh(name='apple', file=apple_obj)
        apple.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='apple', material='apple',
                       contype=0, conaffinity=0)
    else:
        apple_texture(os.path.join(out, 'apple.png'))
        spec.add_texture(name='apple', type=mujoco.mjtTexture.mjTEXTURE_2D, file=os.path.join(out, 'apple.png'))
        amat = spec.add_material(name='apple', specular=0.5, shininess=0.6)
        amat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'apple'
        apple.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[APPLE_R, 0, 0], material='apple',
                       contype=0, conaffinity=0)
        apple.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.003, 0.008, 0], pos=[0, 0, APPLE_R + 0.006],
                       rgba=[0.35, 0.22, 0.1, 1], contype=0, conaffinity=0)
    # light: a window on one side (the bench photo) and room light
    spec.worldbody.add_light(pos=[1.6, 0.6, 2.0], dir=[-0.6, -0.2, -1], diffuse=[0.7, 0.7, 0.7], castshadow=True)
    spec.worldbody.add_light(pos=[0.5, -0.8, 2.5], dir=[0, 0.3, -1], diffuse=[0.35, 0.35, 0.35], castshadow=False)
    # the camera: MuJoCo looks along -z with x right and y up; camera_link has x forward, y left, z up
    R = quat_to_mat(CAM_Q_XYZW)
    xaxis, yaxis = -R[:, 1], R[:, 2]
    spec.worldbody.add_camera(name='d435', pos=list(CAM_T), xyaxes=list(xaxis) + list(yaxis),
                              fovy=math.degrees(2 * math.atan(H / 2 / FX)))
    return spec.compile()


def project(p_world, m, d, cam):
    """pixel (u, v) of a world point in camera cam, and its depth [m]"""
    pos, mat = d.cam_xpos[cam], d.cam_xmat[cam].reshape(3, 3)
    pc = mat.T @ (p_world - pos)            # camera frame of MuJoCo: x right, y up, -z forward
    z = -pc[2]
    return np.array([W / 2 + FX * pc[0] / z, H / 2 - FX * pc[1] / z]), z


def ik(m, d, goal, q):
    """joint angles with the tcp at goal and the tool z axis straight down (damped least squares)"""
    tcp = m.body('tcp').id
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    for _ in range(100):
        d.qpos[:6] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        ep = goal - d.xpos[tcp]
        z = d.xmat[tcp].reshape(3, 3)[:, 2]
        er = np.cross(z, [0, 0, -1])
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(er) < 1e-3:
            break
        mujoco.mj_jacBody(m, d, jp, jr, tcp)
        J = np.vstack([jp[:, :6], jr[:, :6]])
        e = np.concatenate([ep, er])
        q = q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
    return q


def make_yolo(args):
    if args.onnx:
        import onnxruntime as ort
        sess = ort.InferenceSession(args.onnx, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        inp = sess.get_inputs()[0]
        size = inp.shape[2] if isinstance(inp.shape[2], int) else 416
        names = args.classes.split(',')

        def run(bgr):
            s = size / max(W, H)
            img = cv2.resize(bgr, (round(W * s), round(H * s)))
            pad = np.full((size, size, 3), 114, np.uint8)
            pad[:img.shape[0], :img.shape[1]] = img
            x = pad[..., ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255
            out = sess.run(None, {inp.name: x})[0][0]           # (N, 5 + classes), v5 layout
            best = []
            for row in out:
                conf = row[4] * row[5:].max()
                if conf > 0.25:
                    cx, cy, bw, bh = row[:4] / s
                    best.append((conf, names[int(row[5:].argmax())], (cx, cy, bw, bh)))
            return best
        return run, f'onnx {os.path.basename(args.onnx)} @ {size}'
    from ultralytics import YOLO
    model = YOLO('yolov5nu.pt')

    def run(bgr):
        r = model(bgr, imgsz=416, verbose=False)[0]
        return [(float(b.conf), r.names[int(b.cls)], tuple(b.xywh[0].tolist())) for b in r.boxes]
    return run, 'ultralytics yolov5nu (COCO), 416 - a stand-in for the board model'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', type=int, default=300, help='30 fps: 300 = 10 s')
    ap.add_argument('--speed', type=float, default=0.10, help='apple speed on the table [m/s]')
    ap.add_argument('--onnx', default='')
    ap.add_argument('--classes', default='apple,banana,orange,tennis_ball,cup')
    ap.add_argument('--meta', default='',
                    help='decode_meta.json of the compiled model: class names (in order) from it, overrides --classes')
    ap.add_argument('--hover-x', type=float, default=-0.13,
                    help='tool goal x offset from the apple [m]; -0.13: tag and apple both in view (cfg -0.05: the tag hides the apple from above)')
    ap.add_argument('--apple-obj', default='',
                    help='a textured apple mesh (.obj with texture_map.png beside it, e.g. YCB 013_apple); '
                         'without it a red sphere, which COCO YOLO does not call an apple')
    ap.add_argument('--no-yolo', action='store_true')
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--yolo-pass', action='store_true', help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.meta:
        a.classes = ','.join(json.load(open(a.meta))['names'])
    os.makedirs(os.path.join(a.out, 'frames'), exist_ok=True)
    if a.yolo_pass:
        return yolo_pass(a)

    m = build(a.out, a.apple_obj)
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, H, W)
    cam = m.camera('d435').id
    sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', '..', '..', 'ros2_ws', 'src', 'target_3d_pkg')))
    from target_3d_pkg.tool_tag import aruco_detector   # the detector of the pipeline (36h11, cv2.aruco)
    detect_tags = aruco_detector('subpix')

    # pass 1: render, truth, tag. Every image goes to OUT/all; pass 2 (YOLO) runs in its own process:
    # a GL context and torch in one process crashed (OSMesa + triton)
    os.makedirs(os.path.join(a.out, 'all'), exist_ok=True)
    rows, n_tag, tag_err = [], 0, []
    q = Q0.copy()
    for k in range(a.frames):
        t = k / 30.0
        # the apple goes back and forth along y on the table
        y = 0.06 + 0.15 * math.sin(2 * math.pi * a.speed * t / 0.6)
        d.mocap_pos[0] = [0.75, y, TABLE_Z + APPLE_R]
        # the arm follows the apple 150 ms late (a tracking arm), at the hover point
        y_late = 0.06 + 0.15 * math.sin(2 * math.pi * a.speed * (t - 0.15) / 0.6)
        q = ik(m, d, np.array([0.75 + a.hover_x, y_late, TABLE_Z + APPLE_R + HOVER[2]]), q)
        d.qpos[:6] = q
        mujoco.mj_forward(m, d)
        r.update_scene(d, camera=cam)
        bgr = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
        tag_w = d.geom_xpos[m.geom('tag').id].copy()
        uv_tag, _ = project(tag_w, m, d, cam)
        uv_apple, z_apple = project(d.mocap_pos[0].copy(), m, d, cam)
        corners, ids = detect_tags(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        found_tag = ids is not None and TAG_ID in [int(i) for i in np.ravel(ids)]
        if found_tag:
            n_tag += 1
            c = corners[[int(i) for i in np.ravel(ids)].index(TAG_ID)].reshape(4, 2).mean(0)
            tag_err.append(float(np.linalg.norm(c - uv_tag)))
        cv2.imwrite(os.path.join(a.out, 'all', f'{k:05d}.png'), bgr)
        rows.append(dict(frame=k, t_s=f'{t:.4f}', apple_x=f'{d.mocap_pos[0][0]:.4f}', apple_y=f'{y:.4f}',
                         apple_z=f'{d.mocap_pos[0][2]:.4f}', apple_u=f'{uv_apple[0]:.1f}', apple_v=f'{uv_apple[1]:.1f}',
                         apple_depth=f'{z_apple:.4f}', tag_x=f'{tag_w[0]:.4f}', tag_y=f'{tag_w[1]:.4f}',
                         tag_z=f'{tag_w[2]:.4f}', tag_u=f'{uv_tag[0]:.1f}', tag_v=f'{uv_tag[1]:.1f}',
                         tag_found=int(found_tag)))
    r.close()
    write_csv(os.path.join(a.out, 'truth.csv'), rows)
    med = lambda v: f'{np.median(v):.1f} px' if v else '-'
    lines = [f'render_check: {len(rows)} frames {W}x{H}, fx {FX} (assumed), apple {a.speed} m/s, '
             f'tool goal x {a.hover_x:+.2f} m from the apple',
             f'tag 36h11 id {TAG_ID} {TAG_SIZE * 1e3:.0f} mm: found {n_tag}/{len(rows)}, centre error median {med(tag_err)}']
    open(os.path.join(a.out, 'summary.txt'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    if not a.no_yolo:
        cmd = [sys.executable, os.path.abspath(__file__), '--out', a.out, '--yolo-pass', '--classes', a.classes]
        if a.onnx:
            cmd += ['--onnx', a.onnx]
        if a.show:
            cmd += ['--show']
        env = {k: v for k, v in os.environ.items() if k not in ('MUJOCO_GL', 'PYOPENGL_PLATFORM')}
        sys.exit(subprocess.call(cmd, env=env))


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def yolo_pass(a):
    """pass 2: YOLO on OUT/all/*.png against OUT/truth.csv; adds to truth.csv and summary.txt"""
    rows = list(csv.DictReader(open(os.path.join(a.out, 'truth.csv'))))
    yolo, yolo_name = make_yolo(a)
    n_apple, apple_err = 0, []
    for k, row in enumerate(rows):
        bgr = cv2.imread(os.path.join(a.out, 'all', f'{k:05d}.png'))
        uv_apple = np.array([float(row['apple_u']), float(row['apple_v'])])
        row['apple_found'], row['apple_conf'] = 0, '0.000'
        hits = [h for h in yolo(bgr) if h[1] == 'apple']
        if hits:
            conf, _, box = max(hits)
            n_apple += 1
            apple_err.append(float(np.linalg.norm(np.array(box[:2]) - uv_apple)))
            row['apple_found'], row['apple_conf'] = 1, f'{conf:.3f}'
            cx, cy, bw, bh = box
            cv2.rectangle(bgr, (int(cx - bw / 2), int(cy - bh / 2)), (int(cx + bw / 2), int(cy + bh / 2)),
                          (0, 255, 0), 2)
        if k % 30 == 0:
            cv2.circle(bgr, tuple(int(v) for v in uv_apple), 4, (255, 0, 0), -1)
            cv2.circle(bgr, (int(float(row['tag_u'])), int(float(row['tag_v']))), 4, (255, 0, 255), -1)
            cv2.imwrite(os.path.join(a.out, 'frames', f'frame_{k:04d}.png'), bgr)
        if a.show:
            cv2.imshow('render', bgr)
            if cv2.waitKey(33) == 27:
                break
    write_csv(os.path.join(a.out, 'truth.csv'), rows)
    confs = [float(r['apple_conf']) for r in rows if r['apple_found'] == 1]
    line = (f'yolo ({yolo_name}): apple found {n_apple}/{len(rows)}, centre error median '
            f'{np.median(apple_err):.1f} px, confidence median {np.median(confs):.2f}' if apple_err
            else f'yolo ({yolo_name}): apple found 0/{len(rows)}')
    tail = f'frames every 1 s in {os.path.join(a.out, "frames")} (green box = YOLO, blue dot = true apple, magenta = true tag)'
    with open(os.path.join(a.out, 'summary.txt'), 'a') as f:
        f.write(line + '\n' + tail + '\n')
    print(line)
    print(tail)


if __name__ == '__main__':
    main()
