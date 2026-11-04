# =============================================================================
# 02_render_synthetic.py — YCB mesh 6종 domain-randomized synthetic 렌더링
#
# 일반 python이 아니라 blenderproc로 실행한다 (데스크톱, venv 활성화 상태):
#   blenderproc run 02_render_synthetic.py -- \
#     --assets ~/capstone_training/assets --out ~/capstone_training/synth/batch_000 \
#     --scenes 40 --views 4 --seed 0
#
# 한 scene = 물체 pool에서 1~5개를 골라 가상 테이블에 물리 낙하로 배치
#            + distractor 0~4개 + 조명/배경/테이블 텍스처 랜덤화
#            → 카메라 각도 --views 개로 렌더링
# 출력: <out>/coco_data/ (images + coco_annotations.json, category_id = class+1)
#       → 02b_synth_to_yolo.py 가 YOLO format으로 변환
#
# 구조 제약 (중요): enable_segmentation_output()은 "호출 시점에 존재하는
# 객체"만 segmentation에 등록한다. 그래서 모든 객체(타겟 pool + distractor
# pool + 테이블)를 먼저 만들고, 그 다음 enable을 호출하고, scene마다는
# 새 객체를 만들지 않고 pool 객체를 재배치(활성/파킹)만 한다.
# (scene마다 duplicate/delete 하던 초기 구현은 segmap에 안 잡혀 label이
#  전부 비는 문제가 있었음 — 2026-07-07 smoke test로 확인)
#
# 배포 기하 (계획 D10): optical table(회색, 실측 RGB 반영) 위 물체, 카메라는
# 테이블 정중앙 상부 ~0.8 m에서 수직 하방(top-down) 고정. 카메라 샘플링은
# elevation 55~90°(90=수직), 거리 0.55~1.05 m로 이를 중심으로 randomize.
# =============================================================================
import blenderproc as bproc  # 반드시 첫 import
import argparse
import os
import random
import numpy as np

# classes.yaml(names)과 동일한 매핑 — 변경 금지 (blender 내장 python에 pyyaml이
# 없어 여기 하드코딩. classes.yaml 이 유일 기준이며 이 dict는 그 사본이다.)
YCB_TO_CLASS = {
    "013_apple": 0,
    "017_orange": 1,
    "011_banana": 2,
    "056_tennis_ball": 3,
    "006_mustard_bottle": 4,
}

POOL_PER_CLASS = 4      # class당 동시 등장 가능 최대 개수
DISTRACTOR_POOL = 8     # distractor 도형 pool 크기

# D435i 848x480 근사 intrinsics (HFOV ~69.4°) — 배포 카메라와 기하 일치
IMG_W, IMG_H = 848, 480
K = np.array([[612.0, 0.0, 424.0],
              [0.0, 612.0, 240.0],
              [0.0, 0.0, 1.0]])


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--assets", required=True, help="01 스크립트의 ASSETS_DIR")
    p.add_argument("--out", required=True, help="이 batch의 출력 디렉토리")
    p.add_argument("--scenes", type=int, default=40)
    p.add_argument("--views", type=int, default=4, help="scene당 카메라 각도 수")
    p.add_argument("--samples", type=int, default=32, help="렌더 샘플 수(품질/속도)")
    p.add_argument("--texture-dir", default=None,
                   help="배경/테이블 텍스처용 이미지 폴더 (기본: assets/coco/train2017 있으면 사용)")
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args()


def find_mesh(assets_dir, ycb_name):
    for sub in ("google_16k/textured.obj", "poisson/textured.obj", "tsdf/textured.obj"):
        path = os.path.join(assets_dir, "ycb_meshes", ycb_name, sub)
        if os.path.exists(path):
            return path
    raise FileNotFoundError(
        f"{ycb_name} mesh 없음 — 01_download_assets.sh --meshes 먼저 실행 필요")


def build_texture_pool(args):
    tex_dir = args.texture_dir
    if tex_dir is None:
        cand = os.path.join(args.assets, "coco", "train2017")
        tex_dir = cand if os.path.isdir(cand) else None
    if tex_dir is None:
        print("[synth] 텍스처 폴더 없음 → 단색 배경 사용 (coco 압축 해제 후 재실행하면 더 좋음)")
        return []
    pool = [os.path.join(tex_dir, f) for f in os.listdir(tex_dir)
            if f.lower().endswith((".jpg", ".jpeg", ".png"))]
    random.shuffle(pool)
    pool = pool[:5000]  # 파일 목록만 유지 (이미지는 사용 시 로드)
    print(f"[synth] 텍스처 pool: {len(pool)}장 ({tex_dir})")
    return pool


def set_object_material(obj, mat):
    """객체의 모든 material slot을 mat 하나로 교체 (slot 없으면 추가)."""
    try:
        if obj.get_materials():
            obj.replace_materials(mat)
        else:
            obj.new_material("tmp")  # slot 생성
            obj.replace_materials(mat)
    except Exception:
        pass


def random_solid_material(tag):
    mat = bproc.material.create(f"solid_{tag}_{random.randint(0, 10**9)}")
    mat.set_principled_shader_value(
        "Base Color", np.random.uniform(0.05, 0.95, 3).tolist() + [1.0])
    mat.set_principled_shader_value("Roughness", float(np.random.uniform(0.2, 0.95)))
    return mat


def scene_table_material(table, texture_pool, scene_idx):
    """scene마다 테이블 표면 교체. 배포 환경이 optical table(회색 금속판)이므로
    50%는 회색 금속 계열, 50%는 랜덤 사진 텍스처/단색(일반화용)."""
    mat = bproc.material.create(f"table_mat_{scene_idx}")
    use_gray = np.random.rand() < 0.5
    applied = False
    if not use_gray and texture_pool:
        try:
            import bpy
            img = bpy.data.images.load(random.choice(texture_pool))
            mat.set_principled_shader_value("Base Color", img)
            applied = True
        except Exception:
            applied = False
    if not applied:
        if use_gray:
            # 실측 optical table 색 (사용자 제공 2026-07-06):
            #   sRGB 어두움(110,110,108) / 평균(156,155,151) / 밝음(190,190,185)
            #   → linear gray 0.156 / 0.332 / 0.515, B채널 ≈ G의 0.94~0.97 (웜톤)
            g = float(np.random.uniform(0.15, 0.52))
            r_jit = float(np.random.uniform(0.98, 1.02))
            b_ratio = float(np.random.uniform(0.93, 0.98))
            color = [min(1.0, g * r_jit), g, min(1.0, g * b_ratio), 1.0]
        else:
            color = np.random.uniform(0.05, 0.95, 3).tolist() + [1.0]
        mat.set_principled_shader_value("Base Color", color)
    mat.set_principled_shader_value("Roughness", float(np.random.uniform(0.35, 0.9)))
    if use_gray:
        try:
            # anodized aluminum은 강한 거울 반사가 아니라 낮은~중간 metallic
            mat.set_principled_shader_value("Metallic", float(np.random.uniform(0.1, 0.6)))
        except Exception:
            pass
    set_object_material(table, mat)


def make_lights():
    lights = []
    for _ in range(np.random.randint(1, 4)):
        light = bproc.types.Light()
        ltype = random.choice(["POINT", "AREA", "SUN"])
        light.set_type(ltype)
        light.set_location([np.random.uniform(-2.0, 2.0),
                            np.random.uniform(-2.0, 2.0),
                            np.random.uniform(0.8, 2.5)])
        light.set_color(np.random.uniform(0.85, 1.0, 3).tolist())
        light.set_energy(float(np.random.uniform(1, 6)) if ltype == "SUN"
                         else float(np.random.uniform(30, 500)))
        lights.append(light)
    return lights


def park(obj, park_location):
    """객체를 화면 밖 파킹 위치로 치우고 렌더에서 숨긴다."""
    obj.set_location(park_location)
    obj.set_rotation_euler([0, 0, 0])
    obj.blender_obj.hide_render = True


def main():
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)

    bproc.init()
    bproc.camera.set_resolution(IMG_W, IMG_H)
    bproc.camera.set_intrinsics_from_K_matrix(K, IMG_W, IMG_H)
    bproc.renderer.set_max_amount_of_samples(args.samples)

    texture_pool = build_texture_pool(args)

    # ------------------------------------------------------------------
    # 1) 모든 객체를 먼저 생성 (segmentation 등록 전에 scene 완성이 필수)
    # ------------------------------------------------------------------
    # 테이블: 얇은 박스 (윗면 z=0). 평면보다 충돌 판정이 안정적
    table = bproc.object.create_primitive("CUBE")
    table.set_scale([1.5, 1.5, 0.02])
    table.set_location([0, 0, -0.02])
    table.enable_rigidbody(active=False, collision_shape="BOX")
    table.set_cp("category_id", 0)  # background

    # 타겟 pool: class당 POOL_PER_CLASS개 로드 (scene마다 일부만 활성화)
    target_pool = []  # (obj, ycb_name, park_location)
    park_x = 5.0
    for ycb_name, cls in YCB_TO_CLASS.items():
        mesh_path = find_mesh(args.assets, ycb_name)
        for i in range(POOL_PER_CLASS):
            obj = bproc.loader.load_obj(mesh_path)[0]
            obj.set_cp("category_id", cls + 1)  # coco는 1-base
            obj.set_name(f"{ycb_name}_{i}")
            park_loc = [park_x, 0.0, -5.0]
            park_x += 1.0
            park(obj, park_loc)
            target_pool.append((obj, ycb_name, park_loc))
        print(f"[synth] loaded {ycb_name} x{POOL_PER_CLASS}")

    # distractor pool (category_id=0 → 라벨 없음, "무시해야 할 물체" 역할)
    distractor_pool = []
    for i in range(DISTRACTOR_POOL):
        d = bproc.object.create_primitive(
            random.choice(["CUBE", "SPHERE", "CYLINDER", "CONE"]))
        d.set_cp("category_id", 0)
        d.set_name(f"distractor_{i}")
        park_loc = [park_x, 0.0, -5.0]
        park_x += 1.0
        park(d, park_loc)
        distractor_pool.append((d, park_loc))

    # ------------------------------------------------------------------
    # 2) 이제 segmentation 등록 (이 시점의 객체들이 annotation 대상이 된다)
    # ------------------------------------------------------------------
    bproc.renderer.enable_segmentation_output(
        map_by=["category_id", "instance", "name"],
        default_values={"category_id": 0})

    # ------------------------------------------------------------------
    # 3) scene 루프: pool 재배치만 (객체 생성/삭제 없음)
    # ------------------------------------------------------------------
    for scene_idx in range(args.scenes):
        bproc.utility.reset_keyframes()

        n_targets = np.random.randint(1, 6)
        active = [(obj, park_loc) for obj, _, park_loc
                  in random.sample(target_pool, n_targets)]
        n_dis = np.random.randint(0, 5)
        active_dis = random.sample(distractor_pool, n_dis)

        movable = []
        for obj, _ in active + active_dis:
            obj.blender_obj.hide_render = False
            movable.append(obj)
        # distractor는 scene마다 크기/색 재랜덤화
        for d, _ in active_dis:
            d.set_scale(np.random.uniform(0.02, 0.06, 3).tolist())
            set_object_material(d, random_solid_material("dis"))

        # 테이블 위 공중에서 랜덤 pose로 낙하
        def sample_pose(obj):
            obj.set_location(np.random.uniform([-0.25, -0.25, 0.08],
                                               [0.25, 0.25, 0.35]))
            obj.set_rotation_euler(bproc.sampler.uniformSO3())

        bproc.object.sample_poses(movable, sample_pose_func=sample_pose)
        for obj in movable:
            obj.enable_rigidbody(active=True)
        bproc.object.simulate_physics_and_fix_final_poses(
            min_simulation_time=1.5, max_simulation_time=3.5,
            check_object_interval=1)

        # 장면 랜덤화: 테이블 텍스처 / 조명 / 월드 배경
        scene_table_material(table, texture_pool, scene_idx)
        lights = make_lights()
        try:
            bproc.renderer.set_world_background(
                np.random.uniform(0.0, 1.0, 3).tolist(),
                strength=float(np.random.uniform(0.1, 1.0)))
        except Exception:
            pass

        # 카메라: 배포 기하(테이블 중앙 상부 ~0.8 m 수직 하방, D10)를 중심으로
        # randomize. look-at은 물체가 아니라 "테이블 중앙 부근"에 두어, 물체가
        # 화면 가장자리에 걸리는 배포 상황도 그대로 학습되게 한다.
        for _ in range(args.views):
            elev = np.deg2rad(np.random.uniform(55, 90))
            azim = np.random.uniform(0, 2 * np.pi)
            dist = np.random.uniform(0.55, 1.05)
            loc = np.array([dist * np.cos(elev) * np.cos(azim),
                            dist * np.cos(elev) * np.sin(azim),
                            dist * np.sin(elev)])
            look_at = np.array([np.random.uniform(-0.10, 0.10),
                                np.random.uniform(-0.10, 0.10), 0.0])
            rot = bproc.camera.rotation_from_forward_vec(
                look_at - loc, inplane_rot=np.random.uniform(-np.pi, np.pi))
            bproc.camera.add_camera_pose(
                bproc.math.build_transformation_mat(loc, rot))

        data = bproc.renderer.render()
        if scene_idx == 0:
            # 라벨 파이프라인 진단용: 첫 scene의 인스턴스별 속성 출력
            # (category_id 1~6인 항목들이 보여야 정상)
            try:
                print("[synth][debug] instance_attribute_maps[0]:",
                      data["instance_attribute_maps"][0])
            except Exception as exc:
                print("[synth][debug] attribute map 접근 실패:", exc)
        bproc.writer.write_coco_annotations(
            os.path.join(args.out, "coco_data"),
            instance_segmaps=data["instance_segmaps"],
            instance_attribute_maps=data["instance_attribute_maps"],
            colors=data["colors"],
            color_file_format="JPEG",
            append_to_existing_output=True)

        # 정리: rigidbody 해제 + 파킹 (다음 scene을 깨끗한 상태로)
        for obj, park_loc in active + active_dis:
            try:
                obj.disable_rigidbody()
            except Exception:
                pass
            park(obj, park_loc)
        for light in lights:
            light.delete()
        print(f"[synth] scene {scene_idx + 1}/{args.scenes} 완료")

    print(f"[synth] batch 완료 → {os.path.join(args.out, 'coco_data')}")


if __name__ == "__main__":
    main()
