"중심 1픽셀만 depth 계산"이 어떻게 동작하는지, 비유 → 원리 → 실제 코드 순으로 풀어드리겠습니다.

1. 왜 "같은 픽셀의 depth를 그냥 읽으면" 안 되나
detector는 color 카메라 이미지에서 물체 bbox 중앙을 줍니다 (예: color 픽셀 (459, 161)).
그런데 우리가 필요한 건 그 지점까지의 거리(depth) 입니다.

문제는 color 카메라와 depth 카메라가 서로 ~15mm 떨어진 별개의 카메라라는 점입니다. 같은 물체라도 두 카메라에서 다른 픽셀 위치에 찍힙니다 (양 눈으로 손가락을 보면 좌·우 눈에서 위치가 다른 것과 같음 = 시차/parallax).

실제로 우리 검증에서 — 0.3m 거리 물체가 color (568,320) ↔ depth (500,300), 즉 68픽셀이나 차이났습니다. 그래서 color 픽셀 자리에서 depth를 그냥 읽으면 엉뚱한 곳의 거리를 읽습니다.

2. 닭과 달걀 문제
"color 픽셀이 depth 영상의 어느 픽셀이냐"를 알려면 → 그 점의 거리를 알아야 하는데, 거리는 지금 우리가 구하려는 것입니다. (서로가 서로를 필요로 함)

align_depth(full-frame 정렬)는 이 대응을 전체 픽셀에 대해 풀어줍니다. 하지만 우리는 딱 1점만 필요하니, 그 1점만 푸는 게 reverse projection입니다.

3. 핵심 아이디어: "답은 짧은 선 위에 있다" (epipolar line)
거리를 정확히는 몰라도 대략의 범위는 압니다 (현재 설정은 `target_3d.yaml`의 **0.19 m ~ 1.6 m**).

그 color 픽셀이 아주 가까운 물체라면 → depth 영상의 어떤 위치에 찍힘
아주 먼 물체라면 → depth 영상의 다른 위치에 찍힘
거리를 가까이→멀리 바꾸면, 대응하는 depth 픽셀이 직선(선분)을 그리며 이동합니다.
→ 즉 정답 depth 픽셀은 반드시 이 짧은 선분 위에 있습니다. 그 선분만 훑으면 됩니다.


depth 영상
   ┌─────────────────────────┐
   │        (먼 경우) ●        │
   │              ╲           │   ← 이 선분 위 어딘가가 정답
   │               ╲          │
   │        (가까운 경우) ●    │
   └─────────────────────────┘
4. 어떤 점이 정답인지 고르는 법 (되쏘기 검사)
선분 위의 각 후보 depth 픽셀마다 이렇게 확인합니다:

그 depth 픽셀의 실제 측정 거리 z를 읽는다.
"이 픽셀이 z 거리에 있다면, color 카메라에서는 어디에 보일까?"를 계산한다 (3D로 펼친 뒤 color로 되쏨).
그 결과가 원래 color 픽셀 (459,161)에 가장 가깝게 떨어지는 후보가 정답.
비유: depth 담당자가 선을 따라 걸으며 매 지점에서 "내가 잰 이 거리로 보면, color 친구 눈엔 (459,161)로 보이나?" 를 확인하고, 제일 잘 맞는 지점을 고르는 것.

5. 실제 코드 (3가지 도구 함수 + 탐색)
핵심 도구 3개 (pick_target_3d_node.py):


# 픽셀+거리 → 3D 점 (그 카메라 좌표계)
def deproject(fx,fy,cx,cy, u,v,z):
    return [(u-cx)*z/fx, (v-cy)*z/fy, z]

# 3D 점 → 픽셀
def project(fx,fy,cx,cy, p):
    return (p[0]*fx/p[2]+cx, p[1]*fy/p[2]+cy)

# 3D 점을 한 카메라 좌표계 → 다른 카메라 좌표계 (R,t = extrinsics)
#   R @ p + t
탐색 함수 color_pixel_to_depth_pixel:


# ① 선분의 양 끝: color 픽셀을 최소/최대 거리로 펼쳐 → depth 영상 픽셀로 변환
p_min = R_cd @ deproject(color, u_c,v_c, dmin) + t_cd   # color→depth 변환
u_min,v_min = project(depth, p_min)
p_max = R_cd @ deproject(color, u_c,v_c, dmax) + t_cd
u_max,v_max = project(depth, p_max)

# ② 선분을 한 픽셀씩 걸으며 되쏘기 검사
best=None; best_dist=∞
for (ui,vi) in 선분 위 픽셀들:
    z = depth_img[vi,ui] * 0.001        # 그 점의 실제 거리(m)
    if z 범위 밖/0이면 skip
    p  = R_dc @ deproject(depth, ui,vi, z) + t_dc   # depth점 → 3D → color좌표계
    u_chk,v_chk = project(color, p)                 # color로 되쏨
    dist = (u_chk-u_c)² + (v_chk-v_c)²              # 원래 color 픽셀과 거리
    if dist < best_dist: best=(ui,vi)               # 가장 가까운 후보 채택
return best
(이것은 librealsense의 rs2_project_color_pixel_to_depth_pixel과 동일한 알고리즘입니다.)

6. 노드 전체 흐름 (pick_target_callback)
intrinsics/extrinsics 받기 (하드코딩 X, 토픽에서 실시간):
depth/color camera_info → fx,fy,cx,cy
/camera/camera/extrinsics/depth_to_color → 회전 R_dc, 평행이동 t_dc (그 역이 R_cd,t_cd)
bbox 중앙 color 픽셀 (u_c,v_c) 로 위 color_pixel_to_depth_pixel 실행 → 대응 depth 픽셀 (ui,vi) 획득
그 픽셀 주변 9×9 patch의 median 으로 z 계산 (노이즈/구멍에 강건, patch_radius=4)
deproject(depth, ui,vi, z) → depth 좌표계에서 (x, y, z) → /pick_target_3d 발행 (frame = camera_depth_optical_frame)
이후 pick_target_base_node가 TF로 base_link 좌표로 변환
7. 왜 이게 싸고 좋은가
full-frame 정렬은 이 매칭을 ~40만 픽셀 전부에 대해 매 프레임 수행 → 코어 100% 점유.
reverse projection은 딱 1픽셀, 그것도 짧은 선분 몇 픽셀만 훑음 → 사실상 공짜.
그래서 realsense 코어 ~65% 회수 + 검출 ~13→17Hz, 그리고 z 정확도는 동일(자로 검증).
8. 주의했던 디테일
extrinsics 회전 행렬은 column-major 라 np.reshape(3,3, order='F')로 읽어야 함 (안 그러면 transpose돼서 좌표가 틀림).
color/depth 모두 왜곡계수 d=0(rectified)이라 핀홀 공식만으로 충분.
매칭 실패(선분에 유효 depth 없음)면 depth_valid=false로 안전하게 처리.
요약하면: "거리를 몰라도 정답은 짧은 선 위에 있다 → 그 선만 훑어 color로 되쏴보고 가장 잘 맞는 점을 고른다 → 거기서 거리를 읽어 3D로 푼다." 이 한 점짜리 매칭이 full-frame 정렬을 대체합니다.