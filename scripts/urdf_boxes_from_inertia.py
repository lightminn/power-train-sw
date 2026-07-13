"""메시 없는 CAD URDF → 관성텐서로 형상을 **역산**해 box 비주얼 URDF 를 만든다.

    python3 scripts/urdf_boxes_from_inertia.py <cad.urdf> <out.urdf>

────────────────────────────────────────────────────────────────────────
왜 이게 필요한가
────────────────────────────────────────────────────────────────────────
설계팀이 보낸 zip 에 **STL 메시가 하나도 없다**(URDF 가 `meshes/*.stl` 103종을 참조하는데
폴더 자체가 빠졌다). 그래서 RViz 에 아무것도 안 그려진다.

그런데 URDF 의 103개 링크에는 **질량과 관성텐서가 전부 들어있다.** 관성은 형상의 함수이므로
거꾸로 풀면 크기가 나온다. 균일 밀도 직육면체(a×b×c)라면

    Ixx = m(b²+c²)/12,  Iyy = m(a²+c²)/12,  Izz = m(a²+b²)/12

세 식을 a,b,c 에 대해 풀면

    a² = 6(Iyy + Izz − Ixx)/m      (나머지도 순환)

**검산**: `base_link_1_3030_500mm_2` (이름 = 30×30 프로파일 500 mm) 를 넣으면
**32.8 × 500.0 × 32.8 mm** 가 나온다. 이름이 말하는 실물과 맞다 (32.8 > 30 인 건 프로파일이
속이 비어 있어서 — 같은 질량이 바깥쪽에 분포하면 관성이 커진다).

────────────────────────────────────────────────────────────────────────
⚠️ 한계 — 이건 **근사**다
────────────────────────────────────────────────────────────────────────
· 실린더·L브래킷 같은 걸 **직육면체로 뭉갠다.** "관성이 같은 상자"지 실제 형상이 아니다.
· 속 빈 부품은 **실제보다 크게** 나온다 (위 32.8 vs 30 이 그 예다).
· 얇은 판은 a² 가 음수로 떨어질 수 있다 → 0 으로 클램프한다.
· **치수 계산·조향각에는 절대 쓰지 않는다.** 그건 이미 CAD 조인트 원점에서 뽑아 썼다
  (축거 875 / 윤거 705·879·585 mm). 이건 **RViz 에서 눈으로 보기 위한 껍데기**일 뿐이다.

메시가 오면 이 스크립트는 버린다.
"""
import sys
import xml.etree.ElementTree as ET

MIN_EDGE_M = 0.004          # 4 mm — 이보다 얇으면 RViz 에서 안 보인다
CYL_TOL = 0.03              # 주모멘트 2개가 3% 안이면 회전체로 본다


def cylinder_from_inertia(mass, moments):
    """관성텐서 → 등가 **원통** (radius, length, 축 index). 회전체가 아니면 None.

    회전체는 주모멘트 2개가 같다. 축을 x 라 하면 (반지름 R, 길이 L, 균일밀도):

        Ixx = mR²/2                 → R = √(2·Ixx/m)
        Iyy = Izz = m(3R² + L²)/12  → L = √(12·Iyy/m − 3R²)

    ★ 바퀴·타이어·베어링이 여기 걸린다. 상자로 뭉개면 로버가 각진 벽돌로 보이는데,
      원통으로 뽑으면 **바퀴가 바퀴처럼 보인다.**
    """
    for axis in range(3):                       # axis = 대칭축 후보
        other = [m for i, m in enumerate(moments) if i != axis]
        if max(other) <= 0:
            continue
        if abs(other[0] - other[1]) / max(other) > CYL_TOL:
            continue                            # 두 모멘트가 안 같다 → 회전체 아님

        r_sq = 2.0 * moments[axis] / mass
        l_sq = 12.0 * (sum(other) / 2) / mass - 3.0 * r_sq
        if r_sq <= 0 or l_sq <= 0:
            continue                            # 비물리 (얇은 링·원판)
        return r_sq ** 0.5, l_sq ** 0.5, axis
    return None


def box_from_inertia(mass, ixx, iyy, izz):
    """관성텐서 → 등가 직육면체 변 길이 (a, b, c) [m]. 비물리면 None."""
    if mass <= 0:
        return None
    # a² = 6(Iyy+Izz−Ixx)/m  — 삼각부등식이 깨지면 음수가 된다(얇은 판·비직육면체).
    squares = [
        6.0 * (iyy + izz - ixx) / mass,
        6.0 * (izz + ixx - iyy) / mass,
        6.0 * (ixx + iyy - izz) / mass,
    ]
    if all(s < 0 for s in squares):
        return None
    return tuple(max(s, 0.0) ** 0.5 for s in squares)


def _rot(rpy):
    import math
    R, P, Y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(R), math.sin(R), math.cos(P),
                             math.sin(P), math.cos(Y), math.sin(Y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _apply(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def add_rep103_base(root, wheel_radius=0.100):
    """CAD 루트 위에 **REP-103 `base_link`** 를 얹는다.

    이 CAD 는 **요(yaw) −90° 돌아간 Z-up** 이다 (Y-up 이 아니다 — 조인트 축을 base 프레임으로
    풀어보면 조향 14개가 전부 ±Z, 구동이 ±Y, 로커/보기가 ±Y 로 떨어져 확인된다).
    또 루트 링크가 `base_link_1_3030_240mm_1` — **차체 중심이 아니라 프로파일 한 토막**이라
    원점이 엉뚱한 곳에 있다.

    그래서 `base_link`(REP-103: x=전방, y=좌, z=위) 를 새 루트로 만들고 CAD 트리를 그 밑에
    fixed 로 매단다. 병진량은 **바퀴 위치에서 자동으로 역산**한다:
      · x, y = 축거·윤거의 기하 중심
      · z    = 바퀴 축 평면이 `wheel_radius` 위에 오도록  (= base_link 가 지면에 놓인다,
               기존 `jetin_rover.urdf.xacro` 와 같은 규약)
    """
    import math

    tf = {}                                  # link → (부모기준 누적 R, 위치)
    children = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0")).split()] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0")).split()] if o is not None else [0, 0, 0]
        children.setdefault(j.find("parent").get("link"), []).append(
            (j.find("child").get("link"), xyz, _rot(rpy)))

    named = {l.get("name") for l in root.findall("link")}
    child_names = {j.find("child").get("link") for j in root.findall("joint")}
    cad_root = next(iter(named - child_names))

    stack = [(cad_root, [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])]
    while stack:
        link, p, R = stack.pop()
        tf[link] = p
        for c, xyz, Rj in children.get(link, []):
            stack.append((c, [p[i] + _apply(R, xyz)[i] for i in range(3)], _mul(R, Rj)))

    # ros = M · cad   (yaw −90°)
    M = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
    tires = [_apply(M, p) for n, p in tf.items() if "tire" in n]
    if not tires:
        raise SystemExit("타이어 링크를 못 찾았다 — CAD 네이밍이 바뀌었나?")

    cx = (max(t[0] for t in tires) + min(t[0] for t in tires)) / 2
    cy = (max(t[1] for t in tires) + min(t[1] for t in tires)) / 2
    cz = sum(t[2] for t in tires) / len(tires)          # 바퀴 축 평면 높이

    # base_link → cad_root 병진: 위 중심이 (0, 0, wheel_radius) 로 가도록
    off = [-cx, -cy, wheel_radius - cz]

    base = ET.Element("link", name="base_link")
    root.insert(0, base)
    jt = ET.Element("joint", name="base_to_cad", type="fixed")
    ET.SubElement(jt, "parent", link="base_link")
    ET.SubElement(jt, "child", link=cad_root)
    ET.SubElement(jt, "origin",
                  xyz=f"{off[0]:.6f} {off[1]:.6f} {off[2]:.6f}",
                  rpy=f"0 0 {-math.pi / 2:.7f}")
    root.insert(1, jt)

    span_x = max(t[0] for t in tires) - min(t[0] for t in tires)
    print(f"  base_link 삽입 (yaw −90° · 지면 기준) — 축거 {span_x*1000:.1f} mm, "
          f"바퀴축 높이 {wheel_radius*1000:.0f} mm")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]

    tree = ET.parse(src)
    root = tree.getroot()

    cylinders = boxes = degenerate = skipped = 0

    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            skipped += 1
            continue

        mass_el, inertia_el = inertial.find("mass"), inertial.find("inertia")
        if mass_el is None or inertia_el is None:
            skipped += 1
            continue

        mass = float(mass_el.get("value", 0))
        moments = [float(inertia_el.get(k, 0)) for k in ("ixx", "iyy", "izz")]

        # ★ 상자·원통은 **질량중심(inertial origin)** 에 놓는다 — 관성이 거기서 정의됐으니까.
        #   visual origin 이 아니다(그건 메시 파일 기준 오프셋이라 메시가 없으면 무의미하다).
        io = inertial.find("origin")
        xyz = io.get("xyz", "0 0 0") if io is not None else "0 0 0"

        # 관성곱(ixy/iyz/ixz)은 무시한다 = 주축이 링크축과 나란하다고 본다.
        # 이 CAD 에선 관성곱이 대각성분보다 ~6 자릿수 작아서 실제로 그렇다.
        cyl = cylinder_from_inertia(mass, moments) if mass > 0 else None
        if cyl is not None:
            radius, length, axis = cyl
            geom_el = ("cylinder", {"radius": f"{max(radius, MIN_EDGE_M):.6f}",
                                    "length": f"{max(length, MIN_EDGE_M):.6f}"})
            # URDF 원통은 **z 축이 기본** → 대칭축이 x/y 면 돌려 세운다.
            rpy = {0: "0 1.5707963 0", 1: "-1.5707963 0 0", 2: "0 0 0"}[axis]
            cylinders += 1
        else:
            dims = box_from_inertia(mass, *moments)
            if dims is None:
                degenerate += 1
                continue
            dims = tuple(max(d, MIN_EDGE_M) for d in dims)
            geom_el = ("box", {"size": f"{dims[0]:.6f} {dims[1]:.6f} {dims[2]:.6f}"})
            rpy = "0 0 0"
            boxes += 1

        for tag in ("visual", "collision"):
            for old in link.findall(tag):
                link.remove(old)

        vis = ET.SubElement(link, "visual")
        ET.SubElement(vis, "origin", xyz=xyz, rpy=rpy)
        geom = ET.SubElement(vis, "geometry")
        ET.SubElement(geom, geom_el[0], **geom_el[1])
        mat = ET.SubElement(vis, "material", name="cad_grey")
        ET.SubElement(mat, "color", rgba="0.55 0.58 0.62 1.0")

    add_rep103_base(root)

    ET.indent(tree, space="    ")
    tree.write(dst, encoding="utf-8", xml_declaration=True)

    print(f"관성등가 형상 복원 → {dst}")
    print(f"  원통 {cylinders}개 (바퀴·베어링 등 회전체) · 상자 {boxes}개")
    if degenerate:
        print(f"  ⚠️ {degenerate}개는 관성이 비물리적(얇은 판 등) → 비주얼 없음")
    if skipped:
        print(f"  ⚠️ {skipped}개는 inertial 자체가 없음 → 비주얼 없음")
    print("\n⚠️ 이건 **눈으로 보기 위한 근사**다. 치수·조향각 계산엔 쓰지 않는다.")


if __name__ == "__main__":
    main()
