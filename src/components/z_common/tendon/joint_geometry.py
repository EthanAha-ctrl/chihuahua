"""实体双绳输出关节，尺寸单位 mm；加工配合与强度尚未定型。

build_joint() 返回可独立着色、爆炸和导出的 CadQuery 实体。
关节中心 (0,0,0)，下一级轴中心 (0,-65,0)，绳槽中心 z=0 / 6。
"""
import math
import cadquery as cq


def cylinder(r, h, p=(0, 0, 0), d=(0, 0, 1)):
    return cq.Solid.makeCylinder(r, h, cq.Vector(*p), cq.Vector(*d))


def box(x, y, z, p):
    return cq.Workplane("XY").box(x, y, z).val().translate(p)


def ring(ro, ri, h, z):
    return cylinder(ro, h, (0, 0, z)).cut(cylinder(ri, h + 2, (0, 0, z - 1)))


def bolt_z(d, length, x, y, z):
    """螺纹只保留外径，内六角头有实际凹槽。"""
    shaft = cylinder(d / 2, length, (x, y, z))
    head = cylinder(d * 0.9, d * 0.65, (x, y, z + length))
    socket = cq.Workplane("XY").polygon(6, d).extrude(d * 0.5).val().translate((x, y, z + length + d * 0.25))
    return shaft.fuse(head).cut(socket)


def bearing(z):
    # 686 外形 6×13×5；内外滚道与八颗球是可见实体，非供应商内部设计。
    race = cq.Solid.makeTorus(4.8, 0.85, (0, 0, z + 2.5))
    outer = ring(6.5, 5.35, 5, z).cut(race)
    inner = ring(4.25, 3, 5, z).cut(race)
    balls = [cq.Solid.makeSphere(0.8, cq.Vector(4.8 * math.cos(a), 4.8 * math.sin(a), z + 2.5)) for a in [i * math.pi / 4 for i in range(8)]]
    return cq.Compound.makeCompound([outer, inner] + balls)


def tube_between(a, b, r):
    v = cq.Vector(*b).sub(cq.Vector(*a))
    return cylinder(r, v.Length, a, v.toTuple())


def build_joint():
    out = []
    def add(key, name, shape, color, explode, stage, desc, source="自定义", **extra):
        if not shape.isValid():
            raise ValueError(f"Invalid joint solid: {key}")
        out.append(dict(key=key, name=name, shape=shape, color=color, explode=list(explode), group="joint", stage=stage, source=source, description=desc, **extra))

    metal = "#9eacb9"
    steel = "#677785"
    bright = "#cbd2d9"
    blue = "#376b98"
    gold = "#c6a16a"

    # 两个固定耳各自有真实轴承孔，通过位于轮缘以外的桥板连接。
    def ear(z):
        body = cylinder(9, 5, (0, 0, z)).fuse(box(10, 25, 5, (0, 13.5, z + 2.5)))
        if z < 0:
            body = body.fuse(box(6, 4, 5, (7, 12, z + 2.5)))
        body = body.cut(cylinder(6.5, 7, (0, 0, z - 1)))
        for x in (-3, 3):
            body = body.cut(cylinder(1.1, 7, (x, 23, z - 1)))
        if z < 0:
            body = body.cut(cylinder(.95, 7, (9.5,12,z-1)))
        return body
    add("j_rear_fork", "J01 后固定耳：轴承座", ear(-11), metal, (-7, 7, -24), 1, "轴承孔 Ø13；与上方桥板定位。轴承座配合公差待定。")
    add("j_front_fork", "J02 前固定耳：轴承座", ear(14), metal, (7, 7, 28), 8, "前后双耳夹住两个支撑轴承；前耳可拆。")
    bridge = box(10, 4, 30, (0, 28, 4)).fuse(box(10, 7, 2, (0, 24.5, -12)), box(10, 7, 2, (0, 24.5, 20)))
    for x in (-3, 3):
        bridge = bridge.cut(cylinder(1.1, 35, (x, 23, -13)))
    add("j_bridge", "J03 固定叉架桥板", bridge, metal, (0, 18, 0), 1, "桥板位于 y=26..30，避开直径48.8输出轮；此面接躯干/上一级连杆。")
    for n, z in enumerate((-11, 14)):
        add(f"j_bearing_{n}", f"J0{4+n} 686 支撑轴承", bearing(z), bright, (0, 0, -14 if n == 0 else 18), 2 if n == 0 else 8, "外形6×13×5 mm；滚道/滚珠为标准件可视包络，内部细节未绑定品牌。", "标准件包络")

    # 镂空输出轮：两条深圆槽，低速侧绳中心半径24。
    wheel = ring(24.4, 20.7, 12, -3).fuse(ring(8, 3.025, 12, -3))
    wheel = wheel.fuse(box(42, 4, 12, (0, 0, 3)), box(4, 42, 12, (0, 0, 3)), box(10, 16, 12, (0, -13, 3)))
    for angle in (-40, 40):
        wheel = wheel.fuse(box(44, 8.5, 12, (0, 0, 3)).rotate((0,0,0),(0,0,1),angle))
    wheel = wheel.cut(cylinder(3.025, 14, (0, 0, -4)))
    for z in (0, 6):
        wheel = wheel.cut(cq.Solid.makeTorus(24, .45, (0, 0, z)))
    wheel = wheel.cut(cylinder(.8, 20, (-10, 0, 3), (1, 0, 0)))
    for x, y in ((-3.3, -9), (3.3, -9), (0, -13.5)):
        wheel = wheel.cut(cylinder(1.05, 14, (x, y, -4)))
    for angle, z in ((40, 0), (-40, 6)):
        t = math.radians(angle)
        # 真实避让凹腔；滑槽底面落在凹腔地板，未埋在轮盘实体中。
        cavity_bottom = z - 2.2
        cavity_h = 9.2 - cavity_bottom
        pocket = box(11.4, 5.4, cavity_h, (0,0,cavity_bottom + cavity_h/2)).rotate((0,0,0),(0,0,1),angle-90).translate((17.8*math.cos(t),17.8*math.sin(t),0))
        wheel = wheel.cut(pocket)
        hole = cylinder(.52,5,(0,1.8,z+2.9),(0,1,0)).rotate((0,0,0),(0,0,1),angle-90).translate((17.8*math.cos(t),17.8*math.sin(t),0))
        wheel = wheel.cut(hole)
        for sx in (-3.2,3.2):
            mount_hole = cylinder(.6,3.7,(sx,0,z-3.7)).rotate((0,0,0),(0,0,1),angle-90).translate((17.8*math.cos(t),17.8*math.sin(t),0))
            wheel = wheel.cut(mount_hole)
    wheel = wheel.cut(cylinder(.95,4,(-9.5,-7,-4)))
    add("j_wheel", "J06 双槽镂空输出轮", wheel, blue, (0, 0, -3), 4, "绳中心节径48 mm；槽 z=0/6；实体外径48.8。两条槽保持分层，输出轮与腿端帽螺接。")

    shaft = cylinder(3, 36, (0, 0, -14)).cut(cylinder(.8, 8, (-4, 0, 3), (1, 0, 0)))
    for z in (-12.025,19.175):
        shaft = shaft.cut(ring(3.2,2.7,.85,z))
    add("j_axle", "J07 Ø6 旋转贯穿轴", shaft, steel, (0, 0, -35), 3, "穿过双轴承与输出轮；横向定位销将轴与轮连接，端部锁环限制轴向窜动。")
    add("j_cross_pin", "J08 Ø1.6 横向定位销", cylinder(.8, 16, (-8, 0, 3), (1, 0, 0)), bright, (24, 0, 0), 5, "示意横销传扭；孔配合、疲劳与销强度待验证。", "标准件包络")
    for n, z in enumerate((-12, 19.2)):
        collar = ring(4.4, 2.72, .8, z).cut(box(1.5, 5, 2, (0, 4, z + .4)))
        add(f"j_collar_{n}", f"J0{9+n} 轴用弹性挡圈", collar, steel, (0, 0, -29 if n == 0 else 33), 9, "挡圈嵌入Ø5.4轴槽；槽宽0.85、挡圈厚0.8仅作几何设计值，需选标准规格。", "标准件包络")
        washer_z = -11.2 if n == 0 else 19
        add(f"j_axial_shim_{n}",f"J0{9+n}.1 轴向调整垫片",ring(4.25,3.05,.2,washer_z),bright,(0,0,-20 if n==0 else 24),9,"厚0.2垫片填补挡圈与轴承内圈之间的装配间隙。","标准件包络")

    # 输出轮前侧独立金属端帽，5×7 木材进入真实槽腔。
    plate = box(10, 22, 2.2, (0, -12, 10.1))
    socket = box(7.4, 11.5, 9.4, (0, -21.25, 15.5)).cut(box(5.2, 10.5, 7.2, (0, -21.85, 15.5)))
    cap = plate.fuse(socket)
    cap = cap.cut(cylinder(3.2,4,(0,0,8.5)))
    for x, y in ((-3.3, -9), (3.3, -9), (0, -13.5)):
        cap = cap.cut(cylinder(1.1, 4, (x, y, 8.8)))
    cap = cap.cut(cylinder(1.05, 12, (-6, -22, 15.5), (1, 0, 0)))
    add("j_upper_cap", "J11 上端金属腿帽", cap, metal, (0, -8, 19), 6, "轮盘侧三颗M2连接螺钉；5.2×7.2木材插槽。胶接/横向销孔为待验证接头。")
    wood = box(5, 43, 7, (0, -38.7, 15.5))
    for y in (-22, -56):
        wood = wood.cut(cylinder(1.05, 7, (-3.5, y, 15.5), (1, 0, 0)))
    add("j_wood", "J12 木腿杆 5×7×43", wood, gold, (0, -28, 18), 6, "上下关节中心距65 mm；木材本体长43 mm，其余由端帽与插接重叠占用。顺纹轴向承载。")

    lower_ring = ring(8, 6.6, 2.2, 9).translate((0, -65, 0))
    lower_plate = box(10, 12, 2.2, (0, -58, 10.1))
    lower_socket = box(7.4, 9.5, 9.4, (0, -56.75, 15.5)).cut(box(5.2, 8.7, 7.2, (0, -56.15, 15.5)))
    lower = lower_ring.fuse(lower_plate, lower_socket).cut(cylinder(6.6, 4, (0, -65, 8.5)))
    lower = lower.cut(cylinder(1.05, 12, (-6, -56, 15.5), (1, 0, 0)))
    add("j_lower_cap", "J13 下端金属腿帽／下级轴承孔", lower, metal, (0, -40, 19), 6, "下一级关节接口中心 y=-65；Ø13.2孔仅为装配设计值，下一级总成未重复展开。")
    for n, (x, y) in enumerate(((-3.3, -9), (3.3, -9), (0, -13.5))):
        add(f"j_cap_bolt_{n}", f"J14.{n+1} M2 端帽螺钉", bolt_z(2, 5, x, y, 6.2), steel, (0, -8, 28), 7, "螺纹外形简化，实际啮合长度与螺纹孔需加工复核。", "标准件包络")
    for n, y in enumerate((-22, -56)):
        screw = bolt_z(2, 9, 0, 0, 0).rotate((0, 0, 0), (0, 1, 0), 90).translate((-5.3, y, 15.5))
        nut = cq.Workplane("YZ").polygon(6, 4).extrude(1.6).val().translate((-5.3, y, 15.5)).cut(cylinder(1, 2, (-5.5, y, 15.5), (1, 0, 0)))
        add(f"j_wood_pin_{n}", f"J15.{n+1} 木腿横向夹紧螺钉", screw, steel, (18, -15, 8), 7, "M2横向螺钉连接金属端帽和木材；木材孔局部应力未校核。", "标准件包络")
        add(f"j_wood_nut_{n}", f"J16.{n+1} 木腿夹紧螺母", nut, steel, (-18, -15, 8), 7, "M2六角螺母包络。", "标准件包络")

    # 两套独立的螺杆滑块端头。局部X为调整方向，局部Y为轮盘径向。
    # A/B各在各自绳槽轴向平面，两支调整器分置不同角位置。
    anchor_info = []
    for n, (ang, z, cord_color) in enumerate(((40, 0, "#2675ba"), (-40, 6, "#d17e29"))):
        theta = math.radians(ang)
        center = (17.8 * math.cos(theta), 17.8 * math.sin(theta), z)
        def place(s):
            return s.rotate((0,0,0), (0,0,1), ang - 90).translate(center)
        # Channel has lower floor and two end stops. A through adjuster moves a captive nut block.
        channel = box(11, 5, 1.1, (0, 0, -1.65)).fuse(box(1.3, 5, 3.6, (-4.85, 0, .7)), box(1.3, 5, 3.6, (4.85, 0, .7)))
        channel = channel.cut(cylinder(1.1, 14, (-7, 0, .7), (1, 0, 0)))
        for sx in (-3.2,3.2):
            channel = channel.cut(cylinder(.65,2,(sx,0,-2.5)))
        block = box(3.4, 4.4, 4.8, (0, 0, 1.3)).cut(cylinder(1, 5, (-2.5, 0, .7), (1, 0, 0))).cut(cylinder(.55, 6, (0, -3, 2.9), (0, 1, 0)))
        adjuster = bolt_z(2, 14, 0, 0, 0).rotate((0,0,0), (0,1,0), 90).translate((-8.5,0,.7))
        nut = cq.Workplane("YZ").polygon(6, 3.8).extrude(1.5).val().translate((-7,0,.7)).cut(cylinder(1,2,(-7.2,0,.7),(1,0,0)))
        add(f"j_anchor_frame_{n}", f"J17.{n+1} {'A' if n==0 else 'B'} 绳端预紧滑槽座", place(channel), blue, (18, 0, -16 if n==0 else 23), 7, "两颗M1.2固定于轮盘凹腔；螺杆移动绳端滑块约±2 mm，预紧后锁紧螺母。薄壁螺纹需校核。")
        add(f"j_anchor_block_{n}", f"J18.{n+1} 绳端夹持滑块", place(block), cord_color, (24, 0, -19 if n==0 else 27), 7, "滑块中Ø1.1横孔穿绳并夹持；最终绳结/夹绳牙口需要拉脱试验。")
        add(f"j_anchor_screw_{n}", f"J19.{n+1} M2 预紧调节螺杆", place(adjuster), steel, (30, 0, -21 if n==0 else 31), 7, "以细螺纹改变端头位置；需要测量实际绳张力设定预紧。", "标准件包络")
        add(f"j_anchor_nut_{n}", f"J20.{n+1} M2 调节锁紧螺母", place(nut), steel, (34, 0, -23 if n==0 else 34), 7, "预紧完成后锁紧，避免依靠大弹簧保持绳长。", "标准件包络")
        for k,sx in enumerate((-3.2,3.2)):
            add(f"j_anchor_mount_{n}_{k}",f"J20.{n+1}.{k+1} M1.2 滑槽座螺钉",place(bolt_z(1.2,2.5,sx,0,-3.6)),steel,(20,0,-20 if n==0 else 28),7,"固定滑槽座的微型螺钉；啮合长度需按材料与绳拉力复核。","标准件包络")
        # Short tail makes the hole and the rope anchoring location explicit.
        tail = tube_between((0, 2.2, 2.9), (0, 6, 2.9), .4)
        add(f"j_anchor_tail_{n}", f"J21.{n+1} {'A' if n==0 else 'B'} 绳端短尾", place(tail), cord_color, (18,0,-9 if n==0 else 15), 7, "绳端穿入夹块；长传动绳由上层装配连接。", "自定义")
        angle = math.radians(ang-90)
        anchor_info.append([center[0]-6*math.sin(angle), center[1]+6*math.cos(angle), z+2.9])

    # 可选并联扭簧：独立实体；无刚度/疲劳合格声明。
    spring_r, wire_r, pitch, h, z0 = 9.5, .33, .7, 1.75, -5.45
    helix = cq.Wire.makeHelix(pitch, h, spring_r)
    coil = cq.Workplane("XZ").center(spring_r, 0).circle(wire_r).sweep(cq.Workplane(obj=helix), isFrenet=True).val().translate((0,0,z0))
    a = (spring_r,0,z0)
    end_angle = h/pitch * 2*math.pi
    b = (spring_r*math.cos(end_angle), spring_r*math.sin(end_angle), z0+h)
    coil = coil.fuse(tube_between(a,(spring_r,11.1,z0),wire_r), tube_between(b,(-spring_r,-6.1,z0+h),wire_r))
    add("j_optional_spring", "J22 可选并联扭簧", coil, "#9566bd", (0,0,-14), 4, "线径0.66、平均圈径19、2.5圈仅作装配几何示例；刚度、预载与应力尚未选定。", optional=True)
    add("j_spring_fixed_pin", "J23 扭簧固定挂脚", cylinder(.9,3.8,(9.5,12,-8.4)), bright, (0,0,-15), 2, "与固定叉架连接的挂脚位置示例。", optional=True)
    add("j_spring_moving_pin", "J24 扭簧移动挂脚", cylinder(.9,3.5,(-9.5,-7,-4.4)), bright, (0,0,-7), 4, "与输出轮连接的挂脚位置示例；受力点需要结构复核。", optional=True)

    for n,(x,z) in enumerate(((-3,-14),(3,-14),(-3,16),(3,16))):
        screw = bolt_z(2,5,x,23,z) if z>0 else bolt_z(2,5,0,0,0).rotate((0,0,0),(1,0,0),180).translate((x,23,-8))
        add(f"j_fork_screw_{n}", f"J25.{n+1} M2 叉架连接螺钉", screw, steel, (0,10,-35 if z<0 else 38), 9, "连接桥板与前后耳；需另有止口/销承担定位。", "标准件包络")
    return out


if __name__ == "__main__":
    parts = build_joint()
    for p in parts:
        print(p["key"], p["shape"].isValid(), round(p["shape"].Volume(),3))
    print(f"{len(parts)} components; rope groove centers z=0,6; joint centers distance 65")
