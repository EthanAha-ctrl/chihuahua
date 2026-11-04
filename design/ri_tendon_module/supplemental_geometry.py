"""Detailed abduction packaging study, dimensions in millimetres.

Only custom support geometry is created here.  The caller instances complete
motor assemblies, retaining their original official STEP parts.
"""
import math
import cadquery as cq
from build_cad import C, cyl, ring, box, bolt, bearing_parts

# Rotate every RI30 about global Y by +90 degrees, then translate as below.
# The complete modules (including their caps) remain outside the output pulley.
RI30_ABDUCTION_POSITIONS = [(-60.0, -25.0, -14.4),
                           (-60.0,  25.0, -14.4),
                           (-60.0,   0.0,  28.9)]
RI30_ABDUCTION_ROTATION = ((0, 0, 0), (0, 1, 0), 90)
RI20_ABDUCTION_POSITION = (60.0, 0.0, 0.0)


def _cylx(r, h, x, y, z):
    return cq.Solid.makeCylinder(r, h, cq.Vector(x, y, z), cq.Vector(1, 0, 0))


def _cyl_dir(r, h, start, direction):
    return cq.Solid.makeCylinder(r, h, cq.Vector(*start), cq.Vector(*direction))


def _ringx(ro, ri, h, x, y, z):
    return _cylx(ro, h, x, y, z).cut(_cylx(ri, h+2, x-1, y, z))


def _local_yz_box(sx, nd, nt, loc, angle):
    return box(sx, nd, nt).rotate((0,0,0), (1,0,0), angle).translate(loc)


def build_abduction():
    parts = []
    def add(key, name, shape, color='al', stage=1, ex=(0,0,0), desc='', source='自定义', **kw):
        parts.append(dict(key='abd_'+key, name=name, shape=shape,
            color=C.get(color, color), group='abduction', stage=stage,
            explode=list(ex), source=source, description=desc, **kw))

    # The body-fixed back plate and bearing bridge carry leg loads directly.
    base = box(95, 66, 4, (35.5,0,-12))
    base = base.cut(cyl(5.0, 8, -16)).cut(cyl(12.2, 8, -16,60,0))
    for x,y in [(-7,-27),(-7,27),(76,-27),(76,27)]:
        base = base.cut(cyl(1.6, 8, -16,x,y))
    base = base.fuse(ring(10,6.52,5.5,-10))
    add('fixed_backplate', '外展01 固定躯干底座／后轴承座', base,
        ex=(0,0,-18), desc='固定于躯干；前后独立轴承承担摇架载荷。四角M3孔；电机编码板有避让孔。', shell=True)
    parts += bearing_parts('abd_后轴承', -9.5, 'abduction', 2, [0,0,-10])
    add('rear_retainer', '外展02 后轴承外圈压盖', ring(10,4.3,.7,-4.5),
        stage=2, ex=(0,0,-5), desc='仅限位轴承外圈，避开内圈及轴上隔套。')

    columns = [(0,28),(28,0),(0,-28)]
    # Support columns intentionally leave the two tangential tendon paths open.
    front = ring(10,6.52,5,30)
    for i,(x,y) in enumerate(columns):
        ang=math.degrees(math.atan2(y,x))
        arm=box(math.hypot(x,y),5,5,(math.hypot(x,y)/2,0,32.5)).rotate((0,0,0),(0,0,1),ang)
        boss=cyl(4.0,5,30,x,y).cut(cyl(1.15,7,29,x,y))
        front=front.fuse(arm).fuse(boss)
        col=ring(3.0,1.15,40,-10).translate((x,y,0))
        add('body_column_'+str(i),'外展03 固定轴承桥立柱 '+str(i+1),col,
            stage=1,ex=(x*.3,y*.3,0),desc='躯干固定柱；没有跨过A/B绳的切线通道。')
        add('body_bolt_'+str(i),'外展03 M2 轴承桥拉紧螺钉 '+str(i+1),
            bolt(2,46,-11,x,y),'steel',3,(0,0,15),desc='螺纹为名义包络，底部孔和防松仍需制造图细化。')
    front=front.cut(cyl(6.52,7,29))
    add('front_bridge','外展04 固定前轴承桥',front,stage=3,ex=(0,0,20),
        desc='三条真实支臂连接前轴承座；输出轮在桥内转动。',shell=True)
    parts += bearing_parts('abd_前轴承',30,'abduction',3,[0,0,12])
    add('front_retainer','外展05 前轴承外圈压盖',ring(10,4.3,.7,35),
        stage=3,ex=(0,0,24),desc='固定压盖与上方摇架轴套之间留轴向间隙。')

    # Two circumferential grooves have the requested actual rope centre planes.
    pulley=ring(18.4,3.01,11,17.5)
    for z in [19.7,25.7]:
        groove=cq.Solid.makeTorus(18,.45,cq.Vector(0,0,z),cq.Vector(0,0,1))
        pulley=pulley.cut(groove)
    for a in [30,150,270]:
        x,y=10.8*math.cos(math.radians(a)),10.8*math.sin(math.radians(a))
        pulley=pulley.cut(cyl(3.9,13,16.5,x,y))
    cross_bore=_cylx(.79,18,-9,0,23)
    pulley=pulley.cut(cross_bore)
    for i,(x,y,z) in enumerate([(-15,0,19.7),(-15,0,25.7)]):
        # Axial eyelet: cord termination / later sliding-clamp detail, accessible
        # from the outer face.  Rope routing is authored by the parent assembly.
        pulley=pulley.cut(cyl(.6,2.6,z-1.3,x,y))
    add('pulley','外展06 双槽输出轮 · 绳中心Ø36',pulley,'steel',3,(0,0,7),
        desc='R18两道真实圆周槽，中心面Z=19.7/25.7；减重孔和绳端孔可见。r3输入给6:1几何比；端夹与预紧滑块另行配套。')
    # Independent captive M2 adjusters and visible cord clamp plates.
    for i,(z,sgn) in enumerate([(19.7,-1),(25.7,1)]):
        pkey='A' if i==0 else 'B'
        lug=box(5.5,3,3.8,(-15,sgn*7.5,z))
        lug=lug.cut(_cyl_dir(1.1,5,(-14.5,sgn*5.5,z-.5),(0,sgn,0)))
        pulley=pulley.fuse(lug).cut(box(5.2,12,4.4,(-15,0,z)))
        slider=box(4.4,3.8,1.6,(-15,0,z-.4))
        channel=_cyl_dir(.43,5,(-16,-2.5,z+.3),(0,1,0))
        slider=slider.cut(channel)
        plate=box(4.4,3.8,.7,(-15,0,z+.85)).cut(channel)
        sx,sy=-14.0,-sgn*.8
        slider=slider.cut(cyl(.5,3,z-1.5,sx,sy))
        plate=plate.cut(cyl(.65,3,z-.5,sx,sy))
        add('anchor_'+pkey,'外展06 '+pkey+' 绳端调紧滑块',slider,
            stage=5,ex=(-5,sgn*5,0),desc='真实滑槽内的绳端夹座；M2切向螺钉拉动滑块，绳端通道位于X=-16。')
        add('anchor_plate_'+pkey,'外展06 '+pkey+' 绳端压板',plate,
            stage=5,ex=(-5,sgn*5,5),desc='压板夹住0.8mm绳，夹持力及绳损伤需拉脱试验。')
        add('anchor_plate_screw_'+pkey,'外展06 '+pkey+' 压板螺钉',bolt(1.2,2.0,z-.8,sx,sy),'steel',5,(-5,sgn*5,8),
            desc='M1.2名义螺钉；小螺纹和夹持尺寸仍属初稿。')
        rot=-90 if sgn==1 else 90
        adjust=bolt(2,8.6,0,0,0).rotate((0,0,0),(1,0,0),rot).translate((-14.5,sgn*.8,z-.5))
        add('adjust_'+pkey,'外展06 '+pkey+' M2切向调紧螺钉',adjust,'steel',5,(-5,sgn*9,0),
            desc='外侧螺钉头支承固定轮耳，螺纹拉动绳端滑块；有效调紧行程需配合绳长设定。')
        jam=cq.Workplane('XY').polygon(6,4).extrude(1.5).val().cut(cyl(.9,3,-.5))
        jam=jam.rotate((0,0,0),(1,0,0),rot).translate((-14.5,sgn*4.5,z-.5))
        add('adjust_lock_'+pkey,'外展06 '+pkey+' 调紧锁母',jam,'steel',5,(-5,sgn*9,0),
            desc='调好预紧后锁母顶住固定轮耳；运行中没有弹簧张紧器。')
        washer=ring(2.1,1.05,.4).rotate((0,0,0),(1,0,0),rot).translate((-14.5,sgn*9,z-.5))
        add('adjust_washer_'+pkey,'外展06 '+pkey+' 调紧螺钉垫圈',washer,'steel',5,(-5,sgn*9,0))
    for item in parts:
        if item['key']=='abd_pulley':
            item['shape']=pulley
            item['description']='R18双槽，绳中心Z=19.7/25.7；两根绳分别固定在切向滑块，M2螺钉调紧后锁母锁定。6:1几何比；尚待行程/强度验证。'
    shaft=cyl(3,58,-13).cut(cross_bore)
    add('shaft','外展07 独立承载输出轴',shaft,'steel',2,(0,0,8),
        desc='Ø6输出轴穿过双轴承；载荷进入躯干支架。轮与轴由横销传矩，末端螺纹尚未细画。')
    add('torque_pin','外展08 输出轮传矩横销',_cylx(.75,14,-7,0,23),'steel',3,(-12,0,0),
        desc='Ø1.5横销的几何占位，压配/防脱及冲击承载尚需选型。')
    spacers=ring(4,3.02,22,-4.5).fuse(ring(4,3.02,1.5,28.5)).fuse(ring(4,3.02,2,35)).fuse(ring(4,3.02,1,-10.5))
    add('shaft_spacers','外展09 内圈定位隔套',spacers,stage=2,ex=(0,0,6),
        desc='隔套接触轴承内圈；轴承外圈由固定座独立定位。')
    for i,z in enumerate([-12,43]):
        n=cq.Workplane('XY').polygon(6,8).extrude(1.5).val().translate((0,0,z)).cut(cyl(2.6,3,z-.5))
        add('locknut_'+str(i),'外展10 输出轴薄锁母',n,'steel',4,(0,0,-8 if i==0 else 14),
            desc='轴向锁紧示意包络，热伸长、游隙与防松待校核。')

    # Fixed RI20 mount: a split collar around its actual custom stator housing.
    fixed_ring=ring(19,16.05,6,2).translate((60,0,0))
    lo=fixed_ring.cut(box(50,30,10,(60,14.85,5)))
    hi=fixed_ring.cut(box(50,30,10,(60,-14.85,5)))
    for x in [40.5,79.5]:
        lo=lo.fuse(box(6,4,6,(x,-2.15,5)))
        hi=hi.fuse(box(6,4,6,(x,2.15,5)))
        hole=_cyl_dir(1.1,12,(x,-6,5),(0,1,0))
        lo=lo.cut(hole);hi=hi.cut(hole)
        leg=box(6,7,12,(x,-1.5,-4))
        lo=lo.fuse(leg)
        screw=bolt(2,9.15,0,0,0).rotate((0,0,0),(1,0,0),-90).translate((x,-5,5))
        add('ri20_clamp_screw_'+str(x),'外展11 RI20 固定抱箍螺钉',screw,'steel',2,(0,6,0),
            desc='螺钉夹紧定子外壳抱箍；外展电机本体固定在躯干。')
    add('ri20_fixed_saddle','外展11 RI20 固定抱箍下座',lo,stage=1,ex=(8,-8,0),
        desc='夹持RI20定制铝壳Ø32；电机轴线Z，中心X=60。')
    add('ri20_fixed_cap','外展11 RI20 固定抱箍上盖',hi,stage=2,ex=(8,12,0),
        desc='拆下上盖可释放RI20；实物夹紧力和壳体变形尚需校核。',shell=True)

    # RI30 carrier: three real split collars, with narrow middle webs.  Caps
    # face the outside of the triangle so their screws don't hit adjacent units.
    yz=[(p[1],p[2]) for p in RI30_ABDUCTION_POSITIONS]
    cradle=cq.Workplane('YZ',origin=(-57,0,0)).polyline(yz).close().extrude(6).val()
    for yc,zc in yz:
        cradle=cradle.cut(_cylx(22.6,8,-58,yc,zc))
    for i,(_,yc,zc) in enumerate(RI30_ABDUCTION_POSITIONS):
        theta=math.atan2(zc,yc);dy,dz=math.cos(theta),math.sin(theta)
        ty,tz=-dz,dy;deg=math.degrees(theta)
        collar=_ringx(24.7,22.6,6,-57,yc,zc)
        outside=_local_yz_box(10,100,100,(-54,yc+dy*67.55,zc+dz*67.55),deg)
        cutspace=_local_yz_box(10,100,100,(-54,yc+dy*67.25,zc+dz*67.25),deg)
        cap=collar.intersect(outside)
        saddle=collar.cut(cutspace)
        for j,t in enumerate([-20.0,20.0]):
            for which,d in [('base',15.45),('cap',19.35)]:
                loc=(-54,yc+dy*d+ty*t,zc+dz*d+tz*t)
                ear=_local_yz_box(6,3.3,8.0,loc,deg)
                if which=='base': saddle=saddle.fuse(ear)
                else: cap=cap.fuse(ear)
            start=(-54,yc+dy*13.0+ty*t,zc+dz*13.0+tz*t)
            hole=_cyl_dir(.88,11.5,start,(0,dy,dz))
            saddle=saddle.cut(hole);cap=cap.cut(hole)
            screw=bolt(1.6,8,0,0,0).rotate((0,0,0),(1,0,0),math.degrees(math.atan2(-dy,dz))).translate(start)
            add('ri30_clamp_bolt_'+str(i)+'_'+str(j),'外展12 RI30 '+str(i+1)+' 抱箍螺钉',screw,'steel',5,
                (0,dy*10,dz*10),desc='M1.6夹紧螺钉；名义几何，螺纹及紧固载荷待验证。')
        saddle=saddle.cut(_cylx(22.6,8,-58,yc,zc))
        cap=cap.cut(_cylx(22.6,8,-58,yc,zc))
        cradle=cradle.fuse(saddle)
        add('ri30_cap_'+str(i),'外展12 RI30 '+str(i+1)+' 可拆抱箍弧盖',cap,
            stage=5,ex=(0,dy*12,dz*12),desc='内径45.2、外径49.4 mm；弧盖朝三角组外侧，避免压住相邻电机。',shell=True)

    # The carrier wraps behind the three modules and around their left side;
    # no structural arm passes through an existing motor or pulley.
    arm=box(26,5,5,(-64,0,0))
    arm=arm.fuse(box(5,53,5,(-77,-26.5,0)))
    arm=arm.fuse(box(77,5,5,(-38.5,-53,0)))
    arm=arm.fuse(box(5,5,40,(0,-53,20)))
    arm=arm.fuse(box(5,53,6,(0,-26.5,40)))
    arm=arm.fuse(ring(8,3.02,6,37))
    cradle=cradle.fuse(arm)
    add('rotating_cradle','外展13 三电机随动摇架／真实抱箍底座',cradle,'#9aafbf',4,(-12,-12,12),
        desc='整组绕Z转动；三RI30轴线X，中心(-60,-25,-14.4)、(-60,25,-14.4)、(-60,0,28.9)mm。后绕支臂避开电机和绳轮，包络较大；未通过整机紧凑与质量预算。')
    # Positive torque key into carrier rather than an unmodelled friction joint.
    key=_cylx(.75,16,-8,0,40)
    parts[-1]['shape']=parts[-1]['shape'].cut(_cylx(.79,18,-9,0,40))
    # Match the shaft to the second pin hole.
    for p in parts:
        if p['key']=='abd_shaft':
            p['shape']=p['shape'].cut(_cylx(.79,18,-9,0,40))
    add('cradle_pin','外展14 摇架传矩横销',key,'steel',4,(-12,0,12),
        desc='实体横销连接轴与摇架；销径、配合与防脱方案为待验证初稿。')

    for p in parts:
        p['motion']='外展随动' if any(t in p['key'] for t in ['pulley','shaft','torque_pin','spacers','locknut','ri30_','rotating_cradle','cradle_pin','anchor_','adjust_']) else '躯干固定'
        if p['key'].endswith('_inner'): p['motion']='外展随动'
        if not p['shape'].isValid():
            raise RuntimeError('Invalid abduction CAD: '+p['key'])
        if p['shape'].Volume()<=0:
            raise RuntimeError('Empty abduction CAD: '+p['key'])
    return parts
