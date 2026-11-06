"""RI tendon integration study. Units mm. Vendor STEP is imported unchanged.
Custom parts describe a reviewable arrangement, not released production drawings.
"""
from pathlib import Path
import math, json, gzip, base64
import cadquery as cq
import trimesh
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / 'artifacts/ri_tendon_module'
OUT.mkdir(parents=True, exist_ok=True)
C = {'al':'#aebbc8','steel':'#64758a','stator':'#b77d55','rotor':'#39485a',
     'blue':'#2382d0','orange':'#ed8b30','pcb':'#3c816c','wood':'#d5aa6c','spring':'#9b66c6'}

def cyl(r, h, z=0, x=0, y=0):
    return cq.Solid.makeCylinder(r,h,cq.Vector(x,y,z))

def ring(ro, ri, h, z=0):
    return cyl(ro,h,z).cut(cyl(ri,h+2,z-1))

def box(x,y,z,loc=(0,0,0)):
    return cq.Workplane('XY').box(x,y,z).val().translate(loc)

def tube(points, radius):
    pts=[cq.Vector(*p) for p in points]
    path=cq.Wire.assembleEdges([cq.Edge.makeSpline(pts)])
    plane=cq.Plane(origin=pts[0], normal=(pts[1]-pts[0]))
    return cq.Workplane(plane).circle(radius).sweep(path,isFrenet=True).val()

def bolt(d,length,z,x,y):
    shaft=cyl(d/2,length,z,x,y)
    head=cyl(d*.9,d*.7,z+length,x,y)
    recess=cq.Workplane('XY').polygon(6,d).extrude(d*.5).val().translate((x,y,z+length+d*.25))
    return shaft.fuse(head.cut(recess))

def bearing_parts(prefix,z,group,stage,ex):
    # 686ZZ nominal envelope 6 x 13 x 5; races/shields represented separately.
    result=[]
    for k,name,shape in [
      ('outer','外圈',ring(6.5,5.3,5,z)),('inner','内圈',ring(4.1,3,5,z)),
      ('shields','双侧防尘盖',ring(5.3,4.1,.25,z+.25).fuse(ring(5.3,4.1,.25,z+4.5)))]:
        result.append(dict(key=prefix+'_'+k,name=prefix+' 686ZZ '+name,shape=shape,
          color=C['steel'] if k!='shields' else '#d3dce2',group=group,stage=stage,explode=ex,
          source='标准件包络',description='686ZZ，6×13×5 mm；内外圈分开显示，未细画滚珠。'))
    return result

def build_motor(model):
    small=model=='ri20'; R=12.5 if small else 19; H=14.5 if small else 14
    offset=8 if small else 3; bore=5.8 if small else 8.5
    rotorz=2 if small else 3; rotorh=10 if small else 8
    outer=R+3.5; bc=R+1.8; group=model; parts=[]
    def add(key,name,shape,color='al',stage=1,ex=(0,0,0),source='自定义',desc='',**kw):
        parts.append(dict(key=model+'_'+key,name=name,shape=shape,color=C.get(color,color),group=group,
          stage=stage,explode=list(ex),source=source,description=desc,**kw))
    def carrier(z,h):
        s=ring(outer,6.52,h,z)
        for a in [90,210,330]:
            x,y=(R*.66*math.cos(math.radians(a)),R*.66*math.sin(math.radians(a)))
            s=s.cut(cyl(max(2.2,R*.24),h+2,z-1,x,y))
        for a in [30,150,260]:
            x,y=bc*math.cos(math.radians(a)),bc*math.sin(math.radians(a))
            s=s.cut(cyl(1.05,h+2,z-1,x,y))
        return s
    # Sleeve has a wire egress notch and locating registers for the bearing covers.
    sleeve=ring(outer,R+.05,H+.5,-.25)
    sleeve=sleeve.cut(box(7,12,H+2,(0,-R-2,H/2)))
    for a in [30,150,260]:
        x,y=bc*math.cos(math.radians(a)),bc*math.sin(math.radians(a))
        sleeve=sleeve.cut(cyl(1.05,H+3,-1,x,y))
        sleeve=sleeve.cut(cyl(1.61,1.1,H-.8,x,y))
    add('housing','01 铝制定子散热座',sleeve,stage=1,ex=(0,-15,0),desc=f'定子座内径约 {2*R+.1:g} mm；胶层与同轴公差待厂家确认。',shell=True)
    rear=carrier(-6,5.5).fuse(ring(R+.04,R-1,.5,-.5)).fuse(ring(7.2,5.3,.5,-6))
    for x in [-9,9]: rear=rear.cut(cyl(1.05,4,-7,x,0))
    add('rear_cover','02 后轴承端盖／定位止口',rear,stage=1,ex=(0,0,-28),desc='止口定位；螺栓只夹紧。轻量孔避开轴承座。',shell=True)
    parts+=bearing_parts(model+'_后',-5.5,group,2,[0,0,-18])
    vendor=cq.importers.importStep(str(ROOT/'vendor/components/motor'/model/(model.upper()+' 3D Drawing.STEP'))).solids().vals()
    for i,s in enumerate(vendor):
        rotor=(i==4 if small else i==1)
        lead=small and i in [1,2,3]
        add('official_'+str(i),('04 '+model.upper()+' 原厂转子环') if rotor else ('03 原厂引线' if lead else '03 '+model.upper()+' 原厂定子'),
            s.translate((0,0,offset)),color='rotor' if rotor else ('steel' if lead else 'stator'),
            stage=3 if rotor else 2,ex=(0,0,-7 if rotor else -15),source='官方 STEP',
            desc='从厂家 STEP 原尺寸导入；厂家提供的是简化包络，未给出线圈／磁片内部细节。')
    hub=ring(bore-.05,3.01,rotorh,rotorz)
    # Three drilled lightening bores leave continuous inner and outer bonding surfaces.
    if not small:
        for a in [0,120,240]:
            hub=hub.cut(cyl(1.2,rotorh+2,rotorz-1,5.6*math.cos(math.radians(a)),5.6*math.sin(math.radians(a))))
    add('hub','05 转子连接毂',hub,stage=3,ex=(0,0,3),desc=f'外径约 {2*bore-.1:g} mm 胶合转子；中心与轴固定，胶接配合尚待实物验证。')
    shaft=cyl(3,47,-9).cut(cyl(2.03,.7,-9.05))
    shaft=shaft.fuse(cyl(3.4,14.7,15.3))
    # Real helical cut grooves. Neutral cable tangent planes: 19.7 and 25.7.
    angles=[math.acos(15/60),-math.acos(15/60)] if small else [math.asin(21/90),math.pi-math.asin(21/90)]
    for z0,endz,hand,phase in [(16.7,19.7,1,angles[0]),(24.2,25.7,-1,angles[1])]:
        pts=[]
        for i in range(181):
            z=z0+4.5*i/180
            a=phase+hand*2*math.pi*(z-endz)
            pts.append((3*math.cos(a),3*math.sin(a),z))
        shaft=shaft.cut(tube(pts,.42))
    for z in [15.0,22.0,29.0]:
        flange=cyl(6,.8,z)
        # Rounded edge-through anchor holes; separate clamps prevent the cord slipping.
        flange=flange.cut(cyl(.75,2,z-.5,4.9,0))
        shaft=shaft.fuse(flange)
    add('spindle','06 一体阶梯钢轴＋双螺旋绕线器',shaft,'steel',3,(0,0,12),desc='轴承段 Ø6；绳中心节径 Ø6；槽底约 Ø5.16。两槽反向，节距1 mm，每槽4.5圈；全行程排绳待验证。')
    spacers=ring(3.8,3.02,rotorz+.5,-.5).fuse(ring(3.8,3.02,15-rotorz-rotorh,rotorz+rotorh)).fuse(ring(3.8,3.02,2,30)).fuse(ring(3.8,3.02,1.5,-7))
    add('spacers','07 轴承内圈隔套',spacers,stage=3,ex=(0,0,16),desc='可拆隔套形成轴肩；装配时先套转子连接毂，再装隔套和两端轴承。')
    for i,z in enumerate([15,29]):
        clamp=box(2,2,1,(4.9,0,z+1.3)).cut(cyl(.8,3,z,4.9,0))
        add('anchor_'+str(i),'08 '+('A' if i==0 else 'B')+' 绳端压块',clamp,stage=5,ex=(9 if i==0 else -9,0,20),desc='穿绳孔＋压绳块；至少保留1.5圈死圈，固定端需进行拉脱试验。')
        add('anchor_screw_'+str(i),'08 绳端压块螺钉',bolt(1.4,1.5,z+0.4,4.9,0),'steel',5,(9 if i==0 else -9,0,24),desc='螺纹仅以名义包络显示。')
    front=carrier(32,5).fuse(ring(7.2,5.3,.5,37))
    for a in [30,150,260]:
        x,y=bc*math.cos(math.radians(a)),bc*math.sin(math.radians(a))
        front=front.cut(cyl(1.61,1.1,31.95,x,y))
    add('front_cover','09 前轴承桥盖',front,stage=4,ex=(0,0,34),desc='与后端盖共同支撑绕线轴，绳横向载荷经两轴承传入外壳。',shell=True)
    parts+=bearing_parts(model+'_前',32,group,4,[0,0,29])
    for i,a in enumerate([30,150,260]):
        x,y=bc*math.cos(math.radians(a)),bc*math.sin(math.radians(a))
        col=ring(2,1.05,32-H-.25,H+.25).fuse(ring(1.6,1.05,1,H-.75)).fuse(ring(1.6,1.05,1,32)).translate((x,y,0))
        add('column_'+str(i),'10 定距柱 '+str(i+1),col,stage=4,ex=(x*.45,y*.45,17),desc='前后盖共用精确定位；柱体留出绳路窗口。')
        add('tiebolt_'+str(i),'11 M2 拉紧螺钉 '+str(i+1),bolt(2,42,-5,x,y),'steel',4,(x*.5,y*.5,40),desc='螺纹未建模；穿过定子座外侧，不进入电机铁芯。')
    for i,z in enumerate([-8.5,37]):
        nut=cq.Workplane('XY').polygon(6,8).extrude(1.5).val().translate((0,0,z)).cut(cyl(3,3,z-.5))
        add('locknut_'+str(i),'12 轴端薄锁母',nut,'steel',4,(0,0,-35 if i==0 else 44),desc='螺纹和防松待细化；前轴承外圈需容许热伸长。')
    add('magnet','13 轴端编码磁体',cyl(2,1.5,-10),'rotor',6,(0,0,-45),desc='Ø4×1.5 几何占位；磁体、传感器型号与气隙需匹配。')
    pcb=ring(11,0.8,1,-13)
    for a in [0,180]:
        pcb=pcb.cut(cyl(1.1,3,-14,9*math.cos(math.radians(a)),0))
    add('encoder','14 编码器电路板',pcb,'pcb',6,(0,0,-52),source='待选型包络',desc='电机换相／轴角测量；不等同关节输出端编码器。')
    add('sensor','14 磁编码器芯片',box(3,3,.7,(0,0,-11.65)),'rotor',6,(0,0,-52),source='待选型包络',desc='芯片型号及磁场影响未验证。')
    for i,x in enumerate([-9,9]):
        add('pcb_standoff_'+str(i),'15 编码板隔柱',ring(1.8,1.05,6,-12).translate((x,0,0)),stage=6,ex=(0,0,-40),desc='编码板绝缘安装。')
        screw=bolt(2,7,0,x,0).rotate((x,0,0),(x+1,0,0),180).translate((0,0,-6))
        add('pcb_screw_'+str(i),'15 编码板固定螺钉',screw,'steel',6,(0,0,-55),desc='名义包络；后盖紧固孔需终版配合。')
    for i,phi in enumerate(angles):
        endz=19.7+6*i
        pts=[]
        for j in range(181):
            t=j/180;angle=phi+2*math.pi*3*(t-1)
            z=endz+(-3*(1-t) if i==0 else 3*(1-t))
            pts.append((3*math.cos(angle),3*math.sin(angle),z))
        delta=(-60+15*math.cos(phi),15*math.sin(phi)) if small else (21*math.cos(phi),-90+21*math.sin(phi))
        norm=math.hypot(*delta); start=pts[-1]
        for j in range(1,7):pts.append((start[0]+delta[0]/norm*j*2,start[1]+delta[1]/norm*j*2,endz))
        add('cord_'+str(i),'16 '+('A' if i==0 else 'B')+' 绳／中位三圈',tube(pts,.4),'blue' if i==0 else 'orange',5,(-8 if i==0 else 8,-10,0),desc='0.8 mm 绳；中位每槽3圈，完整关节视图显示后续绳路。',localCord=True,display_path=pts,display_radius=.4)
    return parts

def serialize(parts):
    result=[]
    for p in parts:
        shape=p['shape']
        if not shape.isValid():
            raise RuntimeError('Invalid CAD shape: '+p['key'])
        if 'display_path' in p:
            pts=np.array(p['display_path']);verts=[];faces=[];sides=10
            for i,pt in enumerate(pts):
                direction=pts[min(i+1,len(pts)-1)]-pts[max(i-1,0)];direction/=np.linalg.norm(direction)
                seed=np.array([0.,0.,1.]) if abs(direction[2])<.9 else np.array([1.,0.,0.])
                u=np.cross(direction,seed);u/=np.linalg.norm(u);v=np.cross(direction,u)
                for j in range(sides):
                    a=2*math.pi*j/sides;verts.append(pt+p['display_radius']*(u*math.cos(a)+v*math.sin(a)))
                if i:
                    for j in range(sides):
                        a=(i-1)*sides+j;b=(i-1)*sides+(j+1)%sides;c=i*sides+j;d=i*sides+(j+1)%sides
                        faces.extend([[a,b,c],[b,d,c]])
            mesh=trimesh.Trimesh(vertices=verts,faces=faces,process=False)
        else:
            verts,faces=shape.tessellate(.20,.35)
            mesh=trimesh.Trimesh(vertices=[v.toTuple() for v in verts],faces=faces,process=True)
            target=5500 if 'spindle' in p['key'] else 1600
            if len(mesh.faces)>target:
                mesh=mesh.simplify_quadric_decimation(face_count=target)
        item={k:v for k,v in p.items() if k not in ['shape','display_path','display_radius']}
        item['vertices']=np.round(mesh.vertices*50).astype(int).flatten().tolist()
        item['scale']=.02
        item['triangles']=mesh.faces.flatten().tolist()
        bb=shape.BoundingBox();item['center']=[round((bb.xmin+bb.xmax)/2,3),round((bb.ymin+bb.ymax)/2,3),round((bb.zmin+bb.zmax)/2,3)]
        item['volume_mm3']=round(shape.Volume(),3)
        result.append(item)
    return result

def save_assembly(parts,name):
    assy=cq.Assembly(name=name)
    for i,p in enumerate(parts):
        c=p['color'].lstrip('#');rgb=[int(c[k:k+2],16)/255 for k in (0,2,4)]
        assy.add(p['shape'],name=p['key'].replace('后','rear').replace('前','front'),color=cq.Color(*rgb))
    assy.export(str(OUT/(name+'.step')))

def build_drive():
    parts=[]
    for idx in [0,1]:
        phi=math.asin(21/90) if idx==0 else math.pi-math.asin(21/90)
        endz=idx*6
        points=[]
        for i in range(181):
            t=i/180
            angle=phi+2*math.pi*3*(t-1)
            z=endz+(-3*(1-t) if idx==0 else 3*(1-t))
            points.append((3*math.cos(angle),90+3*math.sin(angle),z))
        # Straight external tangent between the two actual cable pitch circles.
        end=(24*math.cos(phi),24*math.sin(phi),endz)
        start=points[-1]
        for i in range(1,12):points.append(tuple(start[k]+(end[k]-start[k])*i/11 for k in range(3)))
        final=math.radians(40 if idx==0 else -40)
        for i in range(1,65):
            a=phi+(final-phi)*i/64
            z=endz+2.9*max(0,(i/64-.85)/.15)
            r=24-.2*max(0,(i/64-.85)/.15)
            points.append((r*math.cos(a),r*math.sin(a),z))
        parts.append(dict(key='drive_cable_'+str(idx),name=('A' if idx==0 else 'B')+' 绳：螺旋卷绕／自由跨距／输出槽',shape=tube(points,.4),color=C['blue' if idx==0 else 'orange'],group='drive',stage=5,explode=[-12 if idx==0 else 12,0,0],source='自定义绳路',description='Ø0.8 绳；中位各约3圈。图示中位几何，端头轴向导入、偏角及全行程排绳仍需验证。',display_path=points,display_radius=.4))
    return parts

def build_abduction_cords():
    parts=[]
    for idx in [0,1]:
        phi=(1 if idx==0 else -1)*math.acos(15/60);endz=19.7+idx*6
        points=[]
        for i in range(181):
            t=i/180;a=phi+6*math.pi*(t-1);z=endz+(-3*(1-t) if idx==0 else 3*(1-t))
            points.append((60+3*math.cos(a),3*math.sin(a),z))
        start=points[-1];end=(18*math.cos(phi),18*math.sin(phi),endz)
        for i in range(1,12):points.append(tuple(start[k]+(end[k]-start[k])*i/11 for k in range(3)))
        final=math.pi if idx==0 else -math.pi
        for i in range(1,55):
            a=phi+(final-phi)*i/54;points.append((18*math.cos(a),18*math.sin(a),endz))
        points.extend([(-17,0,endz+.15),(-16,0,endz+.3)])
        parts.append(dict(key='abd_cable_'+str(idx),name='外展 '+('A' if idx==0 else 'B')+' 双向拉绳',shape=tube(points,.4),color=C['blue' if idx==0 else 'orange'],group='abduction',stage=5,explode=[0,(-1 if idx==0 else 1)*12,0],source='自定义绳路',description='RI20固定，R3驱动R18外展轮；中位绳路，绳端过渡及运动扫掠尚需验证。',display_path=points,display_radius=.4))
    return parts

def main():
    allparts=[]; motors={}
    for model in ['ri30','ri20']:
        print('Building',model,flush=True)
        parts=build_motor(model)
        save_assembly(parts,model+'_motor_module')
        motors[model]=parts
        allparts+=parts
    from src.components.z_common.tendon.joint_geometry import build_joint
    joint=build_joint();save_assembly(joint,'wood_leg_joint');allparts+=joint
    drive=build_drive();allparts+=drive
    connected=[dict(p,shape=p['shape'].translate((0,90,-19.7))) for p in motors['ri30'] if not p.get('localCord')]+joint+drive
    save_assembly(connected,'complete_single_axis')
    from src.components.z_common.tendon.supplemental_geometry import build_abduction
    abd=build_abduction()+build_abduction_cords();allparts+=abd
    full_abd=abd+[dict(p,key='fixed_'+p['key'],shape=p['shape'].translate((60,0,0))) for p in motors['ri20'] if not p.get('localCord')]
    for i,xyz in enumerate([(-60,-25,-14.4),(-60,25,-14.4),(-60,0,28.9)]):
        full_abd += [dict(p,key=f'cradle{i}_'+p['key'],shape=p['shape'].rotate((0,0,0),(0,1,0),90).translate(xyz)) for p in motors['ri30']]
    save_assembly(full_abd,'abduction_cradle')
    data=serialize(allparts)
    (OUT/'geometry.json').write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')))
    (OUT/'geometry.b64').write_text(base64.b64encode(gzip.compress(json.dumps(data,ensure_ascii=False,separators=(',',':')).encode(),compresslevel=9)).decode())
    bom=[{k:v for k,v in p.items() if k not in ['shape','color','explode','display_path','display_radius']} for p in allparts]
    (OUT/'components.json').write_text(json.dumps(bom,ensure_ascii=False,indent=2))
    print('Exported',len(allparts),'component groups; compressed bytes', (OUT/'geometry.b64').stat().st_size,flush=True)

if __name__=='__main__': main()
