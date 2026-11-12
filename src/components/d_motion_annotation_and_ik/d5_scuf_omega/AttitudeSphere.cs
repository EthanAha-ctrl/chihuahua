using Avalonia;
using Avalonia.Media;

namespace OmegaViewer;

// Orthographic, oblique camera. Fixed Z-up world; +Y is the inward equator
// direction. A short transverse marker reveals rotation about the main arrow.
internal sealed partial class MonitorView
{
    private static readonly Vec3 CameraRight=new(Math.Cos(.42),-Math.Sin(.42),0);
    private static readonly Vec3 CameraUp=new(Math.Sin(.42)*Math.Sin(.30),Math.Cos(.42)*Math.Sin(.30),Math.Cos(.30));
    private static readonly Vec3 CameraDepth=new(Math.Sin(.42)*Math.Cos(.30),Math.Cos(.42)*Math.Cos(.30),-Math.Sin(.30));
    private const double SphereX=1375,SphereY=360,SphereRadius=130;
    private static Point Project(Vec3 p)=>new(SphereX+SphereRadius*Vec3.Dot(CameraRight,p),SphereY-SphereRadius*Vec3.Dot(CameraUp,p));

    private static void DrawAttitude(DrawingContext dc,AttitudeFrame attitude)
    {
        dc.DrawRectangle(Panel,null,new Rect(1210,160,330,650),12,12);
        Label(dc,"BLOCH SPHERE",1230,182,22,Text);
        Label(dc,"Quaternion MEKF · relative attitude",1230,215,14,Muted);
        dc.DrawEllipse(Brush("#131E2D"),new Pen(Muted,1),new Point(SphereX,SphereY),SphereRadius,SphereRadius);
        // Draw far arcs dim/dashed, near arcs solid. Equator is emphasized.
        for(int back=1;back>=0;back--)
        {
            Circle3(dc,a=>new Vec3(Math.Cos(a),Math.Sin(a),0),back==1,true);
            Circle3(dc,a=>new Vec3(Math.Cos(a),0,Math.Sin(a)),back==1,false);
            Circle3(dc,a=>new Vec3(0,Math.Cos(a),Math.Sin(a)),back==1,false);
            foreach(double z in new[]{-.5,.5})
                Circle3(dc,a=>new Vec3(Math.Sqrt(1-z*z)*Math.Cos(a),Math.Sqrt(1-z*z)*Math.Sin(a),z),back==1,false);
        }
        var axisPen=new Pen(Brush("#506680"),1);
        foreach(var (v,name) in new[]{(Vec3.UnitX,"X"),(Vec3.UnitY,"Y / inward"),(Vec3.UnitZ,"Z / up")})
        {
            dc.DrawLine(axisPen,Project(-v),Project(v));
            Point p=Project(v*1.12);
            Label(dc,name,p.X-12,p.Y-9,12,Muted);
        }
        var rotation=attitude.Rotation;
        Vec3 tip=rotation.Rotate(Vec3.UnitY), transverse=rotation.Rotate(Vec3.UnitX);
        var arrowBrush=attitude.Ready?Green:Muted;
        var shaft=new Pen(arrowBrush,3);
        Point origin=Project(new()),head=Project(tip);
        dc.DrawLine(shaft,origin,head);
        dc.DrawLine(shaft,head,Project(tip*.80+transverse*.09));
        dc.DrawLine(shaft,head,Project(tip*.80-transverse*.09));
        // Distinct ends remove the previous marker's 180-degree ambiguity.
        Point hub=Project(tip*.60),left=Project(tip*.60-transverse*.22),right=Project(tip*.60+transverse*.22);
        var rightBrush=Brush("#FFB454");
        dc.DrawLine(new Pen(Blue,3),left,hub);
        dc.DrawLine(new Pen(rightBrush,3),hub,right);
        dc.DrawEllipse(Blue,null,left,4,4);
        dc.DrawEllipse(rightBrush,null,right,6,6);
        dc.DrawEllipse(arrowBrush,null,head,4,4);
        dc.DrawEllipse(Text,null,origin,3,3);
        Label(dc,"L: blue   R: orange · initial pose = zero",1230,505,12,Muted,290);
        Label(dc,attitude.Status,1230,535,15,attitude.Ready?Green:Blue,290);
        Label(dc,$"qΔ  (w, x, y, z)\n{rotation.W,9:F4}   {rotation.X,9:F4}\n{rotation.Y,9:F4}   {rotation.Z,9:F4}",1230,582,16,Text,290);
        var b=attitude.Bias*(180/Math.PI);
        Label(dc,$"Gyro bias °/s\n{b.X:F2}    {b.Y:F2}    {b.Z:F2}",1230,650,14,Muted,290);
        Label(dc,$"IMU {attitude.SampleHz:F0} Hz\nGravity updates {attitude.Accepted}\nGated samples {attitude.Rejected}",1230,700,13,Muted,290);
        Label(dc,attitude.Scale,1230,766,11,Muted,290);
        Label(dc,"Yaw is relative; long-term drift is expected.",1230,790,10,Muted,290);
    }
    private static void Circle3(DrawingContext dc,Func<double,Vec3> point,bool back,bool equator)
    {
        var pen=new Pen(Brush(back?"#354960":equator?"#8DA9C7":"#4D6884"),equator?1.4:.8);
        const int steps=120;
        for(int i=0;i<steps;i++)
        {
            Vec3 a=point(2*Math.PI*i/steps),b=point(2*Math.PI*(i+1)/steps);
            bool far=Vec3.Dot(CameraDepth,a+b)>0;
            if(far!=back || (back && i%4<2))continue;
            dc.DrawLine(pen,Project(a),Project(b));
        }
    }
}
