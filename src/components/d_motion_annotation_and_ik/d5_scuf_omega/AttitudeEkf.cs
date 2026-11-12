using System.Diagnostics;

namespace OmegaViewer;

internal readonly record struct Vec3(double X,double Y,double Z)
{
    internal static readonly Vec3 UnitX=new(1,0,0), UnitY=new(0,1,0), UnitZ=new(0,0,1);
    internal double Norm=>Math.Sqrt(Dot(this,this));
    internal Vec3 Unit()=>Norm>1e-12?this/Norm:throw new ArithmeticException("Cannot normalize zero vector");
    internal double this[int i]=>i switch {0=>X,1=>Y,2=>Z,_=>throw new IndexOutOfRangeException()};
    public static Vec3 operator +(Vec3 a,Vec3 b)=>new(a.X+b.X,a.Y+b.Y,a.Z+b.Z);
    public static Vec3 operator -(Vec3 a,Vec3 b)=>new(a.X-b.X,a.Y-b.Y,a.Z-b.Z);
    public static Vec3 operator -(Vec3 a)=>new(-a.X,-a.Y,-a.Z);
    public static Vec3 operator *(Vec3 a,double s)=>new(a.X*s,a.Y*s,a.Z*s);
    public static Vec3 operator /(Vec3 a,double s)=>a*(1/s);
    internal static double Dot(Vec3 a,Vec3 b)=>a.X*b.X+a.Y*b.Y+a.Z*b.Z;
    internal static Vec3 Cross(Vec3 a,Vec3 b)=>new(a.Y*b.Z-a.Z*b.Y,a.Z*b.X-a.X*b.Z,a.X*b.Y-a.Y*b.X);
}

// Hamilton convention; active rotation sensor/body -> world. Scalar W is first.
internal readonly record struct AttitudeQ(double W,double X,double Y,double Z)
{
    internal static readonly AttitudeQ Identity=new(1,0,0,0);
    internal AttitudeQ Conjugate()=>new(W,-X,-Y,-Z);
    internal AttitudeQ Unit()
    {
        double n=Math.Sqrt(W*W+X*X+Y*Y+Z*Z);
        if (!double.IsFinite(n) || n<1e-12) throw new ArithmeticException("Invalid attitude quaternion");
        return new(W/n,X/n,Y/n,Z/n);
    }
    public static AttitudeQ operator *(AttitudeQ a,AttitudeQ b)=>new(
        a.W*b.W-a.X*b.X-a.Y*b.Y-a.Z*b.Z,
        a.W*b.X+a.X*b.W+a.Y*b.Z-a.Z*b.Y,
        a.W*b.Y-a.X*b.Z+a.Y*b.W+a.Z*b.X,
        a.W*b.Z+a.X*b.Y-a.Y*b.X+a.Z*b.W);
    internal Vec3 Rotate(Vec3 v)
    {
        Vec3 u=new(X,Y,Z), t=Vec3.Cross(u,v)*2;
        return v+t*W+Vec3.Cross(u,t);
    }
    internal static AttitudeQ Exp(Vec3 rotation)
    {
        double angle=rotation.Norm;
        double s=angle<1e-8?.5-angle*angle/48:Math.Sin(angle/2)/angle;
        return new AttitudeQ(Math.Cos(angle/2),rotation.X*s,rotation.Y*s,rotation.Z*s).Unit();
    }
    internal static AttitudeQ FromTo(Vec3 from,Vec3 to)
    {
        var a=from.Unit();var b=to.Unit(); double dot=Vec3.Dot(a,b);
        if (dot < -1+1e-10)
        {
            Vec3 axis=Vec3.Cross(a,Math.Abs(a.X)<.9?Vec3.UnitX:Vec3.UnitY).Unit();
            return new(0,axis.X,axis.Y,axis.Z);
        }
        Vec3 c=Vec3.Cross(a,b);
        return new AttitudeQ(1+dot,c.X,c.Y,c.Z).Unit();
    }
}

internal sealed record ImuScale(double AccelFullScaleG,double GyroFullScaleDps)
{
    // QST QMI8658C CTRL2/3 range encoding, confirmed by Omega command 0x58.
    // https://qstcorp.com/upload/pdf/202202/QMI8658C%20datasheet%20rev%200.9.pdf
    internal double AccelGPerCount=>AccelFullScaleG/32768;
    internal double GyroRadPerCount=>GyroFullScaleDps/32768*Math.PI/180;
    internal static ImuScale FromRegisters(byte[] r)
    {
        if (r.Length!=7 || r[0]>3 || r[3]>7 || r[6]>8 || r[0]!=r[1] || r[1]!=r[2] || r[3]!=r[4] || r[4]!=r[5])
            throw new IOException($"Unsupported IMU range response: {Convert.ToHexString(r)}");
        return new(2*(1<<r[0]),16*(1<<r[3]));
    }
}

internal record AttitudeFrame(bool Ready,AttitudeQ Rotation,Vec3 Bias,string Status,
    double SampleHz,long Accepted,long Rejected,string Scale)
{
    internal static readonly AttitudeFrame Waiting=new(false,AttitudeQ.Identity,new(),"Waiting for IMU range query",0,0,0,"");
}

// Right-multiplicative error-state EKF: q_true=q*Exp(dtheta), bias_true=b+db.
// F=[-[omega]x,-I;0,0], h=R(q)^T*[0,0,1], H=[[h]x,0].
// Joseph covariance update, followed by the error-injection reset Jacobian.
// Reference: https://ntrs.nasa.gov/citations/20020060647
internal sealed class AttitudeEkf(ImuScale scale)
{
    private AttitudeQ q=AttitudeQ.Identity, reference=AttitudeQ.Identity;
    private Vec3 bias;
    private double[,] covariance=new double[6,6];
    private bool ready;
    private long previous, calibrationStart, last;
    private double samplePeriod;
    private int calibrationCount;
    private Vec3 meanAccel,meanGyro,m2Accel,m2Gyro;
    private long accepted,rejected;
    private string status="Hold still for 2 seconds";
    // Conservative initial tuning. Physical units: radians, radians/s, g.
    private const double GyroNoise=.003, BiasWalk=.00015, GravityNoise=.02;

    internal AttitudeFrame Snapshot()=>new(ready,ready?(q*reference.Conjugate()).Unit():AttitudeQ.Identity,
        bias,status,samplePeriod>0?1/samplePeriod:0,accepted,rejected,
        $"±{scale.AccelFullScaleG} g / ±{scale.GyroFullScaleDps}°/s · host timing");
    internal long LastSample=>last;

    internal void Update(Vec3 rawGyro,Vec3 rawAccel,long timestamp)
    {
        Vec3 gyro=rawGyro*scale.GyroRadPerCount, accel=rawAccel*scale.AccelGPerCount;
        if (!double.IsFinite(gyro.Norm+accel.Norm))
            throw new IOException("Nonfinite IMU sample");
        double dt=previous==0?0:Stopwatch.GetElapsedTime(previous,timestamp).TotalSeconds;
        previous=last=timestamp;
        if (dt<0) throw new IOException("IMU monotonic timestamp moved backwards");
        if (dt>.25) throw new IOException($"IMU gap {dt*1000:F0} ms; orientation stopped, restart and recalibrate");
        if (dt>0) samplePeriod=samplePeriod==0?dt:.98*samplePeriod+.02*dt;
        if (!ready) { Calibrate(gyro,accel,timestamp); return; }
        if (dt<=0) throw new IOException("Duplicate IMU timestamp");
        Vec3 omega=gyro-bias;
        q=(q*AttitudeQ.Exp(omega*dt)).Unit();
        double[,] f=new double[6,6], skew=Skew(omega);
        for (int i=0;i<3;i++)
        {
            f[i,i+3]=-1;
            for (int j=0;j<3;j++) f[i,j]=-skew[i,j];
        }
        // Second-order transition; retain bias/attitude process-noise coupling.
        double[,] phi=Identity(6), f2=Multiply(f,f);
        for (int i=0;i<6;i++) for (int j=0;j<6;j++) phi[i,j]+=f[i,j]*dt+.5*f2[i,j]*dt*dt;
        covariance=Multiply(Multiply(phi,covariance),Transpose(phi));
        double bg=BiasWalk*BiasWalk;
        for (int i=0;i<3;i++)
        {
            covariance[i,i]+=GyroNoise*GyroNoise*dt+bg*dt*dt*dt/3;
            covariance[i+3,i+3]+=bg*dt;
            covariance[i,i+3]-=bg*dt*dt/2; covariance[i+3,i]-=bg*dt*dt/2;
        }
        double magnitudeError=Math.Abs(accel.Norm-1);
        if (magnitudeError>.15)
        {
            rejected++;status="Gyro prediction · acceleration gate";CheckCovariance();return;
        }
        Vec3 h=q.Conjugate().Rotate(Vec3.UnitZ), z=accel.Unit(), residual=z-h;
        double[,] hMatrix=new double[3,6], hs=Skew(h);
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) hMatrix[i,j]=hs[i,j];
        double variance=GravityNoise*GravityNoise*(1+Math.Pow(magnitudeError/.035,2));
        double[,] pht=Multiply(covariance,Transpose(hMatrix)), s=Multiply(hMatrix,pht);
        for (int i=0;i<3;i++) s[i,i]+=variance;
        double[,] inverse=Inverse3(s);
        double nis=0;
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) nis+=residual[i]*inverse[i,j]*residual[j];
        if (nis>11.345)
        {
            rejected++;status="Gyro prediction · innovation gate";CheckCovariance();return;
        }
        double[,] k=Multiply(pht,inverse);
        double[] delta=new double[6];
        for(int i=0;i<6;i++) for(int j=0;j<3;j++) delta[i]+=k[i,j]*residual[j];
        var dtheta=new Vec3(delta[0],delta[1],delta[2]);
        q=(q*AttitudeQ.Exp(dtheta)).Unit();
        bias+=new Vec3(delta[3],delta[4],delta[5]);
        double[,] a=Identity(6), kh=Multiply(k,hMatrix);
        for(int i=0;i<6;i++) for(int j=0;j<6;j++) a[i,j]-=kh[i,j];
        double[,] next=Multiply(Multiply(a,covariance),Transpose(a)), kk=Multiply(k,Transpose(k));
        for(int i=0;i<6;i++) for(int j=0;j<6;j++) next[i,j]+=variance*kk[i,j];
        double[,] reset=Identity(6), correction=Skew(dtheta);
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) reset[i,j]-=.5*correction[i,j];
        covariance=Multiply(Multiply(reset,next),Transpose(reset));
        accepted++;status="MEKF tracking · relative yaw";CheckCovariance();
    }
    private void Calibrate(Vec3 gyro,Vec3 accel,long timestamp)
    {
        if (Math.Abs(accel.Norm-1)>.15 || gyro.Norm>.35)
        {
            ResetCalibration();status="Hold still · movement detected";return;
        }
        if(calibrationCount==0) calibrationStart=timestamp;
        calibrationCount++;
        Accumulate(accel,ref meanAccel,ref m2Accel);
        Accumulate(gyro,ref meanGyro,ref m2Gyro);
        double elapsed=Stopwatch.GetElapsedTime(calibrationStart,timestamp).TotalSeconds;
        status=$"Hold still · {Math.Min(elapsed,2):F1} / 2.0 s";
        if (elapsed<2 || calibrationCount<80) return;
        double accelStd=Math.Sqrt((m2Accel.X+m2Accel.Y+m2Accel.Z)/(calibrationCount-1));
        double gyroStd=Math.Sqrt((m2Gyro.X+m2Gyro.Y+m2Gyro.Z)/(calibrationCount-1));
        if (accelStd>.015 || gyroStd>.012)
        {
            ResetCalibration();status="Hold still · calibration variance too high";return;
        }
        bias=meanGyro;
        q=reference=AttitudeQ.FromTo(meanAccel,Vec3.UnitZ);
        Vec3 vertical=meanAccel.Unit();
        for(int i=0;i<3;i++) for(int j=0;j<3;j++)
            covariance[i,j]=.0004*(i==j?1:0)+(.12-.0004)*vertical[i]*vertical[j];
        for(int i=3;i<6;i++) covariance[i,i]=.000025;
        ready=true;status="MEKF tracking · relative yaw";
        Console.WriteLine($"MEKF calibrated: {calibrationCount} samples; gyro bias [{bias.X:F5},{bias.Y:F5},{bias.Z:F5}] rad/s");
    }
    private void Accumulate(Vec3 x,ref Vec3 mean,ref Vec3 m2)
    {
        var delta=x-mean;mean+=delta/calibrationCount;var d2=x-mean;
        m2+=new Vec3(delta.X*d2.X,delta.Y*d2.Y,delta.Z*d2.Z);
    }
    private void ResetCalibration() { calibrationCount=0;meanAccel=meanGyro=m2Accel=m2Gyro=new(); }
    private void CheckCovariance()
    {
        for(int i=0;i<6;i++)
        {
            if(!double.IsFinite(covariance[i,i]) || covariance[i,i]<=0) throw new ArithmeticException("MEKF covariance lost positive diagonal");
            for(int j=i+1;j<6;j++)
            {
                double s=.5*(covariance[i,j]+covariance[j,i]);
                if(!double.IsFinite(s)) throw new ArithmeticException("MEKF covariance became nonfinite");
                covariance[i,j]=covariance[j,i]=s;
            }
        }
        if(!double.IsFinite(bias.Norm)) throw new ArithmeticException("MEKF gyro bias became nonfinite");
    }
    private static double[,] Skew(Vec3 v)=>new double[,]{{0,-v.Z,v.Y},{v.Z,0,-v.X},{-v.Y,v.X,0}};
    private static double[,] Identity(int n) { var a=new double[n,n];for(int i=0;i<n;i++)a[i,i]=1;return a; }
    private static double[,] Transpose(double[,] a)
    {
        var b=new double[a.GetLength(1),a.GetLength(0)];
        for(int i=0;i<a.GetLength(0);i++)for(int j=0;j<a.GetLength(1);j++)b[j,i]=a[i,j];return b;
    }
    private static double[,] Multiply(double[,] a,double[,] b)
    {
        if(a.GetLength(1)!=b.GetLength(0))throw new ArgumentException("Matrix dimensions");
        var c=new double[a.GetLength(0),b.GetLength(1)];
        for(int i=0;i<c.GetLength(0);i++)for(int k=0;k<a.GetLength(1);k++)for(int j=0;j<c.GetLength(1);j++)c[i,j]+=a[i,k]*b[k,j];
        return c;
    }
    private static double[,] Inverse3(double[,] a)
    {
        // Cholesky solves avoid explicitly dividing by a small determinant.
        var l=new double[3,3];
        for(int i=0;i<3;i++)for(int j=0;j<=i;j++)
        {
            double s=a[i,j];for(int k=0;k<j;k++)s-=l[i,k]*l[j,k];
            if(i==j)
            {
                if(!double.IsFinite(s)||s<=0)throw new ArithmeticException("MEKF innovation covariance is not positive definite");
                l[i,j]=Math.Sqrt(s);
            }
            else l[i,j]=s/l[j,j];
        }
        var inv=new double[3,3];
        for(int col=0;col<3;col++)
        {
            var y=new double[3];var x=new double[3];
            for(int i=0;i<3;i++) { double s=i==col?1:0;for(int k=0;k<i;k++)s-=l[i,k]*y[k];y[i]=s/l[i,i]; }
            for(int i=2;i>=0;i--) { double s=y[i];for(int k=i+1;k<3;k++)s-=l[k,i]*x[k];x[i]=s/l[i,i]; }
            for(int i=0;i<3;i++)inv[i,col]=x[i];
        }
        return inv;
    }
}
