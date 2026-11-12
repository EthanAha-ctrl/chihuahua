using System.Diagnostics;
using System.Globalization;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Threading;

namespace OmegaViewer;

internal sealed class MonitorWindow : Window
{
    private readonly InputState state=new();
    private readonly Capture capture;
    private readonly MonitorView view=new();
    private readonly DispatcherTimer timer;
    private bool closing, cleaned;

    internal MonitorWindow()
    {
        Title="SCUF Omega — Avalonia / native HID";
        Width=1200; Height=900; MinWidth=840; MinHeight=650;
        Background=new SolidColorBrush(Color.Parse("#101722"));
        Content=view;
        capture=new(state);
        timer=new DispatcherTimer(TimeSpan.FromMilliseconds(1000.0/60),DispatcherPriority.Background,(_,_)=>
        {
            view.Update(state.TakeFrame(),state.Ps);
        });
        Opened+=(_,_)=>{ capture.Start(); timer.Start(); };
        Closing+=OnClosing;
    }
    private async void OnClosing(object? sender, WindowClosingEventArgs e)
    {
        if (cleaned) return;
        e.Cancel=true;
        if (closing) return;
        closing=true;
        state.SetStatus("Stopping capture and restoring touch setting…");
        capture.Stop();
        await capture.Completion;
        timer.Stop();
        var final=state.TakeFrame();
        view.Update(final,state.Ps);
        cleaned=true;
        if (final.Error!=null)
        {
            // Leave cleanup failure visible. A second close dismisses the window.
            Title="FATAL — "+final.Error;
            Environment.ExitCode=1;
        }
        else Close();
    }
}

internal sealed class MonitorView : Control
{
    private static readonly IBrush Text=Brush("#E3EDF7"), Muted=Brush("#94A9C2"), Green=Brush("#59E3A5"),
        Blue=Brush("#72BBFF"), Panel=Brush("#192435"), Idle=Brush("#324358"), Red=Brush("#FF6262");
    private static readonly Typeface Font=new("Arial");
    private Frame? frame;
    private bool ps;
    private readonly Dictionary<string,long> pulses=new();
    private readonly Queue<string> recent=new();
    private long metricTime=Stopwatch.GetTimestamp();
    private int renders;
    private double fps, queueDelay, maxQueueDelay;
    private int batchSize;
    private static IBrush Brush(string hex)=>new SolidColorBrush(Color.Parse(hex));

    internal void Update(Frame next,bool isPs)
    {
        frame=next; ps=isPs; batchSize=next.Events.Length;
        maxQueueDelay=Math.Max(maxQueueDelay,next.QueueDelayMs);
        long now=Stopwatch.GetTimestamp();
        foreach (var ev in next.Events)
        {
            if (ev.Down) pulses[ev.Name]=now; // Even a press+release between frames gets a visible pulse.
            recent.Enqueue($"#{ev.Sequence} {ev.Name} {(ev.Down?"down":"up")}");
            while (recent.Count>4) recent.Dequeue();
        }
        double elapsed=Stopwatch.GetElapsedTime(metricTime).TotalSeconds;
        if (elapsed>=1)
        {
            fps=renders/elapsed; renders=0; metricTime=now;
            queueDelay=maxQueueDelay; maxQueueDelay=0;
        }
        InvalidateVisual();
    }
    public override void Render(DrawingContext context)
    {
        base.Render(context); renders++;
        if (Bounds.Width<=0 || Bounds.Height<=0) return;
        double scale=Math.Min(Bounds.Width/1200,Bounds.Height/850);
        using var transform=context.PushTransform(Matrix.CreateScale(scale,scale));
        if (frame==null) { Label(context,"Opening native input…",30,30,24,Text); return; }
        Label(context,$"OMEGA   {(ps?"PS":"PC")} MODE",30,24,26,Text);
        Label(context,"Avalonia · C# · native HIDAPI / libusb",720,30,17,Muted);
        Label(context,frame.Status,30,65,17,frame.Error==null?Green:Red);
        Label(context,$"Render {fps:F0} fps   |   host → UI event wait, max / 1 s: {queueDelay:F1} ms   |   FIFO batch {batchSize}, peak {frame.QueuePeak}",30,95,14,Muted);
        Label(context,"Green: held    Blue: recent press (150 ms)    # : press count",30,121,14,Muted);
        Button(context,"L2",50,160); Button(context,"R2",1030,160);
        Trigger(context,185,170,frame.Triggers[0]); Trigger(context,745,170,frame.Triggers[1]);
        Button(context,"L1",50,220); Button(context,"R1",1030,220);
        Button(context,"Create",440,160); Button(context,"Options",665,160);
        Button(context,"Up",180,270); Button(context,"Left",60,320); Button(context,"Right",300,320); Button(context,"Down",180,370);
        Button(context,"Triangle",940,270); Button(context,"Square",820,320); Button(context,"Circle",1060,320); Button(context,"Cross",940,370);
        var pad=new Rect(475,245,250,90);
        context.DrawRectangle(Panel,new Pen(Muted,1),pad,10,10);
        for (int i=0;i<2;i++)
        {
            var t=frame.Touch[i];
            if (t.Active) context.DrawEllipse(i==0?Green:Blue,null,new Point(pad.X+t.X/1919.0*pad.Width,pad.Y+t.Y/1079.0*pad.Height),6,6);
        }
        Button(context,"Touch click",535,350,140);
        Button(context,"PS",550,410);
        Stick(context,400,410,frame.Sticks[0],frame.Sticks[1]);
        Stick(context,800,410,frame.Sticks[2],frame.Sticks[3]);
        Button(context,"L3",350,470); Button(context,"R3",750,470);
        string[] extra=["L outer","L inner","L SAX","R SAX","R inner","R outer"];
        for (int i=0;i<extra.Length;i++) Button(context,extra[i],30+i*195,535,170);
        for (int i=0;i<5;i++) Button(context,$"G{i+1}",30+i*195,600,170);
        Label(context,$"IMU raw   gyro [{string.Join(", ",frame.Motion.Take(3))}]     accel [{string.Join(", ",frame.Motion.Skip(3))}]",30,668,17,Text);
        Label(context,$"Touch 1: {frame.Touch[0].Active} ({frame.Touch[0].X}, {frame.Touch[0].Y})     Touch 2: {frame.Touch[1].Active} ({frame.Touch[1].X}, {frame.Touch[1].Y})",30,697,15,Muted);
        Label(context,"Keyboard: "+frame.Keyboard,30,724,14,Muted);
        Label(context,string.Join("   |   ",frame.Reports.OrderBy(x=>x.Key).Select(x=>$"IF{x.Key}: {x.Value} reports"))+$"   |   {frame.Sequence} button edges",30,749,14,Muted);
        Label(context,string.Join("   ·   ",recent),30,774,14,Text);
        Label(context,"Vendor: "+frame.VendorNotice,30,799,12,Muted);
        if (frame.Error!=null)
        {
            context.DrawRectangle(Brush("#401C28"),new Pen(Red,2),new Rect(20,145,1160,110),8,8);
            Label(context,"FATAL — "+frame.Error,35,160,16,Red,1120);
        }
    }
    private void Button(DrawingContext dc,string name,double x,double y,double width=110)
    {
        var s=frame!.Buttons[name];
        bool pulse=pulses.TryGetValue(name,out long t) && Stopwatch.GetElapsedTime(t).TotalMilliseconds<150;
        var spec=InputState.Specs.First(b=>b.Name==name);
        IBrush fill=s.Down?Green:pulse?Blue:Idle;
        var rect=new Rect(x,y,width,44);
        dc.DrawRectangle(fill,null,rect,7,7);
        string label=name+(spec.Letter.Length==0?"":$" [{spec.Letter}]");
        var text=MakeText(label,15,s.Down||pulse?Panel:Text);
        dc.DrawText(text,new Point(x+(width-text.Width)/2,y+8));
        Label(dc,$"#{s.Presses}",x+6,y+28,10,s.Down||pulse?Panel:Muted);
        if (ps && spec.Usage!=0) Label(dc,"PC mapping",x+width-66,y+29,9,Muted);
    }
    private static void Trigger(DrawingContext dc,double x,double y,byte value)
    {
        dc.DrawRectangle(Panel,null,new Rect(x,y,260,16),4,4);
        dc.DrawRectangle(Green,null,new Rect(x,y,260*value/255.0,16),4,4);
        Label(dc,$"{value}/255   {100*value/255.0:F1}%",x,y+22,14,Text);
    }
    private static void Stick(DrawingContext dc,double x,double y,double sx,double sy)
    {
        dc.DrawEllipse(Panel,new Pen(Muted,1),new Point(x,y),38,38);
        dc.DrawLine(new Pen(Idle,1),new Point(x-35,y),new Point(x+35,y));
        dc.DrawLine(new Pen(Idle,1),new Point(x,y-35),new Point(x,y+35));
        dc.DrawEllipse(Green,null,new Point(x+30*sx,y-30*sy),7,7);
    }
    private static FormattedText MakeText(string value,double size,IBrush brush,double width=double.PositiveInfinity)
        =>new(value,CultureInfo.InvariantCulture,FlowDirection.LeftToRight,Font,size,brush){MaxTextWidth=width};
    private static void Label(DrawingContext dc,string value,double x,double y,double size,IBrush brush,double width=double.PositiveInfinity)
        =>dc.DrawText(MakeText(value,size,brush,width),new Point(x,y));
}
