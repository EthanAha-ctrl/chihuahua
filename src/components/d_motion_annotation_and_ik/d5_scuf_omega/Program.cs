using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.ApplicationLifetimes;

namespace OmegaViewer;

internal static class Program
{
    [STAThread]
    public static int Main(string[] args)
    {
        if (args.Length==2 && args[0]=="--probe")
        {
            var state=new InputState(); var capture=new Capture(state); capture.Start();
            var end=DateTime.UtcNow.AddSeconds(int.Parse(args[1]));
            Frame? frame=null;
            while (DateTime.UtcNow<end && !capture.Completion.IsCompleted)
            {
                Thread.Sleep(16); frame=state.TakeFrame();
                if (frame.Error!=null) break;
            }
            capture.Stop(); capture.Completion.GetAwaiter().GetResult();
            frame=state.TakeFrame();
            Console.WriteLine($"Reports: {string.Join(", ",frame.Reports.Select(x=>$"IF{x.Key}={x.Value}"))}; events={frame.Sequence}; FIFO peak={frame.QueuePeak}");
            Console.WriteLine($"IMU: {string.Join(',',frame.Motion)}; touch: {string.Join(';',frame.Touch.Select(x=>x.ToString()))}");
            if (frame.Error!=null || frame.Reports.Count==0) { Console.Error.WriteLine(frame.Error ?? "No reports received"); return 1; }
            return 0;
        }
        return AppBuilder.Configure<OmegaApp>().UsePlatformDetect().LogToTrace().StartWithClassicDesktopLifetime(args);
    }
}

public sealed class OmegaApp : Application
{
    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
            desktop.MainWindow=new MonitorWindow();
        base.OnFrameworkInitializationCompleted();
    }
}
