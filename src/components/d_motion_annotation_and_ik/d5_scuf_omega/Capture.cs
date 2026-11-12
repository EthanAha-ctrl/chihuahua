using System.Runtime.InteropServices;

namespace OmegaViewer;

internal sealed class Capture(InputState state)
{
    private readonly CancellationTokenSource stop = new();
    private Task? worker;
    internal Task Completion => worker ?? Task.CompletedTask;
    internal void Start() => worker = Task.Run(Run);
    internal void Stop() => stop.Cancel();
    private void Guard(Action work)
    {
        try { work(); }
        catch (Exception e) { state.Fail(e); stop.Cancel(); }
    }
    private void Run()
    {
        IntPtr context=IntPtr.Zero, usb=IntPtr.Zero;
        bool initialized=false, claimed=false;
        var handles = new Dictionary<int, IntPtr>();
        VendorProtocol? vendor=null;
        var readers = new List<Task>();
        try
        {
            if (!OperatingSystem.IsMacOS()) throw new PlatformNotSupportedException("Native transport currently built/validated for macOS; no platform fallback");
            if (Native.hid_init()!=0) throw new IOException("HIDAPI init failed");
            initialized=true;
            Native.hid_darwin_set_open_exclusive(0);
            var entries=new Dictionary<string,Native.HidInfo>();
            IntPtr head=Native.hid_enumerate(0x1B1C,0);
            try
            {
                for (IntPtr p=head;p!=IntPtr.Zero;)
                {
                    var info=Marshal.PtrToStructure<Native.HidInfo>(p);
                    if (info.Product is 0x3A27 or 0x3A28)
                        entries[Marshal.PtrToStringUTF8(info.Path) ?? throw new IOException("Null HID path")]=info;
                    p=info.Next;
                }
            }
            finally { Native.hid_free_enumeration(head); }
            if (entries.Count!=2 || entries.Values.Select(x=>x.Product).Distinct().Count()!=1)
                throw new IOException($"Expected two HID paths for one wired Omega; found {entries.Count}. Check USB and Input Monitoring permission.");
            state.Ps=entries.First().Value.Product==0x3A27;
            foreach (var (path,info) in entries)
            {
                var handle=Native.hid_open_path(path);
                if (handle==IntPtr.Zero) throw new IOException($"Open IF {info.Interface}: {Native.HidError(handle)}. Grant Input Monitoring to the launching application.");
                if (!handles.TryAdd(info.Interface,handle)) { Native.hid_close(handle); throw new IOException("Duplicate HID interface"); }
            }
            int vendorInterface=state.Ps?1:0;
            vendor=new(handles[vendorInterface], raw=>state.Accept(vendorInterface,raw));
            vendor.Start(!state.Ps,state.ConfigureImu);
            if (!state.Ps)
            {
                Native.UsbCheck(Native.libusb_init(out context),"libusb init");
                usb=Native.libusb_open_device_with_vid_pid(context,0x1B1C,0x3A28);
                if (usb==IntPtr.Zero) throw new IOException("Cannot open Omega PC USB device");
                Native.UsbCheck(Native.libusb_claim_interface(usb,5),"Claim IF 5");
                claimed=true;
            }
            foreach (var (number,handle) in handles)
            {
                readers.Add(Task.Factory.StartNew(()=>Guard(()=>
                {
                    while (!stop.IsCancellationRequested)
                    {
                        byte[] r=Native.Read(handle);
                        if (r.Length!=0) state.Accept(number,r,vendor?.Session ?? 0);
                        if(number==0) state.CheckImuFreshness();
                    }
                }),CancellationToken.None,TaskCreationOptions.LongRunning,TaskScheduler.Default));
            }
            if (usb!=IntPtr.Zero) readers.Add(Task.Factory.StartNew(()=>Guard(()=>
            {
                byte[] buffer=new byte[32];
                while (!stop.IsCancellationRequested)
                {
                    int code=Native.libusb_interrupt_transfer(usb,0x84,buffer,buffer.Length,out int n,20);
                    if (code==-7) continue; // Bounded wait expired; no data has been fabricated.
                    Native.UsbCheck(code,"Read Xbox interface");
                    state.Accept(5,buffer[..n]);
                }
            }),CancellationToken.None,TaskCreationOptions.LongRunning,TaskScheduler.Default));
            state.SetStatus($"{(state.Ps?"PS":"PC")} mode · native HID/USB capture running");
            Console.WriteLine($"READY: {(state.Ps?"PS":"PC")} mode, native C# acquisition, {readers.Count} readers");
            Task.WaitAll(readers.ToArray());
        }
        catch (Exception e) { state.Fail(e); stop.Cancel(); }
        finally
        {
            stop.Cancel();
            Task.WaitAll(readers.ToArray());
            void Cleanup(Action action) { try { action(); } catch (Exception e) { state.Fail(e); } }
            if (vendor!=null) Cleanup(vendor.Stop);
            if (claimed) Cleanup(()=>Native.UsbCheck(Native.libusb_release_interface(usb,5),"Release IF 5"));
            if (usb!=IntPtr.Zero) Native.libusb_close(usb);
            if (context!=IntPtr.Zero) Native.libusb_exit(context);
            foreach (var handle in handles.Values) Native.hid_close(handle);
            if (initialized) Native.hid_exit();
            Console.WriteLine("Capture stopped; native handles closed; see FATAL output if restoration failed.");
        }
    }
}
