using System.Runtime.InteropServices;
using System.Text;

namespace OmegaViewer;

internal static class Native
{
    [StructLayout(LayoutKind.Sequential)]
    internal struct HidInfo
    {
        public IntPtr Path;
        public ushort Vendor, Product;
        public IntPtr Serial;
        public ushort Release;
        public IntPtr Manufacturer, ProductName;
        public ushort UsagePage, Usage;
        public int Interface;
        public IntPtr Next;
        public int Bus;
    }
    [DllImport("hidapi")] internal static extern int hid_init();
    [DllImport("hidapi")] internal static extern int hid_exit();
    [DllImport("hidapi")] internal static extern void hid_darwin_set_open_exclusive(int value);
    [DllImport("hidapi")] internal static extern IntPtr hid_enumerate(ushort vid, ushort pid);
    [DllImport("hidapi")] internal static extern void hid_free_enumeration(IntPtr head);
    [DllImport("hidapi")] internal static extern IntPtr hid_open_path([MarshalAs(UnmanagedType.LPUTF8Str)] string path);
    [DllImport("hidapi")] internal static extern int hid_read_timeout(IntPtr device, byte[] data, nuint size, int milliseconds);
    [DllImport("hidapi")] internal static extern int hid_write(IntPtr device, byte[] data, nuint size);
    [DllImport("hidapi")] internal static extern void hid_close(IntPtr device);
    [DllImport("hidapi")] internal static extern IntPtr hid_error(IntPtr device);
    [DllImport("libusb-1.0")] internal static extern int libusb_init(out IntPtr context);
    [DllImport("libusb-1.0")] internal static extern void libusb_exit(IntPtr context);
    [DllImport("libusb-1.0")] internal static extern IntPtr libusb_open_device_with_vid_pid(IntPtr context, ushort vid, ushort pid);
    [DllImport("libusb-1.0")] internal static extern int libusb_claim_interface(IntPtr handle, int number);
    [DllImport("libusb-1.0")] internal static extern int libusb_release_interface(IntPtr handle, int number);
    [DllImport("libusb-1.0")] internal static extern void libusb_close(IntPtr handle);
    [DllImport("libusb-1.0")] internal static extern int libusb_interrupt_transfer(IntPtr handle, byte endpoint, byte[] data, int length, out int actual, uint timeout);
    [DllImport("libusb-1.0")] internal static extern IntPtr libusb_error_name(int code);

    internal static string HidError(IntPtr handle)
    {
        var p = hid_error(handle);
        if (p == IntPtr.Zero) return "HIDAPI returned no error description";
        var text = new StringBuilder();
        // macOS wchar_t is UTF-32, not .NET's UTF-16.
        for (int i = 0; i < 2048; i++)
        {
            int c = Marshal.ReadInt32(p, i * 4);
            if (c == 0) break;
            text.Append(char.ConvertFromUtf32(c));
        }
        return text.ToString();
    }
    internal static void UsbCheck(int code, string operation)
    {
        if (code < 0) throw new IOException($"{operation}: {Marshal.PtrToStringUTF8(libusb_error_name(code))} ({code})");
    }
    internal static byte[] Read(IntPtr handle, int timeout = 10)
    {
        var data = new byte[256];
        int n = hid_read_timeout(handle, data, (nuint)data.Length, timeout);
        if (n < 0) throw new IOException($"HID read failed: {HidError(handle)}");
        return data[..n];
    }
}
