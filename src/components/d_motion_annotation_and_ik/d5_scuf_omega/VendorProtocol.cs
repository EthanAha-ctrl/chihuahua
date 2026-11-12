using System.Buffers.Binary;
using System.Diagnostics;

namespace OmegaViewer;

internal static class Wire
{
    internal static ushort Crc(ReadOnlySpan<byte> data)
    {
        ushort crc = 0xFFFF;
        foreach (byte b in data)
        {
            crc ^= (ushort)(b << 8);
            for (int i = 0; i < 8; i++) crc = (ushort)((crc & 0x8000) != 0 ? (crc << 1) ^ 0x1021 : crc << 1);
        }
        return crc;
    }
    internal static byte[] Encode(ReadOnlySpan<byte> data)
    {
        var output = new List<byte> { 0 };
        int index = 0, count = 1;
        foreach (byte b in data)
        {
            if (b == 0) { output[index] = (byte)count; index = output.Count; output.Add(0); count = 1; }
            else
            {
                output.Add(b);
                if (++count == 255) { output[index] = 255; index = output.Count; output.Add(0); count = 1; }
            }
        }
        output[index] = (byte)count;
        return output.ToArray();
    }
    internal static byte[] Unpack(ReadOnlySpan<byte> raw)
    {
        if (raw.Length < 7 || (raw[0] & 3) != 0) throw new IOException("Invalid/fragmented vendor packet");
        int end = raw[2..].IndexOf((byte)0);
        if (end < 0) throw new IOException("Vendor COBS terminator missing");
        var data = raw.Slice(2, end);
        var decoded = new List<byte> { raw[0], raw[1] };
        for (int i = 0; i < data.Length;)
        {
            int count = data[i];
            if (count == 0 || i + count > data.Length) throw new IOException("Invalid vendor COBS block");
            for (int j = i + 1; j < i + count; j++) decoded.Add(data[j]);
            i += count;
            if (count < 255 && i < data.Length) decoded.Add(0);
        }
        byte[] p = decoded.ToArray();
        if (p.Length < 6 || Crc(p.AsSpan(0, p.Length - 2)) != BinaryPrimitives.ReadUInt16LittleEndian(p.AsSpan(p.Length - 2)))
            throw new IOException("Vendor CRC mismatch");
        return p[..^2];
    }
    internal static byte[] Request(byte session, ushort command, ReadOnlySpan<byte> payload)
    {
        if (command is not (0 or 1 or 0x58 or 0x59 or 0x77 or 0x78)) throw new ArgumentException("Command outside verified allowlist");
        byte[] p = new byte[6 + payload.Length];
        p[0] = 0x88; p[1] = (byte)(1 | session << 4);
        BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(2), command);
        payload.CopyTo(p.AsSpan(4));
        BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(p.Length - 2), Crc(p.AsSpan(0, p.Length - 2)));
        byte[] encoded = Encode(p.AsSpan(2));
        if (encoded.Length + 3 > 64) throw new ArgumentException("Vendor packet too large");
        byte[] report = new byte[65]; // HIDAPI's zero report ID, then 64-byte USB report.
        report[1] = p[0]; report[2] = p[1]; encoded.CopyTo(report, 3);
        return report;
    }
}

// Protocol verified against Omega PC mode and SCUF's official 2.18.431 firmware.
// No firmware-update commands are permitted; 0x78 writes only the touch switch.
internal sealed class VendorProtocol(IntPtr device, Action<byte[]> notification)
{
    internal byte Session { get; private set; }
    internal ImuScale? Scale { get; private set; }
    private byte? restoreTouch;
    private bool imu;

    private byte[] Request(ushort command, params byte[] payload)
    {
        byte[] report = Wire.Request(Session, command, payload);
        if (Native.hid_write(device, report, (nuint)report.Length) != report.Length)
            throw new IOException($"Vendor write failed: {Native.HidError(device)}");
        long start = Stopwatch.GetTimestamp();
        while (Stopwatch.GetElapsedTime(start).TotalSeconds < 3)
        {
            byte[] raw = Native.Read(device);
            if (raw.Length == 0) continue;
            byte[] p = Wire.Unpack(raw);
            if ((p[1] & 15) == 3) { notification(raw); continue; }
            if (p.Length < 5 || (p[1] & 15) != 2 || BinaryPrimitives.ReadUInt16LittleEndian(p.AsSpan(2)) != command)
                throw new IOException($"Unexpected vendor response: {Convert.ToHexString(p)}");
            if (p[4] != 0) throw new IOException($"Vendor command {command:X2} rejected: {p[4]:X2}");
            return p[5..];
        }
        throw new TimeoutException($"Vendor command {command:X2} timed out");
    }
    internal void Start(bool pcMode = true, Action<ImuScale>? configured = null)
    {
        byte[] response = Request(0, 0x78, 0x56, 0x34, 0x12);
        if (response.Length != 5 || response[0] is < 1 or > 15) throw new IOException("Invalid vendor session ID");
        Session = response[0];
        // Firmware 2.18.431 handler 0x0e050ce8 reads QMI CTRL2/CTRL3 FS bits.
        // Response: accel FS x/y/z, gyro FS x/y/z, gyro ODR. Read-only command.
        byte[] config = Request(0x58);
        Scale = ImuScale.FromRegisters(config);
        configured?.Invoke(Scale);
        Console.WriteLine($"IMU scale: ±{Scale.AccelFullScaleG} g, ±{Scale.GyroFullScaleDps} deg/s; host monotonic timestamps");
        if (!pcMode) return;
        byte[] touch = Request(0x77);
        if (touch.Length != 1 || touch[0] > 1) throw new IOException("Invalid touch switch response");
        if (touch[0] == 0) { restoreTouch = 0; Request(0x78, 1); }
        if (!Request(0x77).SequenceEqual(new byte[] { 1 })) throw new IOException("Touch enable readback failed");
        imu = true;
        Request(0x59, 1);
    }
    internal void Stop()
    {
        var errors = new List<Exception>();
        void Attempt(Action action) { try { action(); } catch (Exception e) { errors.Add(e); } }
        if (imu) Attempt(() => { Request(0x59, 0); imu = false; });
        if (restoreTouch is byte original) Attempt(() =>
        {
            Request(0x78, original);
            if (!Request(0x77).SequenceEqual(new[] { original })) throw new IOException("Touch restore readback failed");
            restoreTouch = null;
        });
        if (Session != 0) Attempt(() => { Request(1, Session); Session = 0; });
        if (errors.Count != 0) throw new AggregateException("Vendor cleanup failed", errors);
    }
}
