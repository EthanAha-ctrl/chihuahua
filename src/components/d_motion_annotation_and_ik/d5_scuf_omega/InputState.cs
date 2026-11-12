using System.Buffers.Binary;
using System.Diagnostics;

namespace OmegaViewer;

internal record ButtonSpec(string Name, string Letter = "", byte Usage = 0);
internal record ButtonEvent(long Sequence, long Time, string Name, bool Down);
internal record ButtonState(bool Down, long LastPress, long Presses);
internal record TouchPoint(bool Active, int X, int Y);
internal record Frame(Dictionary<string, ButtonState> Buttons, double[] Sticks, byte[] Triggers,
    short[] Motion, TouchPoint[] Touch, Dictionary<int, long> Reports, Dictionary<int, long> Arrivals,
    ButtonEvent[] Events, int QueuePeak, long Sequence, string Status, string? Error,
    string Keyboard, string VendorNotice, double QueueDelayMs);

internal sealed class InputState
{
    internal static readonly ButtonSpec[] Specs = [
        new("Up"), new("Down"), new("Left"), new("Right"),
        new("Square"), new("Cross"), new("Circle"), new("Triangle"),
        new("L1"), new("R1"), new("L2"), new("R2"), new("L3"), new("R3"),
        new("Create"), new("Options"), new("PS"), new("Touch click"),
        new("L outer", "L", 0x0F), new("L inner", "P", 0x13),
        new("R inner", "O", 0x12), new("R outer", "K", 0x0E),
        new("L SAX", "I", 0x0C), new("R SAX", "J", 0x0D),
        new("G1", "U", 0x18), new("G2", "H", 0x0B), new("G3", "V", 0x19),
        new("G4", "B", 0x05), new("G5", "N", 0x11)
    ];
    private readonly object gate = new();
    private readonly Dictionary<string, ButtonState> buttons = Specs.ToDictionary(x => x.Name, _ => new ButtonState(false, 0, 0));
    private readonly Queue<ButtonEvent> events = new();
    private readonly double[] sticks = new double[4];
    private readonly byte[] triggers = new byte[2];
    private short[] motion = new short[6]; // gyro then acceleration (raw device axes)
    private TouchPoint[] touch = [new(false, 0, 0), new(false, 0, 0)];
    private readonly Dictionary<int, long> reports = new(), arrivals = new();
    private long sequence;
    private int queuePeak;
    private string status = "Opening device…", keyboard = "--", notice = "--";
    private string? error;
    internal bool Ps { get; set; }

    internal void SetStatus(string value) { lock (gate) status = value; }
    internal void Fail(Exception e)
    {
        Console.Error.WriteLine(e);
        lock (gate) { error = error is null ? e.Message : error + " | " + e.Message; status = "FATAL — acquisition stopped"; }
    }
    private void SetButton(string name, bool down, long time)
    {
        var old = buttons[name];
        if (old.Down == down) return;
        if (events.Count >= 4096) throw new IOException("Button FIFO overflow (4096): capture stopped; no silent event loss");
        events.Enqueue(new(++sequence, time, name, down));
        queuePeak = Math.Max(queuePeak, events.Count);
        buttons[name] = new(down, down ? time : old.LastPress, old.Presses + (down ? 1 : 0));
    }
    internal Frame TakeFrame()
    {
        lock (gate)
        {
            double delay = events.TryPeek(out var first) ? Stopwatch.GetElapsedTime(first.Time).TotalMilliseconds : 0;
            var batch = events.ToArray(); events.Clear();
            return new(new(buttons), (double[])sticks.Clone(), (byte[])triggers.Clone(), (short[])motion.Clone(),
                (TouchPoint[])touch.Clone(), new(reports), new(arrivals), batch, queuePeak, sequence,
                status, error, keyboard, notice, delay);
        }
    }
    private static short I16(ReadOnlySpan<byte> r, int at) => BinaryPrimitives.ReadInt16LittleEndian(r[at..]);
    private static ushort U16(ReadOnlySpan<byte> r, int at) => BinaryPrimitives.ReadUInt16LittleEndian(r[at..]);
    private static void Require(bool condition, string message) { if (!condition) throw new IOException(message); }
    internal void Accept(int source, byte[] report, byte session = 0)
    {
        long now = Stopwatch.GetTimestamp();
        lock (gate)
        {
            ReadOnlySpan<byte> r = report;
            reports[source] = reports.GetValueOrDefault(source) + 1;
            arrivals[source] = now;
            if (source == 5)
            {
                Require(r.Length == 20 && r[0] == 0 && r[1] == 20, "Unexpected PC gamepad packet");
                string[] byte2 = ["Up", "Down", "Left", "Right", "Options", "Create", "L3", "R3"];
                for (int i = 0; i < 8; i++) SetButton(byte2[i], (r[2] & 1 << i) != 0, now);
                foreach (var (bit, key) in new[] { (0,"L1"), (1,"R1"), (2,"PS"), (4,"Cross"), (5,"Circle"), (6,"Square"), (7,"Triangle") })
                    SetButton(key, (r[3] & 1 << bit) != 0, now);
                SetTriggers(r[4], r[5], now);
                for (int i = 0; i < 4; i++) { short v = I16(r, 6 + i * 2); sticks[i] = v / (v >= 0 ? 32767.0 : 32768.0); }
            }
            else if (Ps && source == 0)
            {
                Require(r.Length == 64 && r[0] == 1, "Unexpected PS gamepad packet");
                string[] face = ["Square", "Cross", "Circle", "Triangle"];
                for (int i = 0; i < 4; i++) SetButton(face[i], (r[8] & 1 << (i+4)) != 0, now);
                string[] byte9 = ["L1", "R1", "L2", "R2", "Create", "Options", "L3", "R3"];
                for (int i = 0; i < 8; i++) if (i is not (2 or 3)) SetButton(byte9[i], (r[9] & 1 << i) != 0, now);
                int hat = r[8] & 15;
                SetButton("Up", hat is 7 or 0 or 1, now); SetButton("Right", hat is 1 or 2 or 3, now);
                SetButton("Down", hat is 3 or 4 or 5, now); SetButton("Left", hat is 5 or 6 or 7, now);
                SetButton("PS", (r[10]&1)!=0, now); SetButton("Touch click", (r[10]&2)!=0, now);
                SetTriggers(r[5], r[6], now);
                for (int i = 0; i < 4; i++) sticks[i] = (r[1+i] - 127.5) / 127.5 * (i % 2 == 1 ? -1 : 1);
                for (int i = 0; i < 6; i++) motion[i] = I16(r, 16 + i*2);
                for (int i = 0; i < 2; i++) { int p = 32+i*4; touch[i] = new((r[p]&128)==0, r[p+1]|(r[p+2]&15)<<8, (r[p+2]>>4)|r[p+3]<<4); }
            }
            else if (!Ps && source == 0)
            {
                byte[] p = Wire.Unpack(r);
                Require((p[1]&15)==3 && (session==0 || p[1]>>4==session), "Unexpected vendor notification/session");
                int id = U16(p, 2);
                if (id == 0x1B)
                {
                    Require(p.Length==16, "Invalid IMU payload length");
                    for (int i=0;i<3;i++) { motion[i]=I16(p,10+i*2); motion[i+3]=I16(p,4+i*2); }
                }
                else notice = $"0x{id:X2}: {Convert.ToHexString(p)}"; // Visible diagnostics, no guessed semantics.
            }
            else if (!Ps && source == 1)
            {
                Require(r.Length>0, "Empty keyboard/touch report");
                if (r[0]==1 && r.Length==17)
                {
                    var pressed = report.Skip(2).Where(x=>x!=0).ToHashSet();
                    Require(!pressed.Overlaps(new byte[]{1,2,3}), "Keyboard reports rollover/error");
                    foreach (var spec in Specs.Where(x=>x.Usage!=0)) SetButton(spec.Name, pressed.Contains(spec.Usage), now);
                    keyboard = $"modifier 0x{r[1]:X2}   usages {string.Join(' ', pressed.Order().Select(x=>$"{x:X2}"))}";
                }
                else if (r[0]==2 && r.Length==15)
                {
                    for (int i=0;i<2;i++) { int p=1+i*5; touch[i]=new((r[p]&2)!=0,U16(r,p+1),U16(r,p+3)); }
                    SetButton("Touch click", (r[14]&1)!=0, now);
                }
                else if (r[0]!=3 || r.Length!=2) throw new IOException($"Unknown PC HID report: {Convert.ToHexString(report)}");
            }
            else notice = $"IF {source}: {Convert.ToHexString(report)}";
        }
    }
    private void SetTriggers(byte left, byte right, long now)
    {
        triggers[0]=left; triggers[1]=right;
        SetButton("L2", left>0, now); SetButton("R2", right>0, now);
    }
}
