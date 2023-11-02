using BepuUtilities;
using DemoContentLoader;
using DemoUtilities;
using Monitor = Silk.NET.Windowing.Monitor;

namespace Demos
{
    public class Program
    {
        public static void Main()
        {
            var res = Monitor.GetMainMonitor(null).VideoMode.Resolution ?? Monitor.GetMainMonitor(null).Bounds.Size;
            var window = new Window("pretty cool multicolored window",
                new Int2((int)(res.X * 0.75f), (int)(res.Y * 0.75f)), WindowMode.Windowed);
            var loop = new GameLoop(window);
            ContentArchive content;
            using (var stream = typeof(Program).Assembly.GetManifestResourceStream("BepuPhysicsDemo.Demos.contentarchive"))
            {
                content = ContentArchive.Load(stream);
            }
            //HeadlessTest.Test<ShapePileTestDemo>(content, 4, 32, 512);
            var demo = new DemoHarness(loop.GL, loop, content);
            loop.Run(demo);
            if (window.View.IsClosing)
            {
                // TODO loop.Dispose();
                window.Dispose();
            }
        }
    }
}
