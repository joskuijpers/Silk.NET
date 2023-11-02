using System;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.CompilerServices;
using BepuUtilities;
using System.Threading;
using Silk.NET.Maths;
using Silk.NET.Windowing;
using Monitor = Silk.NET.Windowing.Monitor;
using Vector2 = System.Numerics.Vector2;

namespace DemoUtilities
{
    public enum WindowMode
    {
        FullScreen,
        Windowed
    }

    /// <summary>
    /// Simple and not-very-general-purpose window management.
    /// </summary>
    public class Window : IDisposable
    {
        internal IView View { get; private set; }

        private bool resized = true;

        private WindowMode windowMode;

        WindowMode WindowMode
        {
            get { return windowMode; }
            set
            {
                if (View is not IWindow proper)
                {
                    return;
                }

                switch (value)
                {
                    case WindowMode.FullScreen:
                        if (windowMode != WindowMode.FullScreen)
                        {
                            windowMode = value;
                            proper.WindowState = WindowState.Fullscreen;
                            proper.WindowBorder = WindowBorder.Hidden;
                            var primaryMonitor = Monitor.GetMainMonitor(proper);
                            proper.Position = primaryMonitor.Bounds.Origin;
                            proper.Size = primaryMonitor.VideoMode.Resolution ?? primaryMonitor.Bounds.Size;
                            resized = true;
                        }

                        break;
                    case WindowMode.Windowed:
                        if (windowMode != WindowMode.Windowed)
                        {
                            windowMode = value;
                            proper.WindowState = WindowState.Normal;
                            proper.WindowBorder = WindowBorder.Resizable;
                            resized = true;
                        }

                        break;
                }
            }
        }

        /// <summary>
        /// Gets or sets the resolution of the window's body.
        /// </summary>
        public Int2 Resolution
        {
            get { return new Int2(View.FramebufferSize.X, View.FramebufferSize.Y); }
            set
            {
                if (View is not IWindow proper)
                {
                    return;
                }

                proper.Size = new Vector2D<int>(value.X, value.Y) / (View.FramebufferSize / View.Size);
                resized = true;
            }
        }

        /// <summary>
        /// Constructs a new rendering-capable window.
        /// </summary>
        /// <param name="title">Title of the window.</param>
        /// <param name="resolution">Initial size in pixels of the window's drawable surface.</param>
        /// <param name="location">Initial location of the window's drawable surface.</param>
        /// <param name="windowMode">Initial window mode.</param>
        public Window(string title, Int2 resolution, Int2 location, WindowMode windowMode)
        {
            var viewOptions = ViewOptions.Default with
            {
                API = Silk.NET.Windowing.Window.IsViewOnly
                    ? new GraphicsAPI(ContextAPI.OpenGLES, new APIVersion(3, 0))
                    : GraphicsAPI.Default
            };


            View = Silk.NET.Windowing.Window.IsViewOnly
                ? Silk.NET.Windowing.Window.GetView(viewOptions)
                : Silk.NET.Windowing.Window.Create
                (
                    new WindowOptions(viewOptions)
                    {
                        Title = title, Size = Unsafe.As<Int2, Vector2D<int>>(ref resolution),
                        Position = Unsafe.As<Int2, Vector2D<int>>(ref location)
                    }
                );
            View.Initialize();
            Resolution = new(View.FramebufferSize.X, View.FramebufferSize.Y);
            View.FramebufferResize += _ => resized = true;
            View.Closing += OnClosing;
            WindowMode = windowMode;
        }

        /// <summary>
        /// Constructs a new rendering-capable window.
        /// </summary>
        /// <param name="title">Title of the window.</param>
        /// <param name="resolution">Initial size in pixels of the window's drawable surface.</param>
        /// <param name="windowMode">Initial window mode.</param>
        public Window(string title, Int2 resolution, WindowMode windowMode)
            : this
            (
                title, resolution,
                new Int2
                (
                    ((Monitor.GetMainMonitor(null).VideoMode.Resolution ?? Monitor.GetMainMonitor(null).Bounds.Size).X - resolution.X) / 2,
                    ((Monitor.GetMainMonitor(null).VideoMode.Resolution ?? Monitor.GetMainMonitor(null).Bounds.Size).Y - resolution.Y) / 2
                ), windowMode
            )
        {
        }

        public Vector2 GetNormalizedMousePosition(Int2 mousePosition)
        {
            return new Vector2((float) mousePosition.X / Resolution.X, (float) mousePosition.Y / Resolution.Y);
        }

        private void OnClosing()
        {
            //This will redundantly call window.Close, but that's fine.
            tryToClose = true;
        }

        private bool windowUpdateLoopRunning;
        private bool tryToClose;

        /// <summary>
        /// Closes the window at the next available opportunity.
        /// </summary>
        public void Close()
        {
            if (windowUpdateLoopRunning)
                tryToClose = true;
            else
                View.Close();
        }


        /// <summary>
        /// Launches the update loop for the window. Processes events before every invocation of the update handler.
        /// </summary>
        /// <param name="updateHandler">Delegate to be invoked within the loop repeatedly.</param>
        public void Run(Action<float> updateHandler, Action<Int2> onResize)
        {
            long previousTime = Stopwatch.GetTimestamp();
            windowUpdateLoopRunning = true;
            View.Render += delta =>
            {
                if (disposed)
                {
                    View.Reset();
                    return;
                }

                if (tryToClose)
                {
                    View.Close();
                    return;
                }
                if (resized)
                {
                    //Note that minimizing or resizing the window to invalid sizes don't result in actual resize attempts. Zero width rendering surfaces aren't allowed.
                    if (View.FramebufferSize is { X: > 0, Y: > 0 })
                    {
                        onResize(new Int2(View.FramebufferSize.X, View.FramebufferSize.Y));
                    }

                    resized = false;
                }
                if (View is not IWindow or IWindow { WindowState: not WindowState.Minimized })
                {
                    updateHandler((float)delta);
                }
                else
                {
                    //If the window is minimized, take a breather.
                    Thread.Sleep(1);
                }
            };

            View.Run();
            windowUpdateLoopRunning = !View.IsClosing;
        }

        private bool disposed;

        public void Dispose()
        {
            if (!disposed)
            {
                View.Dispose();
                disposed = true;
            }
        }
    }
}
