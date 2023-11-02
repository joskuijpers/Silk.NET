using DemoRenderer;
using DemoUtilities;
using System;
using BepuUtilities;
using BepuUtilities.Memory;
using Silk.NET.OpenGL;

namespace Demos
{
    public class GameLoop : IDisposable
    {
        public Window Window { get; private set; }
        public Input Input { get; private set; }
        public Camera Camera { get; private set; }
        public GL GL { get; private set; }
        public Renderer Renderer { get; private set; }
        public DemoHarness DemoHarness { get; set; }
        public BufferPool Pool { get; } = new BufferPool();

        public GameLoop(Window window)
        {
            Window = window;
            Input = new Input(window, Pool);
            GL = window.View.CreateOpenGL();
            Renderer = new Renderer(GL, window.View, window.Resolution);
            Camera = new Camera(window.Resolution.X / (float)window.Resolution.Y, (float)Math.PI / 3, 0.01f, 100000);            
        }

        void Update(float dt)
        {
            Input.Start();
            if (DemoHarness != null)
            {
                //We'll let the delegate's logic handle the variable time steps.
                DemoHarness.Update(dt);
                //At the moment, rendering just follows sequentially. Later on we might want to distinguish it a bit more with fixed time stepping or something. Maybe.
                DemoHarness.Render(Renderer);
            }
            Renderer.Render(Camera);
            Input.End();
        }

        public void Run(DemoHarness harness)
        {
            DemoHarness = harness;
            Window.Run(Update, OnResize);
        }

        private void OnResize(Int2 resolution)
        {
            //We just don't support true fullscreen in the demos. Would be pretty pointless.
            GL.Viewport(0, 0, (uint)resolution.X, (uint)resolution.Y);
            Camera.AspectRatio = resolution.X / (float)resolution.Y;
            DemoHarness?.OnResize(resolution);
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                Input.Dispose();
                Renderer.Dispose();
                Pool.Clear();
                //Note that we do not own the window.
            }
        }
    }
}
