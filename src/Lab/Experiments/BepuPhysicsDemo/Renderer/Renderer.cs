using System;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer.Background;
using DemoRenderer.UI;
using DemoRenderer.PostProcessing;
using DemoRenderer.ShapeDrawing;
using DemoRenderer.Constraints;
using BepuUtilities.Memory;
using DemoUtilities;
using Silk.NET.Core.Contexts;
using Silk.NET.OpenGL;

namespace DemoRenderer
{
    public class Renderer : Disposable
    {
        public Int2 Resolution { get; private set; }
        public BackgroundRenderer Background { get; private set; }
        //TODO: Down the road, the sphere renderer will be joined by a bunch of other types. 
        //They'll likely be stored in an array indexed by a shape type rather than just being a swarm of properties.
        public RayTracedRenderer<SphereInstance> SphereRenderer { get; private set; }
        public RayTracedRenderer<CapsuleInstance> CapsuleRenderer { get; private set; }
        public RayTracedRenderer<CylinderInstance> CylinderRenderer { get; private set; }
        public BoxRenderer BoxRenderer { get; private set; }
        public TriangleRenderer TriangleRenderer { get; private set; }
        public MeshRenderer MeshRenderer { get; private set; }
        public ShapesExtractor Shapes { get; private set; }
        public LineRenderer LineRenderer { get; private set; }
        public LineExtractor Lines { get; private set; }
        public ImageRenderer ImageRenderer { get; private set; }
        public GlyphRenderer GlyphRenderer { get; private set; }
        public UILineRenderer UILineRenderer { get; private set; }
        public CompressToSwap CompressToSwap { get; private set; }

        public ImageBatcher ImageBatcher { get; private set; }
        public TextBatcher TextBatcher { get; private set; }
        public UILineBatcher UILineBatcher { get; private set; }

        private readonly ParallelLooper looper = new ParallelLooper();
        private readonly BufferPool pool = new BufferPool();

        private readonly uint depthBuffer;
        //Technically we could get away with rendering directly to the backbuffer, but a dedicated color buffer simplifies some things- 
        //you aren't bound by the requirements of the swapchain's buffer during rendering, and post processing is nicer.
        //Not entirely necessary for the demos, but hey, you could add tonemapping if you wanted?
        private readonly uint colorBuffer;
        private readonly uint framebuffer;
        private readonly uint resolvedColorBuffer;
        private readonly uint resolvedFramebuffer;
        private int width = 0;
        private int height = 0;
        private readonly GL gl;

        public Renderer(GL gl, IGLContextSource contextSource, Int2 framebufferSize)
        {
            this.gl = gl;
            ContentArchive content;
            using (var stream = GetType().Assembly.GetManifestResourceStream("BepuPhysicsDemo.Renderer.contentarchive"))
            {
                content = ContentArchive.Load(stream);
            }
            depthBuffer = gl.GenTexture();
            colorBuffer = gl.GenTexture();
            framebuffer = gl.GenFramebuffer();
            resolvedColorBuffer = gl.GenTexture();
            resolvedFramebuffer = gl.GenFramebuffer();
            Shapes = new ShapesExtractor(gl, looper, pool);
            SphereRenderer = new RayTracedRenderer<SphereInstance>(gl, content, @"ShapeDrawing\RenderSpheres");
            CapsuleRenderer = new RayTracedRenderer<CapsuleInstance>(gl, content, @"ShapeDrawing\RenderCapsules");
            CylinderRenderer = new RayTracedRenderer<CylinderInstance>(gl, content, @"ShapeDrawing\RenderCylinders");
            BoxRenderer = new BoxRenderer(gl, content);
            TriangleRenderer = new TriangleRenderer(gl, content);
            MeshRenderer = new MeshRenderer(gl, Shapes.MeshCache, content);
            Lines = new LineExtractor(pool, looper);
            LineRenderer = new LineRenderer(gl, content);
            Background = new BackgroundRenderer(gl, content);
            CompressToSwap = new CompressToSwap(gl, content);

            ImageRenderer = new ImageRenderer(gl, content);
            ImageBatcher = new ImageBatcher(pool);
            GlyphRenderer = new GlyphRenderer(gl, content);
            TextBatcher = new TextBatcher();
            UILineRenderer = new UILineRenderer(gl, content);
            UILineBatcher = new UILineBatcher();

            OnResize(framebufferSize);
        }

        public unsafe void OnResize(Int2 resolution)
        {
            width = resolution.X;
            height = resolution.Y;

            TextBatcher.Resolution = resolution;
            ImageBatcher.Resolution = resolution;
            UILineBatcher.Resolution = resolution;

            if (!gl.GetStringS(StringName.Version).StartsWith("OpenGL ES"))
            {
                gl.BindFramebuffer(FramebufferTarget.Framebuffer, framebuffer);
                gl.BindTexture(TextureTarget.Texture2DMultisample, depthBuffer);
                gl.TexImage2DMultisample
                (
                    TextureTarget.Texture2DMultisample, 4u, InternalFormat.DepthComponent, (uint) width, (uint) height,
                    false
                );
                gl.FramebufferTexture2D
                (
                    FramebufferTarget.Framebuffer, FramebufferAttachment.DepthAttachment,
                    TextureTarget.Texture2DMultisample, depthBuffer, 0
                );
                //Using a 64 bit texture in the demos for lighting is pretty silly. But we gon do it.
                gl.BindTexture(TextureTarget.Texture2DMultisample, colorBuffer);
                gl.TexImage2DMultisample
                (
                    TextureTarget.Texture2DMultisample, 4u, InternalFormat.Rgba16f, (uint) width, (uint) height, false
                );
                gl.FramebufferTexture2D
                (
                    FramebufferTarget.Framebuffer, FramebufferAttachment.ColorAttachment0,
                    TextureTarget.Texture2DMultisample, colorBuffer, 0
                );
                gl.BindTexture(TextureTarget.Texture2DMultisample, 0);
            }

            gl.BindFramebuffer(FramebufferTarget.Framebuffer, resolvedFramebuffer);
            gl.BindTexture(TextureTarget.Texture2D, resolvedColorBuffer);
            gl.TexImage2D(GLEnum.Texture2D, 0, InternalFormat.Rgba16f, (uint)width, (uint)height, 0, PixelFormat.Rgba, PixelType.UnsignedByte, null);
            gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest);
            gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest);
            gl.FramebufferTexture2D(FramebufferTarget.Framebuffer, FramebufferAttachment.ColorAttachment0, TextureTarget.Texture2D, resolvedColorBuffer, 0);
            gl.BindTexture(TextureTarget.Texture2D, 0);

            gl.BindFramebuffer(FramebufferTarget.Framebuffer, 0);
        }

        public void Render(Camera camera)
        {
            var noMsaa = gl.GetStringS(StringName.Version).StartsWith("OpenGL ES");
            Shapes.MeshCache.FlushPendingUploads();

            gl.BindFramebuffer(FramebufferTarget.Framebuffer, noMsaa ? resolvedFramebuffer : framebuffer);
            //Note reversed depth.
            gl.ClearDepth(0.0f);
            gl.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            gl.Enable(EnableCap.CullFace);
            gl.Enable(EnableCap.DepthTest);
            gl.DepthFunc(DepthFunction.Greater);

            //All ray traced shapes use analytic coverage writes to get antialiasing.
            gl.Enable(EnableCap.SampleAlphaToCoverage);
            SphereRenderer.Render(camera, Resolution, SpanConverter.AsSpan(Shapes.ShapeCache.Spheres.Span), 0, Shapes.ShapeCache.Spheres.Count);
            CapsuleRenderer.Render(camera, Resolution, SpanConverter.AsSpan(Shapes.ShapeCache.Capsules.Span), 0, Shapes.ShapeCache.Capsules.Count);
            CylinderRenderer.Render(camera, Resolution, SpanConverter.AsSpan(Shapes.ShapeCache.Cylinders.Span), 0, Shapes.ShapeCache.Cylinders.Count);

            //Non-raytraced shapes just use regular opaque rendering.
            gl.Disable(EnableCap.SampleAlphaToCoverage);
            BoxRenderer.Render(camera, Resolution, SpanConverter.AsSpan(Shapes.ShapeCache.Boxes.Span), 0, Shapes.ShapeCache.Boxes.Count);
            TriangleRenderer.Render(camera, Resolution, SpanConverter.AsSpan(Shapes.ShapeCache.Triangles.Span), 0, Shapes.ShapeCache.Triangles.Count);
            MeshRenderer.Render(camera, Resolution, SpanConverter.AsSpan(Shapes.ShapeCache.Meshes.Span), 0, Shapes.ShapeCache.Meshes.Count);
            LineRenderer.Render(camera, Resolution, SpanConverter.AsSpan(Lines.lines.Span), 0, Lines.lines.Count);

            Background.Render(camera);
            gl.Disable(EnableCap.CullFace);
            gl.Disable(EnableCap.DepthTest);

            //Resolve MSAA rendering down to a single sample buffer for screenspace work.
            //Note that we're not bothering to properly handle tonemapping during the resolve. That's going to hurt quality a little, but the demos don't make use of very wide ranges.
            //(If for some reason you end up expanding the demos to make use of wider HDR, you can make this a custom resolve pretty easily.)
            if (!noMsaa)
            {
                gl.BindFramebuffer(FramebufferTarget.DrawFramebuffer, resolvedFramebuffer);
                gl.BlitFramebuffer(0, 0, width, height, 0, 0, width, height, ClearBufferMask.ColorBufferBit, BlitFramebufferFilter.Nearest);
                gl.BindFramebuffer(FramebufferTarget.DrawFramebuffer, 0);
                gl.BindFramebuffer(FramebufferTarget.Framebuffer, resolvedFramebuffer);
            }

            //Glyph and screenspace line drawing rely on the same premultiplied alpha blending transparency. We'll handle their state out here.
            gl.Enable(EnableCap.Blend);
            gl.BlendFunc(BlendingFactor.One, BlendingFactor.OneMinusSrcAlpha);
            gl.BlendEquation(BlendEquationModeEXT.FuncAdd);
            ImageRenderer.PreparePipeline();
            ImageBatcher.Flush(Resolution, ImageRenderer);
            UILineBatcher.Flush(Resolution, UILineRenderer);
            GlyphRenderer.PreparePipeline();
            TextBatcher.Flush(Resolution, GlyphRenderer);
            gl.Disable(EnableCap.Blend);

            gl.BindFramebuffer(FramebufferTarget.Framebuffer, 0);
            CompressToSwap.Render(resolvedColorBuffer);
        }

        protected override void DoDispose()
        {
            Background.Dispose();
            CompressToSwap.Dispose();

            Lines.Dispose();

            SphereRenderer.Dispose();
            CapsuleRenderer.Dispose();
            CylinderRenderer.Dispose();
            BoxRenderer.Dispose();
            TriangleRenderer.Dispose();
            MeshRenderer.Dispose();

            UILineRenderer.Dispose();
            GlyphRenderer.Dispose();

            gl.DeleteFramebuffer(framebuffer);
            gl.DeleteTexture(depthBuffer);
            gl.DeleteTexture(colorBuffer);
            gl.DeleteFramebuffer(resolvedFramebuffer);
            gl.DeleteTexture(resolvedColorBuffer);

            Shapes.Dispose();
        }
    }
}
