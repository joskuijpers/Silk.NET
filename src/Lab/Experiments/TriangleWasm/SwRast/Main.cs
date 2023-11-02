using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Runtime.Versioning;
using System.Text;
using Silk.NET.Input;
using Silk.NET.OpenGL;
using Silk.NET.Windowing;

namespace SoftwareRasterizer;

using static VectorMath;

[SupportedOSPlatform("windows")]
public static unsafe class Main
{
#if true
    const string SCENE = "Castle";
    const float FOV = 0.628f;
    static Vector3 g_cameraPosition = new(27.0f, 2.0f, 47.0f);
    static Vector3 g_cameraDirection = new(0.142582759f, 0.0611068942f, -0.987894833f);
    static Vector3 g_upVector = new(0.0f, 1.0f, 0.0f);
#else
    const string SCENE = "Sponza";
    const float FOV = 1.04f;
    static Vector3 g_cameraPosition = new(0.0f, 0.0f, 0.0f);
    static Vector3 g_cameraDirection = new(1.0f, 0.0f, 0.0f);
    static Vector3 g_upVector = new(0.0f, 0.0f, 1.0f);
#endif

    static RasterizationTable g_rasterizationTable;
    static Rasterizer g_rasterizer;

    static List<Occluder> g_occluders = new();
    private static GL g_gl;

    static byte* g_rawData;

    public static int Run()
    {
        var view = Window.GetView
            (ViewOptions.Default with { API = new GraphicsAPI(ContextAPI.OpenGLES, new APIVersion(3, 0)) });

        uint tex = 0, prog = 0, vao = 0;

        void CreateSizeDependentResources()
        {
            g_rawData = (byte*) NativeMemory.Alloc((nuint) (view.FramebufferSize.X * view.FramebufferSize.Y * 4));
        }

        IInputContext input = null!;
        view.Load += () =>
        {
            CreateSizeDependentResources();
            g_gl = view.CreateOpenGL();
            vao = g_gl.GenVertexArray();
            tex = g_gl.GenTexture();
            var vert = g_gl.CreateShader(ShaderType.VertexShader);
            g_gl.ShaderSource
            (
                vert,
                """
                #version 300 es
                precision highp float;
                out vec2 uv;
                void main()
                {
                    gl_Position = vec4(
                      gl_VertexID >= 2 ? -1.0 : 1.0,
                      (gl_VertexID % 2) == 0 ? -1.0 : 1.0,
                      0.0,
                      1.0
                      );
                    uv = (gl_Position.xy + vec2(1.0, 1.0)) / vec2(2.0, 2.0);
                }
                """
            );
            g_gl.CompileShader(vert);
            if (g_gl.GetShader(vert, ShaderParameterName.CompileStatus) != 1)
            {
                throw new(g_gl.GetShaderInfoLog(vert));
            }

            var frag = g_gl.CreateShader(ShaderType.FragmentShader);
            g_gl.ShaderSource
            (
                frag,
                """
                #version 300 es
                precision highp float;
                in vec2 uv;
                out mediump vec4 out_color;
                uniform sampler2D uTexture;
                void main()
                {
                    out_color = texture(uTexture, uv);
                }
                """
            );
            g_gl.CompileShader(frag);
            if (g_gl.GetShader(frag, ShaderParameterName.CompileStatus) != 1)
            {
                var len = (uint)g_gl.GetShader(frag, ShaderParameterName.InfoLogLength) + 1;
                Span<byte> log = stackalloc byte[(int)len];
                g_gl.GetShaderInfoLog(frag, len, out len, out log[0]);
                throw new(Encoding.UTF8.GetString(log[..(int)len]));
            }

            prog = g_gl.CreateProgram();
            g_gl.AttachShader(prog, vert);
            g_gl.AttachShader(prog, frag);
            g_gl.LinkProgram(prog);
            if (g_gl.GetProgram(prog, ProgramPropertyARB.LinkStatus) != 1)
            {
                var len = (uint)g_gl.GetProgram(prog, ProgramPropertyARB.InfoLogLength) + 1;
                Span<byte> log = stackalloc byte[(int)len];
                g_gl.GetProgramInfoLog(prog, len, out len, out log[0]);
                throw new(Encoding.UTF8.GetString(log[..(int)len]));
            }

            g_gl.DeleteShader(vert);
            g_gl.DeleteShader(frag);
            input = view.CreateInput();

            g_rasterizationTable = new RasterizationTable();
            g_rasterizer = V128Rasterizer<FmaIntrinsic>.Create
                (g_rasterizationTable, (uint) view.FramebufferSize.X, (uint) view.FramebufferSize.Y);
            uint[] inputIndices;
            {
                using var inFile = typeof(Main).Assembly.GetManifestResourceStream
                    ("TriangleWasm.SwRast.Castle.IndexBuffer.bin");

                long size = inFile.Length;
                long numIndices = size / sizeof(uint);

                inputIndices = new uint[numIndices];

                Span<byte> dst = MemoryMarshal.AsBytes(inputIndices.AsSpan());
                inFile.ReadAtLeast(dst, dst.Length);
            }

            Vector4* vertices;
            uint vertexCount;
            {
                using var inFile = typeof(Main).Assembly.GetManifestResourceStream
                    ("TriangleWasm.SwRast.Castle.VertexBuffer.bin");

                long size = inFile.Length;
                vertexCount = (uint) (size / sizeof(Vector4));

                uint vertexByteCount = vertexCount * (uint) sizeof(Vector4);
                vertices = (Vector4*) NativeMemory.Alloc(vertexByteCount);

                Span<byte> dst = new(vertices, (int) vertexByteCount);
                inFile.ReadAtLeast(dst, dst.Length);
            }

            List<uint> indexList = QuadDecomposition.decompose
            (
                inputIndices,
                new ReadOnlySpan<Vector4>(vertices, (int) vertexCount)
            );


            // Pad to a multiple of 8 quads
            while (indexList.Count % 32 != 0)
            {
                indexList.Add(indexList[0]);
            }

            ReadOnlySpan<uint> indices = CollectionsMarshal.AsSpan(indexList);

            int quadAabbCount = indices.Length / 4;
            nuint quadAabbByteCount = (nuint) quadAabbCount * (uint) sizeof(Aabb);
            Aabb* quadAabbs = (Aabb*) NativeMemory.AlignedAlloc(quadAabbByteCount, (uint) sizeof(Vector4));
            for (int quadIndex = 0; quadIndex < quadAabbCount; ++quadIndex)
            {
                Aabb aabb = new();
                aabb.include(vertices[indices[4 * quadIndex + 0]]);
                aabb.include(vertices[indices[4 * quadIndex + 1]]);
                aabb.include(vertices[indices[4 * quadIndex + 2]]);
                aabb.include(vertices[indices[4 * quadIndex + 3]]);
                quadAabbs[quadIndex] = aabb;
            }

            List<SurfaceAreaHeuristic.Vector> batchAssignment = new();
            uint* batchAssignmentPtr = SurfaceAreaHeuristic.generateBatches
                (new ReadOnlySpan<Aabb>(quadAabbs, quadAabbCount), 512, 8, batchAssignment);

            Aabb refAabb = new();
            for (uint i = 0; i < vertexCount; i++)
            {
                refAabb.include(vertices[i]);
            }

            // Bake occluders
            foreach (SurfaceAreaHeuristic.Vector batch in batchAssignment)
            {
                Vector4[] batchVertices = new Vector4[batch.Length * 4];
                for (int i = 0; i < batch.Length; i++)
                {
                    uint quadIndex = batch.Start[i];
                    batchVertices[i * 4 + 0] = vertices[(int) indices[(int) (quadIndex * 4 + 0)]];
                    batchVertices[i * 4 + 1] = vertices[(int) indices[(int) (quadIndex * 4 + 1)]];
                    batchVertices[i * 4 + 2] = vertices[(int) indices[(int) (quadIndex * 4 + 2)]];
                    batchVertices[i * 4 + 3] = vertices[(int) indices[(int) (quadIndex * 4 + 3)]];
                }

                g_occluders.Add(Occluder.Bake(batchVertices, refAabb.m_min, refAabb.m_max));
            }

            SurfaceAreaHeuristic.freeBatches(batchAssignmentPtr);
        };
        view.FramebufferResize += _ => CreateSizeDependentResources();
        view.Render += delta =>
        {
            Matrix4x4 projMatrix = XMMatrixPerspectiveFovLH
                (FOV, view.FramebufferSize.X / (float) view.FramebufferSize.Y, 1.0f, 5000.0f);
            Matrix4x4 viewMatrix = XMMatrixLookToLH(g_cameraPosition, g_cameraDirection, g_upVector);
            Matrix4x4 viewProjection = Matrix4x4.Multiply(viewMatrix, projMatrix);

            float* mvp = stackalloc float[16];
            Unsafe.CopyBlockUnaligned(mvp, &viewProjection, 64);

            long raster_start = Stopwatch.GetTimestamp();
            g_rasterizer.clear();
            g_rasterizer.setModelViewProjection(mvp);

            // Sort front to back
            Algo.sort
                (CollectionsMarshal.AsSpan(g_occluders), new OccluderComparerV128(new Vector4(g_cameraPosition, 0)));

            int clips = 0;
            int notClips = 0;
            int misses = 0;
            foreach (ref readonly Occluder occluder in CollectionsMarshal.AsSpan(g_occluders))
            {
                if (g_rasterizer.queryVisibility(occluder.m_boundsMin, occluder.m_boundsMax, out bool needsClipping))
                {
                    if (needsClipping)
                    {
                        g_rasterizer.rasterize<Rasterizer.NearClipped>(occluder);
                        clips++;
                    }
                    else
                    {
                        g_rasterizer.rasterize<Rasterizer.NotNearClipped>(occluder);
                        notClips++;
                    }
                }
                else
                {
                    misses++;
                }
            }

            long raster_end = Stopwatch.GetTimestamp();
            //Console.WriteLine(clips + " - " + notClips + " - " + misses);

            double rasterTime = Stopwatch.GetElapsedTime(raster_start, raster_end).TotalMilliseconds;

            samples.Enqueue(rasterTime);

            double avgRasterTime = samples.Sum() / samples.Count;
            double sqSum = samples.Select(x => x * x).Sum();
            double stDev = Math.Sqrt(sqSum / samples.Count - avgRasterTime * avgRasterTime);

            double median = samples.Order().ElementAt(samples.Count / 2);

            while (samples.Count > 60 * 10)
            {
                samples.Dequeue();
            }

            int fps = (int) (1000.0f / avgRasterTime);

            string title =
                $"FPS: {fps}      Rasterization time: {avgRasterTime:0.000}�{stDev:0.000}ms stddev / {median:0.000}ms median";
            Console.WriteLine(title);

            g_rasterizer.readBackDepth(g_rawData);

            g_gl.Clear(ClearBufferMask.ColorBufferBit);
            g_gl.BindVertexArray(vao);
            g_gl.UseProgram(prog);
            g_gl.ActiveTexture(TextureUnit.Texture0);
            g_gl.BindTexture(TextureTarget.Texture2D, tex);
            g_gl.TexImage2D
            (
                TextureTarget.Texture2D, 0, InternalFormat.Rgba8, (uint) view.FramebufferSize.X,
                (uint) view.FramebufferSize.Y, 0,
                PixelFormat.Rgba, PixelType.UnsignedByte, g_rawData
            );

            g_gl.DrawArrays(PrimitiveType.Triangles, 0, 3);
            g_gl.DrawArrays(PrimitiveType.Triangles, 1, 3);

            long now = Stopwatch.GetTimestamp();

            Vector3 right = Vector3.Normalize(Vector3.Cross(g_cameraDirection, g_upVector));
            float deltaTime = (float) Stopwatch.GetElapsedTime(lastPaint, now).TotalMilliseconds;
            float translateSpeed = 0.01f * deltaTime;
            float rotateSpeed = 0.002f * deltaTime;

            lastPaint = now;

            if (input.Keyboards[0].IsKeyPressed(Key.ShiftLeft))
                translateSpeed *= 3.0f;

            if (input.Keyboards[0].IsKeyPressed(Key.ControlLeft))
                translateSpeed *= 0.1f;

            if (input.Keyboards[0].IsKeyPressed(Key.W))
                g_cameraPosition = Vector3.Add(g_cameraPosition, Vector3.Multiply(g_cameraDirection, translateSpeed));

            if (input.Keyboards[0].IsKeyPressed(Key.S))
                g_cameraPosition = Vector3.Add(g_cameraPosition, Vector3.Multiply(g_cameraDirection, -translateSpeed));

            if (input.Keyboards[0].IsKeyPressed(Key.A))
                g_cameraPosition = Vector3.Add(g_cameraPosition, Vector3.Multiply(right, translateSpeed));

            if (input.Keyboards[0].IsKeyPressed(Key.D))
                g_cameraPosition = Vector3.Add(g_cameraPosition, Vector3.Multiply(right, -translateSpeed));

            if (input.Keyboards[0].IsKeyPressed(Key.Up))
                g_cameraDirection = Vector3.Transform
                    (g_cameraDirection, Quaternion.CreateFromAxisAngle(right, rotateSpeed));

            if (input.Keyboards[0].IsKeyPressed(Key.Down))
                g_cameraDirection = Vector3.Transform
                    (g_cameraDirection, Quaternion.CreateFromAxisAngle(right, -rotateSpeed));

            if (input.Keyboards[0].IsKeyPressed(Key.Left))
                g_cameraDirection = Vector3.Transform
                    (g_cameraDirection, Quaternion.CreateFromAxisAngle(g_upVector, -rotateSpeed));

            if (input.Keyboards[0].IsKeyPressed(Key.Right))
                g_cameraDirection = Vector3.Transform
                    (g_cameraDirection, Quaternion.CreateFromAxisAngle(g_upVector, rotateSpeed));

            if (input.Keyboards[0].IsKeyPressed(Key.R))
            {
                var previousRasterizer = g_rasterizer;
                //if (previousRasterizer is Avx2Rasterizer<FmaIntrinsic> or Avx2Rasterizer<FmaX86>)
                //{
                //    g_rasterizer = V128Rasterizer<FmaX86>.Create(g_rasterizationTable, (uint) view.FramebufferSize.X, (uint) view.FramebufferSize.Y);
                //}
                //else if (previousRasterizer is V128Rasterizer<FmaIntrinsic> or V128Rasterizer<FmaX86>)
                //{
                //    g_rasterizer = ScalarRasterizer.Create(g_rasterizationTable, (uint) view.FramebufferSize.X, (uint) view.FramebufferSize.Y);
                //}
                //else
                //{
                //    g_rasterizer = Avx2Rasterizer<FmaX86>.Create(g_rasterizationTable, (uint) view.FramebufferSize.X, (uint) view.FramebufferSize.Y);
                //}
                if (previousRasterizer is V128Rasterizer<FmaIntrinsic>)
                {
                    g_rasterizer = ScalarRasterizer.Create
                        (g_rasterizationTable, (uint) view.FramebufferSize.X, (uint) view.FramebufferSize.Y);
                }
                else
                {
                    g_rasterizer = V128Rasterizer<FmaIntrinsic>.Create
                        (g_rasterizationTable, (uint) view.FramebufferSize.X, (uint) view.FramebufferSize.Y);
                }

                previousRasterizer.Dispose();
                Console.WriteLine($"Changed to {g_rasterizer}  (from {previousRasterizer})");
            }
        };

        view.Run();
        if (view.IsClosing)
        {
            view.Dispose();
        }

        return 0;
    }

    static Queue<double> samples = new();

    static long lastPaint = Stopwatch.GetTimestamp();

    readonly struct OccluderComparerV128 : Algo.IComparer<Occluder>
    {
        public readonly Vector4 CameraPosition;

        public OccluderComparerV128(Vector4 cameraPosition)
        {
            CameraPosition = cameraPosition;
        }

        public bool Compare(in Occluder x, in Occluder y)
        {
            Vector128<float> dist1 = (x.m_center - CameraPosition).AsVector128();
            Vector128<float> dist2 = (y.m_center - CameraPosition).AsVector128();

            Vector128<float> a = V128Helper.DotProduct_x7F(dist1, dist1);
            Vector128<float> b = V128Helper.DotProduct_x7F(dist2, dist2);

            if (Sse.IsSupported)
            {
                return Sse.CompareScalarOrderedLessThan(a, b);
            }
            else
            {
                float sA = a.ToScalar();
                float sB = b.ToScalar();
                return !float.IsNaN(sA) && !float.IsNaN(sB) && sA < sB;
            }
        }
    }

    readonly struct ScalarOccluderComparer : Algo.IComparer<Occluder>
    {
        public readonly Vector4 CameraPosition;

        public ScalarOccluderComparer(Vector4 cameraPosition)
        {
            CameraPosition = cameraPosition;
        }

        public bool Compare(in Occluder x, in Occluder y)
        {
            Vector4 dist1 = (x.m_center - CameraPosition);
            Vector4 dist2 = (y.m_center - CameraPosition);

            float a = ScalarMath.DotProduct_x7F(dist1);
            float b = ScalarMath.DotProduct_x7F(dist2);

            return a < b;
        }
    }
}
