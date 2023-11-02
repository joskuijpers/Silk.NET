using BepuUtilities;
using BepuUtilities.Collections;
using DemoContentLoader;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Silk.NET.OpenGL;

namespace DemoRenderer.ShapeDrawing
{
    [StructLayout(LayoutKind.Explicit, Size = 48)]
    public struct MeshInstance
    {
        [FieldOffset(0)] public Vector3 Position;
        [FieldOffset(12)] public uint PackedColor;
        [FieldOffset(16)] public ulong PackedOrientation;
        [FieldOffset(24)] public int VertexStart;
        [FieldOffset(28)] public int VertexCount;
        [FieldOffset(32)] public Vector3 Scale;
    }

    public class MeshRenderer(GL gl, MeshCache meshCache, ContentArchive content, int maximumInstancesPerDraw = 2048)
        : Shader
        (gl,
            content.Load<GLSLContent>(@"ShapeDrawing\RenderMeshes.glvs").Source,
            content.Load<GLSLContent>(@"ShapeDrawing\RenderMeshes.glfs").Source
        )
    {
        private readonly ConstantsBuffer<RasterizedVertexConstants> vertexConstants = new
            (gl, BufferTargetARB.UniformBuffer, debugName: $"Mesh Renderer Vertex Constants");

        private readonly StructuredBuffer<MeshInstance> instances = new
            (gl, BufferTargetARB.ShaderStorageBuffer, maximumInstancesPerDraw, $"Mesh Instances");

        public unsafe void Render
            (Camera camera, Int2 screenResolution, Span<MeshInstance> instances, int start, int count)
        {
            Use();
            vertexConstants.Bind(0);
            this.instances.Bind(0);
            meshCache.TriangleBuffer.Bind(1);

            //Examine the set of instances and batch them into groups using the same mesh data.
            var batches = new QuickDictionary<ulong, QuickList<MeshInstance>, PrimitiveComparer<ulong>>
                (16, meshCache.Pool);
            var end = start + count;
            for (int i = start; i < end; ++i)
            {
                ref var instance = ref instances[i];
                ref var id = ref Unsafe.As<int, ulong>(ref instance.VertexStart);

                if (batches.GetTableIndices(ref id, out var tableIndex, out var elementIndex))
                {
                    //The id was already present.
                    batches.Values[elementIndex].Add(instance, meshCache.Pool);
                }
                else
                {
                    //There is no batch for this vertex region, so create one.
                    var newCount = batches.Count + 1;
                    if (newCount > batches.Keys.Length)
                    {
                        batches.Resize(newCount, meshCache.Pool);
                        //Resizing will change the table indices, so we have to grab it again.
                        batches.GetTableIndices(ref id, out tableIndex, out _);
                    }

                    batches.Keys[batches.Count] = id;
                    ref var listSlot = ref batches.Values[batches.Count];
                    listSlot = new QuickList<MeshInstance>(64, meshCache.Pool);
                    listSlot.Add(instance, meshCache.Pool);
                    batches.Table[tableIndex] = newCount;
                    batches.Count = newCount;
                }
            }

            var vertexConstantsData = new RasterizedVertexConstants
            {
                Projection = camera.Projection,
                CameraPosition = camera.Position,
                CameraRight = camera.Right,
                CameraUp = camera.Up,
                CameraBackward = camera.Backward,
            };
            vertexConstants.Update(ref vertexConstantsData);

            for (int i = 0; i < batches.Count; ++i)
            {
                ref var batch = ref batches.Values[i];
                var batchVertexCount = batch[0].VertexCount;
                while (batch.Count > 0)
                {
                    var subbatchStart = Math.Max(0, batch.Count - this.instances.Capacity);
                    var subbatchCount = batch.Count - subbatchStart;
                    this.instances.Update(new Span<MeshInstance>(batch.Span.Memory + subbatchStart, subbatchCount));
                    gl.DrawArraysInstanced(PrimitiveType.Triangles, 0, (uint)batchVertexCount, (uint)subbatchCount);
                    batch.Count -= subbatchCount;
                }

                batch.Dispose(meshCache.Pool);
            }

            batches.Dispose(meshCache.Pool);
        }

        protected override void DoDispose()
        {
            instances.Dispose();
            vertexConstants.Dispose();
            base.DoDispose();
        }
    }
}
