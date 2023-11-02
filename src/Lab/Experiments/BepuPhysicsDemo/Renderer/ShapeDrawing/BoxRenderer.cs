using DemoContentLoader;
using System.Numerics;
using System.Runtime.InteropServices;
using Silk.NET.OpenGL;

namespace DemoRenderer.ShapeDrawing
{
    //Could get this down to 32 bytes with some extra packing (e.g. 4 byte quaternion), but it would require some effort with really, really questionable gains.
    [StructLayout(LayoutKind.Explicit, Size = 48)]
    public struct BoxInstance
    {
        [FieldOffset(0)]
        public Vector3 Position;
        [FieldOffset(12)]
        public uint PackedColor;
        [FieldOffset(16)]
        public Quaternion Orientation;
        [FieldOffset(32)]
        public float HalfWidth;
        [FieldOffset(36)]
        public float HalfHeight;
        [FieldOffset(40)]
        public float HalfLength;
    }
    public class BoxRenderer(GL gl, ContentArchive content, int maximumInstancesPerDraw = 2048)
        : RasterizedRenderer<BoxInstance>(gl, content, @"ShapeDrawing\RenderBoxes", maximumInstancesPerDraw)
    {
        private readonly IndexBuffer indices = new(gl, Helpers.GetBoxIndices(maximumInstancesPerDraw), "Box Indices");

        protected override void OnDrawSetup() => indices.Bind();
        protected override unsafe void OnBatchDraw(int batchCount) =>
            gl.DrawElements(PrimitiveType.Triangles, 36 * (uint)batchCount, indices.Type, null);
        protected override void DoDispose()
        {
            base.DoDispose();
            indices.Dispose();
        }
    }
}
