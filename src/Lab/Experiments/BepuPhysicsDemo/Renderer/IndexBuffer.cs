using DemoUtilities;
using Silk.NET.OpenGL;

namespace DemoRenderer
{
    public class IndexBuffer : Disposable
    {
        private readonly uint buffer;
        public DrawElementsType Type => DrawElementsType.UnsignedInt;
        public readonly string DebugName;
        private readonly GL gl;

        public IndexBuffer(GL gl, uint[] indices, string debugName = "UNNAMED INDEX BUFFER")
        {
            this.gl = gl;
            buffer = gl.GenBuffer();
            gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, buffer);
            gl.BufferData<uint>(BufferTargetARB.ElementArrayBuffer, indices, BufferUsageARB.DynamicDraw);
            gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, 0);
            DebugName = debugName;
            if (gl.CanSetObjectLabel())
            {
                gl.ObjectLabel(ObjectIdentifier.Buffer, buffer, (uint)DebugName.Length, DebugName);
            }
        }
        public void Bind() => gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, buffer);
        protected override void DoDispose() => gl.DeleteBuffer(buffer);
    }
}
