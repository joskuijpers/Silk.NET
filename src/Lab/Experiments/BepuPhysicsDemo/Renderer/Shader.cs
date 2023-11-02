using System;
using Silk.NET.OpenGL;

namespace DemoRenderer
{
    public class Shader : Disposable
    {
        private readonly uint program;
        private readonly uint vao;
        private readonly GL gl;

        private void Compile(ShaderType type, string source, Action action)
        {
            var handle = gl.CreateShader(type);
            try
            {
                gl.ShaderSource(handle, source);
                gl.CompileShader(handle);
                var error = gl.GetShaderInfoLog(handle);
                if (error != string.Empty) throw new Exception(error);
                gl.AttachShader(program, handle);
                try
                {
                    action();
                }
                finally
                {
                    gl.DetachShader(program, handle);
                }
            }
            finally
            {
                gl.DeleteShader(handle);
            }
        }

        public Shader(GL gl, string vertex, string fragment)
        {
            this.gl = gl;
            program = gl.CreateProgram();
            vao = gl.GenVertexArray();
            Compile
            (
                ShaderType.VertexShader, vertex, () =>
                    Compile
                    (
                        ShaderType.FragmentShader, fragment, () =>
                        {
                            gl.LinkProgram(program);
                            var error = gl.GetProgramInfoLog(program);
                            if (error != string.Empty) throw new Exception(error);
                        }
                    )
            );
        }

        public void Use()
        {
            gl.UseProgram(program);
            gl.BindVertexArray(vao);
        }
        protected override void DoDispose()
        {
            gl.DeleteProgram(program);
            gl.DeleteVertexArray(vao);
        }
    }
}
