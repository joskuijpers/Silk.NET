using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Drawing;
using System.Numerics;
using System.Runtime.CompilerServices;
using Silk.NET.Input;
using Silk.NET.Windowing;

namespace DemoUtilities
{
    using KeySet = QuickSet<Key, KeyComparer>;
    using MouseButtonSet = QuickSet<MouseButton, MouseButtonComparer>;
    struct KeyComparer : IEqualityComparerRef<Key>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref Key a, ref Key b)
        {
            return a == b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref Key item)
        {
            return (int)item;
        }
    }
    struct MouseButtonComparer : IEqualityComparerRef<MouseButton>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref MouseButton a, ref MouseButton b)
        {
            return a == b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref MouseButton item)
        {
            return (int)item;
        }
    }
    public class Input : IDisposable
    {
        private IInputContext context;
        private IView window;

        //You could use GetState-like stuff to avoid the need to explicitly grab these, but shrug. This keeps events localized to just the window, and we can do a little logic of our own.
        KeySet anyDownedKeys;
        KeySet downedKeys;
        KeySet previousDownedKeys;
        MouseButtonSet anyDownedButtons;
        MouseButtonSet downedButtons;
        MouseButtonSet previousDownedButtons;
        BufferPool pool;
        public QuickList<char> TypedCharacters;

        /// <summary>
        /// Forces the mouse to stay at the center of the screen by recentering it on every flush.
        /// </summary>
        public bool MouseLocked
        {
            get; set;
        }
        Int2 WindowCenter { get { return new Int2(window.Size.X / 2, window.Size.Y / 2); } }

        /// <summary>
        /// Gets or sets the mouse position in window coordinates without changing the net mouse delta.
        /// </summary>
        public Int2 MousePosition
        {
            get
            {
                var state = context.Mice[0].Position;
                var clientPosition = window.PointToClient(new((int)state.X, (int)state.Y));
                return new Int2 { X = clientPosition.X, Y = clientPosition.Y };
            }
            set
            {
                //Note that changing the cursor position does not change the raw mouse x/y.
                var screen = window.PointToScreen(new(value.X, value.Y));
                context.Mice[0].Position = (Vector2) screen;
            }
        }

        /// <summary>
        /// Gets the change in mouse position since the previous flush.
        /// </summary>
        public Int2 MouseDelta
        {
            get
            {
                return mouseDelta;
            }
        }

        /// <summary>
        /// Gets the amount of upward mouse wheel scrolling since the last flush regardless of how much downward scrolling occurred.
        /// </summary>
        public float ScrolledUp { get; private set; }
        /// <summary>
        /// Gets the amount of downward mouse wheel scrolling since the last flush regardless of how much upward scrolling occurred.
        /// </summary>
        public float ScrolledDown { get; private set; }
        /// <summary>
        /// Gets the mouse wheel scroll delta since the last flush.
        /// </summary>
        public float ScrollDelta { get { return ScrolledUp + ScrolledDown; } }

        public Input(Window window, BufferPool pool)
        {
            this.window = window.View;
            context = window.View.CreateInput();
            context.Keyboards[0].KeyDown += KeyDown;
            context.Keyboards[0].KeyUp += KeyUp;
            context.Mice[0].MouseDown += MouseDown;
            context.Mice[0].MouseUp += MouseUp;
            context.Mice[0].Scroll += MouseWheel;
            context.Keyboards[0].KeyChar += KeyPress;
            this.pool = pool;
            anyDownedButtons = new MouseButtonSet(8, pool);
            downedButtons = new MouseButtonSet(8, pool);
            previousDownedButtons = new MouseButtonSet(8, pool);
            anyDownedKeys = new KeySet(8, pool);
            downedKeys = new KeySet(8, pool);
            previousDownedKeys = new KeySet(8, pool);
            TypedCharacters = new QuickList<char>(32, pool);
        }

        private void KeyPress(IKeyboard keyboard, char key)
        {
            TypedCharacters.Add(key, pool);
        }

        private void MouseWheel(IMouse mouse, ScrollWheel wheel)
        {
            if (wheel.Y > 0)
                ScrolledUp += wheel.Y;
            else
                ScrolledDown += wheel.Y;
        }

        private void MouseDown(IMouse mouse, MouseButton btn)
        {
            anyDownedButtons.Add(btn, pool);
            downedButtons.Add(btn, pool);
        }
        private void MouseUp(IMouse mouse, MouseButton btn)
        {
            downedButtons.FastRemove(btn);
        }

        private void KeyDown(IKeyboard keyboard, Key key, int scancode)
        {
            Console.WriteLine($"Down: {key}");
            anyDownedKeys.Add(key, pool);
            downedKeys.Add(key, pool);
            //Unfortunately, backspace isn't reported by keypress, so we do it manually.
            if (key == Key.Backspace)
                TypedCharacters.Add('\b', pool);
        }
        private void KeyUp(IKeyboard keyboard, Key key, int scancode)
        {
            Console.WriteLine($"Up: {key}");
            downedKeys.FastRemove(key);
        }



        /// <summary>
        /// Gets whether a key is currently pressed according to the latest event processing call.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was pressed in the latest event processing call, false otherwise.</returns>
        public bool IsDown(Key key)
        {
            return downedKeys.Contains(key);
        }

        /// <summary>
        /// Gets whether a key was down at the time of the previous flush.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was down at the time of the previous flush, false otherwise.</returns>
        public bool WasDown(Key key)
        {
            return previousDownedKeys.Contains(key);
        }

        /// <summary>
        /// Gets whether a down event occurred at any point between the previous flush and up to the last event process call for a key that was not down in the previous flush.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was pressed in the latest event processing call, false otherwise.</returns>
        public bool WasPushed(Key key)
        {
            return !previousDownedKeys.Contains(key) && anyDownedKeys.Contains(key);
        }


        /// <summary>
        /// Gets whether a button is currently pressed according to the latest event processing call.
        /// </summary>
        /// <param name="button">Button to check.</param>
        /// <returns>True if the button was pressed in the latest event processing call, false otherwise.</returns>
        public bool IsDown(MouseButton button)
        {
            return downedButtons.Contains(button);
        }

        /// <summary>
        /// Gets whether a button was down at the time of the previous flush.
        /// </summary>
        /// <param name="mouseButton">Button to check.</param>
        /// <returns>True if the button was down at the time of the previous flush, false otherwise.</returns>
        public bool WasDown(MouseButton mouseButton)
        {
            return previousDownedButtons.Contains(mouseButton);
        }

        /// <summary>
        /// Gets whether a down event occurred at any point between the previous flush and up to the last event process call for a button that was not down in the previous flush.
        /// </summary>
        /// <param name="button">Button to check.</param>
        /// <returns>True if the button was pressed in the latest event processing call, false otherwise.</returns>
        public bool WasPushed(MouseButton button)
        {
            return !previousDownedButtons.Contains(button) && anyDownedButtons.Contains(button);
        }

        Int2 mouseDelta;
        Int2 previousRawMouse;
        public void Start()
        {
            var currentState = context.Mice[0].Position;
            //Given a long enough time, this could theoretically hit overflow.
            //But that would require hours of effort with a high DPI mouse, and this is a demo application...
            mouseDelta.X = (int)currentState.X - previousRawMouse.X;
            mouseDelta.Y = (int)currentState.Y - previousRawMouse.Y;
            previousRawMouse = new Int2((int)currentState.X, (int)currentState.Y);
            if (MouseLocked)
            {
                //This is pretty doofy, but it works reasonably well and we don't have easy access to the windows-provided capture stuff through opentk (that I'm aware of?).
                //Could change it later if it matters, but realistically it won't matter.
                MousePosition = WindowCenter;
                context.Mice[0].Cursor.CursorMode = CursorMode.Hidden;
            }
            else
            {
                context.Mice[0].Cursor.CursorMode = CursorMode.Normal;
            }

        }
        public void End()
        {
            anyDownedKeys.Clear();
            anyDownedButtons.Clear();
            previousDownedKeys.Clear();
            previousDownedButtons.Clear();
            for (int i = 0; i < downedKeys.Count; ++i)
                previousDownedKeys.Add(downedKeys[i], pool);
            for (int i = 0; i < downedButtons.Count; ++i)
                previousDownedButtons.Add(downedButtons[i], pool);
            ScrolledDown = 0;
            ScrolledUp = 0;
            TypedCharacters.Count = 0;
        }

        /// <summary>
        /// Unhooks the input management from the window.
        /// </summary>
        public void Dispose()
        {
            context.Keyboards[0].KeyDown -= KeyDown;
            context.Keyboards[0].KeyUp -= KeyUp;
            context.Mice[0].MouseDown -= MouseDown;
            context.Mice[0].MouseUp -= MouseUp;

            anyDownedKeys.Dispose(pool);
            downedKeys.Dispose(pool);
            previousDownedKeys.Dispose(pool);
            anyDownedButtons.Dispose(pool);
            downedButtons.Dispose(pool);
            previousDownedButtons.Dispose(pool);
            context.Dispose();
        }
    }
}
