using System.Globalization;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace Volatile
{
    public struct Color : IEquatable<Color>, IFormattable
    {
        // Red component of the color.
        public float r;

        // Green component of the color.
        public float g;

        // Blue component of the color.
        public float b;

        // Alpha component of the color.
        public float a;


        // Constructs a new Color with given r,g,b,a components.
        public Color(float r, float g, float b, float a)
        {
            this.r = r;
            this.g = g;
            this.b = b;
            this.a = a;
        }

        // Constructs a new Color with given r,g,b components and sets /a/ to 1.
        public Color(float r, float g, float b)
        {
            this.r = r;
            this.g = g;
            this.b = b;
            this.a = 1.0F;
        }

        /// *listonly*
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public override string ToString()
        {
            return ToString(null, null);
        }

        // Returns a nicely formatted string of this color.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public string ToString(string format)
        {
            return ToString(format, null);
        }

        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            if (string.IsNullOrEmpty(format))
                format = "F3";
            if (formatProvider == null)
                formatProvider = CultureInfo.InvariantCulture.NumberFormat;
            return string.Format("RGBA({0}, {1}, {2}, {3})", r.ToString(format, formatProvider), g.ToString(format, formatProvider), b.ToString(format, formatProvider), a.ToString(format, formatProvider));
        }

        // used to allow Colors to be used as keys in hash tables
        public override int GetHashCode()
        {
            return r.GetHashCode() ^ (g.GetHashCode() << 2) ^ (b.GetHashCode() >> 2) ^ (a.GetHashCode() >> 1);
        }

        // also required for being able to use Colors as keys in hash tables
        public override bool Equals(object other)
        {
            if (other is Color color)
                return Equals(color);
            return false;
        }

        public bool Equals(Color other)
        {
            return r.Equals(other.r) && g.Equals(other.g) && b.Equals(other.b) && a.Equals(other.a);
        }

        // Solid red. RGBA is (1, 0, 0, 1).
        public static Color red { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(1F, 0F, 0F, 1F); } }
        // Solid green. RGBA is (0, 1, 0, 1).
        public static Color green { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(0F, 1F, 0F, 1F); } }
        // Solid blue. RGBA is (0, 0, 1, 1).
        public static Color blue { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(0F, 0F, 1F, 1F); } }
        // Solid white. RGBA is (1, 1, 1, 1).
        public static Color white { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(1F, 1F, 1F, 1F); } }
        // Solid black. RGBA is (0, 0, 0, 1).
        public static Color black { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(0F, 0F, 0F, 1F); } }
        // Yellow. RGBA is (1, 0.92, 0.016, 1), but the color is nice to look at!
        public static Color yellow { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(1F, 235F / 255F, 4F / 255F, 1F); } }
        // Cyan. RGBA is (0, 1, 1, 1).
        public static Color cyan { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(0F, 1F, 1F, 1F); } }
        // Magenta. RGBA is (1, 0, 1, 1).
        public static Color magenta { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(1F, 0F, 1F, 1F); } }
        // Gray. RGBA is (0.5, 0.5, 0.5, 1).
        public static Color gray { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(.5F, .5F, .5F, 1F); } }
        // English spelling for ::ref::gray. RGBA is the same (0.5, 0.5, 0.5, 1).
        public static Color grey { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(.5F, .5F, .5F, 1F); } }
        // Completely transparent. RGBA is (0, 0, 0, 0).
        public static Color clear { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return new Color(0F, 0F, 0F, 0F); } }

        // The grayscale value of the color (RO)
        public float grayscale { [MethodImpl(MethodImplOptionsEx.AggressiveInlining)] get { return 0.299F * r + 0.587F * g + 0.114F * b; } }

    }
}
