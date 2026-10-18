// Headless stand-in for android.graphics.Bitmap (robot36 reference harness only).
// SPDX-License-Identifier: ISC
package android.graphics;

public class Bitmap {
    public enum Config { ARGB_8888 }

    public static Bitmap createBitmap(int width, int height, Config config) { return new Bitmap(); }

    public int getHeight() { return 0; }
}
