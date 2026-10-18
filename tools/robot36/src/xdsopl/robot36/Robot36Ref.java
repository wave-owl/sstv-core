/*
 * Headless driver for robot36's decoder (reference harness, never part of libsstv): feeds a
 * mono 16-bit WAV file through xdsopl.robot36.Decoder exactly as MainActivity does
 * (MainActivity.java:228-247, 693-698: process() per block, an image is finished when
 * imageBuffer.line reaches imageBuffer.height; finish() at the end of input) and writes every
 * finished picture as OUT_PREFIX<k>.ppm.
 *
 *   java -jar robot36-ref.jar IN.wav OUT_PREFIX
 *
 * SPDX-License-Identifier: ISC
 */
package xdsopl.robot36;

import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Paths;

public class Robot36Ref {
    static int count = 0;

    static void store(PixelBuffer img, String prefix, String mode) throws IOException {
        String path = prefix + count + ".ppm";
        try (FileOutputStream out = new FileOutputStream(path)) {
            out.write(("P6\n" + img.width + " " + img.height + "\n255\n").getBytes());
            byte[] row = new byte[img.width * 3];
            for (int y = 0; y < img.height; ++y) {
                for (int x = 0; x < img.width; ++x) {
                    int p = img.pixels[y * img.width + x];
                    row[3 * x] = (byte) ((p >> 16) & 0xff);
                    row[3 * x + 1] = (byte) ((p >> 8) & 0xff);
                    row[3 * x + 2] = (byte) (p & 0xff);
                }
                out.write(row);
            }
        }
        System.out.println("image " + count + " " + mode + " " + img.width + "x" + img.height);
        count++;
    }

    public static void main(String[] args) throws IOException {
        byte[] wav = Files.readAllBytes(Paths.get(args[0]));
        ByteBuffer bb = ByteBuffer.wrap(wav).order(ByteOrder.LITTLE_ENDIAN);
        int rate = 0, pos = 12, dataPos = -1, dataLen = 0;
        while (pos + 8 <= wav.length) {
            String id = new String(wav, pos, 4);
            int len = bb.getInt(pos + 4);
            if (id.equals("fmt ")) rate = bb.getInt(pos + 12);
            if (id.equals("data")) { dataPos = pos + 8; dataLen = len; break; }
            pos += 8 + len + (len & 1);
        }
        PixelBuffer scope = new PixelBuffer(640, 2 * 1280);
        PixelBuffer image = new PixelBuffer(800, 616);
        Decoder decoder = new Decoder(scope, image, "Raw Mode", rate);
        int block = Math.max(1, rate / 50);
        int n = dataLen / 2;
        for (int off = 0; off < n; off += block) {
            int k = Math.min(block, n - off);
            float[] buf = new float[k];
            for (int i = 0; i < k; ++i) buf[i] = bb.getShort(dataPos + 2 * (off + i)) / 32768.f;
            if (decoder.process(buf, 0) && image.line >= image.height) {
                store(image, args[1], decoder.currentMode.getName());
                image.line = -1;
            }
        }
        if (decoder.finish(0) && image.line >= image.height) {
            store(image, args[1], decoder.currentMode.getName());
            image.line = -1;
        }
        System.out.println("images " + count);
    }
}
