package com.xiaomi.robotjoy

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.google.protobuf.InvalidProtocolBufferException
import com.xiaomi.robotjoy.proto.OccupancyMap

object OccupancyMapBitmaps {

    fun fromProtoOrNull(data: ByteArray): Bitmap? {
        val map = try {
            OccupancyMap.parseFrom(data)
        } catch (_: InvalidProtocolBufferException) {
            return null
        }
        return fromProtoOrNull(map)
    }

    fun fromProtoOrNull(map: OccupancyMap): Bitmap? {
        val w = map.width.toInt().coerceIn(1, 4096)
        val h = map.height.toInt().coerceIn(1, 4096)
        val cells = map.cells.toByteArray()
        val expected = w * h
        if (cells.size < expected) {
            return null
        }

        val pixels = IntArray(w * h)
        for (y in 0 until h) {
            for (x in 0 until w) {
                val v = cells[x + y * w].toInt() and 0xFF
                pixels[x + y * w] = cellToArgb(v)
            }
        }

        val bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        bitmap.setPixels(pixels, 0, w, 0, 0, w, h)

        val rbx = map.robotX.toInt()
        val rby = map.robotY.toInt()
        if (rbx in 0 until w && rby in 0 until h) {
            val c = Canvas(bitmap)
            val p = Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color = Color.RED
                style = Paint.Style.FILL
            }
            c.drawCircle(rbx + 0.5f, rby + 0.5f, 5f, p)
        }

        return bitmap
    }

    private fun cellToArgb(v: Int): Int {
        return when {
            v >= 255 -> Color.rgb(130, 130, 140)
            v >= 100 -> Color.rgb(25, 25, 30)
            else -> {
                val g = 245 - v * 2
                Color.rgb(g.coerceIn(180, 245), g.coerceIn(180, 245), g.coerceIn(180, 245))
            }
        }
    }
}
