package com.xiaomi.robotjoy

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.hypot

class JoystickView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    data class Vector(val x: Float, val y: Float)

    var onMove: ((Vector) -> Unit)? = null

    private val basePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.parseColor("#334455") }
    private val knobPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.parseColor("#33B5E5") }

    private var centerX = 0f
    private var centerY = 0f
    private var baseRadius = 0f
    private var knobRadius = 0f
    private var knobX = 0f
    private var knobY = 0f

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        centerX = w / 2f
        centerY = h / 2f
        baseRadius = minOf(w, h) * 0.36f
        knobRadius = baseRadius * 0.35f
        resetKnob()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawCircle(centerX, centerY, baseRadius, basePaint)
        canvas.drawCircle(knobX, knobY, knobRadius, knobPaint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                moveKnob(event.x, event.y)
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                resetKnob()
                onMove?.invoke(Vector(0f, 0f))
                invalidate()
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    private fun moveKnob(x: Float, y: Float) {
        val dx = x - centerX
        val dy = y - centerY
        val distance = hypot(dx.toDouble(), dy.toDouble()).toFloat()
        val maxDistance = baseRadius - knobRadius

        if (distance > maxDistance && distance > 0f) {
            val scale = maxDistance / distance
            knobX = centerX + dx * scale
            knobY = centerY + dy * scale
        } else {
            knobX = x
            knobY = y
        }

        val normX = ((knobX - centerX) / maxDistance).coerceIn(-1f, 1f)
        val normY = ((knobY - centerY) / maxDistance).coerceIn(-1f, 1f)
        onMove?.invoke(Vector(normX, normY))
        invalidate()
    }

    private fun resetKnob() {
        knobX = centerX
        knobY = centerY
    }
}
