package com.xiaomi.robotjoy

import com.google.protobuf.InvalidProtocolBufferException
import com.xiaomi.robotjoy.proto.CmdVel
import com.xiaomi.robotjoy.proto.OccupancyMap
import com.xiaomi.robotjoy.proto.ZmqMessage
import org.zeromq.SocketType
import org.zeromq.ZContext
import org.zeromq.ZMQ

class RobotClient(
    private val host: String,
    private val pushPort: Int,
    private val subPort: Int = pushPort + 1,
) {
    private val context = ZContext()
    private val publishSocket: ZMQ.Socket = context.createSocket(SocketType.PUB)
    private val subSocket: ZMQ.Socket = context.createSocket(SocketType.SUB)

    @Volatile
    private var latestMap: OccupancyMap? = null

    @Volatile
    private var receiveThread: Thread? = null

    init {
        publishSocket.connect("tcp://${host.trim()}:$pushPort")

        subSocket.setReceiveTimeOut(250)
        subSocket.connect("tcp://${host.trim()}:$subPort")
        subSocket.subscribe(byteArrayOf())

        val t = Thread {
            try {
                while (!Thread.currentThread().isInterrupted) {
                    val raw = subSocket.recv() ?: continue
                    processMessage(raw)
                }
            } catch (_: Exception) {
            }
        }
        t.isDaemon = true
        receiveThread = t
        t.start()
    }

    private fun processMessage(raw: ByteArray) {
        if (raw.isEmpty()) return

        try {
            val zmqMsg = ZmqMessage.parseFrom(raw)
            if (zmqMsg.hasOccupancyMap()) {
                latestMap = zmqMsg.occupancyMap
            }
        } catch (_: InvalidProtocolBufferException) {
            // ignore non-protobuf or corrupt frames
        }
    }

    fun getMap(): OccupancyMap? = latestMap

    fun sendCmd(vx: Float, vy: Float, w: Float) {
        val cmdVel = CmdVel.newBuilder()
            .setVx(vx.toDouble())
            .setVy(vy.toDouble())
            .setW(w.toDouble())
            .build()

        val raw = ZmqMessage.newBuilder()
            .setTimestamp(nowMicros())
            .setTopic("cmd_vel")
            .setCmdVel(cmdVel)
            .build()
            .toByteArray()

        if (!publishSocket.send(raw, 0)) {
            throw RuntimeException("ZMQ send failed")
        }
    }

    /** Same unit as middleware::ServiceManager::Now() — microseconds since epoch. */
    private fun nowMicros(): Long = System.currentTimeMillis() * 1000L

    fun close() {
        receiveThread?.interrupt()
        receiveThread = null
        latestMap = null
        context.close()
    }
}
