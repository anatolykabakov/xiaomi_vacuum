package com.xiaomi.robotjoy

import android.content.Context
import android.graphics.Bitmap
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.EditText
import android.widget.ImageView
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import java.net.ConnectException
import java.net.SocketTimeoutException
import java.util.Locale

class MainActivity : AppCompatActivity() {
    private lateinit var hostEditText: EditText
    private lateinit var portEditText: EditText
    private lateinit var mapPortEditText: EditText
    private lateinit var connectButton: Button
    private lateinit var statusTextView: TextView
    private lateinit var commandTextView: TextView
    private lateinit var occupancyMapView: ImageView
    private lateinit var joystickView: JoystickView

    private val mainHandler = Handler(Looper.getMainLooper())

    @Volatile
    private var client: RobotClient? = null

    @Volatile
    private var sessionThread: Thread? = null

    @Volatile
    private var allowMapUpdates: Boolean = false

    private var mapBitmap: Bitmap? = null

    @Volatile
    private var latestVx = 0f

    @Volatile
    private var latestVy = 0f

    @Volatile
    private var latestW = 0f

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        hostEditText = findViewById(R.id.hostEditText)
        portEditText = findViewById(R.id.portEditText)
        mapPortEditText = findViewById(R.id.mapPortEditText)
        connectButton = findViewById(R.id.connectButton)
        statusTextView = findViewById(R.id.statusTextView)
        commandTextView = findViewById(R.id.commandTextView)
        occupancyMapView = findViewById(R.id.occupancyMapView)
        joystickView = findViewById(R.id.joystickView)

        joystickView.onMove = { vector ->
            latestVx = (-vector.y).coerceIn(-1f, 1f)
            latestVy = vector.x.coerceIn(-1f, 1f)
            latestW = vector.x.coerceIn(-1f, 1f) * 0.6f
            renderCommand()
        }

        connectButton.setOnClickListener {
            if (client == null) {
                showToast("Connecting...")
                connect()
            } else {
                showToast("Disconnected")
                disconnect()
            }
        }
    }

    override fun onDestroy() {
        disconnect()
        super.onDestroy()
    }

    private fun mainLoop(session: RobotClient) {
        while (!Thread.currentThread().isInterrupted) {
            try {
                session.sendCmd(latestVx, latestVy, latestW)
                val map = session.getMap()
                if (map != null && allowMapUpdates) {
                    val bitmap = OccupancyMapBitmaps.fromProtoOrNull(map)
                    if (bitmap != null) {
                        mainHandler.post {
                            if (allowMapUpdates) {
                                applyMapBitmap(bitmap)
                            } else {
                                bitmap.recycle()
                            }
                        }
                    }
                }
            } catch (_: Exception) {
                break
            }
            try {
                Thread.sleep(50)
            } catch (_: InterruptedException) {
                break
            }
        }
    }

    private fun connect() {
        val (host, cmdPort, mapPort) = parseConnectionParams()
        if (host.isEmpty()) {
            showToast("Enter robot IP (e.g. 192.168.0.137)")
            return
        }
        if (!hasActiveNetwork()) {
            showToast("No network: turn on Wi‑Fi or mobile data")
            return
        }
        updateStatus(getString(R.string.status_disconnected))

        sessionThread?.interrupt()

        val t = Thread {
            var session: RobotClient? = null
            try {
                session = RobotClient(
                    host = host,
                    pushPort = cmdPort,
                    subPort = mapPort
                )
                session.sendCmd(0f, 0f, 0f)
                client = session
                mainHandler.post {
                    allowMapUpdates = true
                    updateStatus(getString(R.string.status_connected))
                    connectButton.text = getString(R.string.disconnect)
                    showToast("Connected")
                }
                mainLoop(session)
            } catch (ex: Exception) {
                mainHandler.post {
                    updateStatus(getString(R.string.status_disconnected))
                    showToast(formatConnectError(host, cmdPort, ex))
                }
            } finally {
                allowMapUpdates = false
                session?.close()
                mainHandler.post {
                    client = null
                    applyDisconnectedUi()
                }
            }
        }
        t.isDaemon = true
        sessionThread = t
        t.start()
    }

    private fun hasActiveNetwork(): Boolean {
        val cm = getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        val network = cm.activeNetwork ?: return false
        val caps = cm.getNetworkCapabilities(network) ?: return false
        return caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
    }

    private fun formatConnectError(host: String, port: Int, ex: Throwable): String {
        val detail = when (ex) {
            is ConnectException,
            is SocketTimeoutException ->
                "no route to $host:$port (same LAN? IP ok? xiaomi_robot listening?)"
            else -> ex.message ?: ex.javaClass.simpleName
        }
        return "Connection failed: $detail"
    }

    private fun parseConnectionParams(): Triple<String, Int, Int> {
        var host = hostEditText.text.toString().trim()
        val portField = portEditText.text.toString().trim()
        val mapPortField = mapPortEditText.text.toString().trim()
        var cmdPort = portField.toIntOrNull() ?: 9090
        val mapPort = mapPortField.toIntOrNull() ?: 9091

        host = host.removePrefix("http://").removePrefix("https://")
        val slash = host.indexOf('/')
        if (slash >= 0) {
            host = host.substring(0, slash)
        }
        host = host.trim()

        if (!host.startsWith("[") && host.count { it == ':' } == 1) {
            val idx = host.lastIndexOf(':')
            val maybePort = host.substring(idx + 1).toIntOrNull()
            if (maybePort != null) {
                cmdPort = maybePort
                host = host.substring(0, idx).trim()
            }
        }

        return Triple(host, cmdPort, mapPort)
    }

    private fun disconnect() {
        allowMapUpdates = false
        val tCmd = sessionThread
        sessionThread = null
        tCmd?.interrupt()
        try {
            tCmd?.join(3000)
        } catch (_: InterruptedException) {
        }
        client = null
        applyDisconnectedUi()
        clearMapDisplay()
    }

    private fun applyDisconnectedUi() {
        connectButton.text = getString(R.string.connect)
        updateStatus(getString(R.string.status_disconnected))
    }

    private fun applyMapBitmap(newBitmap: Bitmap) {
        mapBitmap?.recycle()
        mapBitmap = newBitmap
        occupancyMapView.setImageBitmap(newBitmap)
    }

    private fun clearMapDisplay() {
        occupancyMapView.setImageDrawable(null)
        mapBitmap?.recycle()
        mapBitmap = null
    }

    private fun updateStatus(text: String) {
        statusTextView.text = text
    }

    private fun showToast(text: String) {
        val len = if (text.length > 60) Toast.LENGTH_LONG else Toast.LENGTH_SHORT
        Toast.makeText(this, text, len).show()
    }

    private fun renderCommand() {
        commandTextView.text = String.format(
            Locale.US,
            "vx=%.2f vy=%.2f w=%.2f",
            latestVx,
            latestVy,
            latestW
        )
    }
}
