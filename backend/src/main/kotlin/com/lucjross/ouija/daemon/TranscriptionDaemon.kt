package com.lucjross.ouija.daemon

import com.lucjross.ouija.Mic
import com.lucjross.ouija.client.DeepgramClient
import kotlinx.coroutines.*
import io.github.oshai.kotlinlogging.KotlinLogging

/** Keeps a Deepgram session running and feeds each utterance to [OuijaState]. Reconnects when the session drops. */
class TranscriptionDaemon(
    private val deepgram: DeepgramClient,
    private val mic: Mic,
    private val ouijaState: OuijaState
) {

    private val logger = KotlinLogging.logger {}

    companion object {
        const val FIRST_RETRY_MS = 1_000L
        const val MAX_RETRY_MS = 30_000L
        // a session that lasted this long was healthy, so the next failure starts the backoff again
        const val HEALTHY_SESSION_MS = 60_000L
    }

    fun start(scope: CoroutineScope): Job = scope.launch(Dispatchers.IO) {
        var retryMs = FIRST_RETRY_MS
        while (isActive) {
            val started = System.nanoTime()
            try {
                logger.info { "starting stream transcription" }
                deepgram.stream(mic.chunks()) { ouijaState.heard(it) }
                logger.info { "stream transcription ended" }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                logger.warn(e) { "stream transcription failed: ${e.message}" }
            }

            val lastedMs = (System.nanoTime() - started) / 1_000_000
            if (lastedMs >= HEALTHY_SESSION_MS) retryMs = FIRST_RETRY_MS
            logger.info { "reconnecting in ${retryMs / 1000} s" }
            delay(retryMs)
            retryMs = (retryMs * 2).coerceAtMost(MAX_RETRY_MS)
        }
    }
}
